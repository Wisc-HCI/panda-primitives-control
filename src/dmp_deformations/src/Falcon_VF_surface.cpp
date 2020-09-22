#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <panda_ros_msgs/HybridPose.h>

#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>

// Force Dimension - for keyboard
#include "dhdc.h"

//Falcon
#include "falcon/core/FalconDevice.h"
#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"

#include "BSplineSurface.h"
#include "nlopt.hpp"
#include <iomanip>
#include <thread>
#include <ros/package.h>

using namespace std;
using namespace libnifalcon;
using namespace StamperKinematicImpl;
using namespace std::chrono;

FalconDevice m_falconDevice;

// For calculating the falcon velocity
array<double, 3> falcon_vel = {0.0, 0.0, 0.0};
std::array<double, 3> frozen_position = {0.0, 0.0, 0.0};
std::array<double, 3> last_falcon = {0.0, 0.0, 0.0};

array<double,4> q_normal = {0.0, 0.0, 0.0, 1.0};
array<double,4> q_prev = {0.0, 0.0, 0.0, 1.0};


// For the NL optimization
typedef struct {
    double x, y, z;
    BSplineSurface surface;
} opt_data;

array<double, 3> actual_pos;

bool last_falcon_updated = false;



// For recording
std::ofstream outputfile;
double x, y, z, fx, fy, fz;
double cx, cy, cz;
double dmp_fx, dmp_fy, dmp_fz;

bool replay_active = false;


/**
* Orientation interpolation
* based on https://en.wikipedia.org/wiki/Slerp
* interpolation, x, is 0 to 1 bounded
*/
array<double,4> slerp(array<double,4> q_start, array<double,4> q_end,double x){
    array<double,4> q_interp;
    
    // Make sure quaterions are properly normalized
    double mag_q_start = sqrt(q_start[0]*q_start[0]+q_start[1]*q_start[1]+q_start[2]*q_start[2]+q_start[3]*q_start[3]);
    q_start[0] = q_start[0]/mag_q_start; q_start[1] = q_start[1]/mag_q_start;
    q_start[2] = q_start[2]/mag_q_start; q_start[3] = q_start[3]/mag_q_start;

    double mag_q_end = sqrt(q_end[0]*q_end[0]+q_end[1]*q_end[1]+q_end[2]*q_end[2]+q_end[3]*q_end[3]);
    q_end[0] = q_end[0]/mag_q_end; q_end[1] = q_end[1]/mag_q_end;
    q_end[2] = q_end[2]/mag_q_end; q_end[3] = q_end[3]/mag_q_end;
    
    // Compute the cosine of the angle between the two vectors.
    // dot product for quaternions is normal vector inner product
    double dot = q_start[0]*q_end[0]+q_start[1]*q_end[1]+q_start[2]*q_end[2]+q_start[3]*q_end[3];

    // make sure the shorter path is chosen
    if (dot < 0.0f) {
        q_end[0] = -q_end[0]; q_end[1] = -q_end[1]; q_end[2] = -q_end[2]; q_end[3] = -q_end[3];
        dot = -dot;
    }

    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD) {
        // for close vectors, linear interpolation
        q_interp[0] = q_start[0]+x*(q_end[0]-q_start[0]);
        q_interp[1] = q_start[1]+x*(q_end[1]-q_start[1]);
        q_interp[2] = q_start[2]+x*(q_end[2]-q_start[2]);
        q_interp[3] = q_start[3]+x*(q_end[3]-q_start[3]);
        
        double mag_q_interp = sqrt(q_interp[0]*q_interp[0]+q_interp[1]*q_interp[1]+q_interp[2]*q_interp[2]+q_interp[3]*q_interp[3]);
        q_interp[0] = q_interp[0]/mag_q_interp; q_interp[1] = q_interp[1]/mag_q_interp;
        q_interp[2] = q_interp[2]/mag_q_interp; q_interp[3] = q_interp[3]/mag_q_interp;
        return q_interp;
    }

    double theta_0 = acos(dot);
    double theta = theta_0*x;
    double sin_theta = sin(theta);
    double sin_theta_0 = sin(theta_0);

    double s0 = cos(theta) - dot * sin_theta / sin_theta_0;
    double s1 = sin_theta / sin_theta_0;

    q_interp[0] = s0*q_start[0]+s1*q_end[0];
    q_interp[1] = s0*q_start[1]+s1*q_end[1];
    q_interp[2] = s0*q_start[2]+s1*q_end[2];
    q_interp[3] = s0*q_start[3]+s1*q_end[3];

    return q_interp;
}


void signalHandler(int sig)
{
    std::cout << "Interrupt " << sig << " recieved in ForceDimension.cpp\n";
    //cout << "Closing Force Dimension" << endl;
    //dhdClose();
    dhdEnableForce (DHD_OFF);
    dhdClose();
    exit(sig);
}

void falconVelocity() {
    double delta_T = 1000;
    // Compute the velocity to add some viscous friction to help with stability
    array<double, 3> falconPos = {0,0,0};
    m_falconDevice.runIOLoop();
    falconPos = m_falconDevice.getPosition();

    // Provide there is a last position to compute the velocity, calculate with backwards differencing
    if(last_falcon_updated)
    {
        // Velocity calculated as basic differencing
        falcon_vel = {(falconPos[0]-last_falcon[0])/delta_T,(falconPos[1]-last_falcon[1])/delta_T,(falconPos[2]-last_falcon[2])/delta_T};
    }

    //Update last falcon for use in the velocity calculation
    last_falcon[0] = falconPos[0];
    last_falcon[1] = falconPos[1];
    last_falcon[2] = falconPos[2];

    last_falcon_updated = true;
}

/**
 * Returns a boolean array for the current state of the four buttons on the falcon
 * 1 is pressed, 0 is unpressed
 */
array<bool,4> getButtons(){
    unsigned int my_buttons = m_falconDevice.getFalconGrip()->getDigitalInputs();
    array<bool,4> buttons;
    buttons[0] = (my_buttons & libnifalcon::FalconGripFourButton::CENTER_BUTTON)  ? 1 : 0;
    buttons[1] = (my_buttons & libnifalcon::FalconGripFourButton::PLUS_BUTTON)    ? 1 : 0;
    buttons[2] = (my_buttons & libnifalcon::FalconGripFourButton::MINUS_BUTTON)   ? 1 : 0;
    buttons[3] = (my_buttons & libnifalcon::FalconGripFourButton::FORWARD_BUTTON) ? 1 : 0;
    return buttons;
}

/**
* This function takes a rotation matrix expressed as 3 column vectors and creates
* a quaternion of the form x,y,z,w
*/
void rotationToQuaternion(array<double,3> x_hat, array<double,3> y_hat, array<double,3>z_hat, array<double,4> &q_out){
    // Using simple (non-optimal) formulation found here: https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
    // page 2

    double trace = x_hat[0] + y_hat[1] + z_hat[2];
    double k;

    // Positive trace - W is largest
    if(trace > 0.0){
        k = 0.5/(sqrt(1.0+trace));
        q_out = {k*(z_hat[1]-y_hat[2]), k*(x_hat[2]-z_hat[0]), k*(y_hat[0]-x_hat[1]), 0.25/k};
    }

    // X is largest
    else if ((x_hat[0]>y_hat[1]) && (x_hat[0]>z_hat[2])){
        k = 0.5/(sqrt(1.0+x_hat[0]-y_hat[1]-z_hat[2]));
        q_out = {0.25/k, k*(x_hat[1]+y_hat[0]), k*(x_hat[2]+z_hat[0]), k*(z_hat[1]-y_hat[2])};
    }

    // Y is largest
    else if (y_hat[1]>z_hat[2]){
        k = 0.5/(sqrt(1.0+y_hat[1]-x_hat[0]-z_hat[2]));
        q_out = {k*(x_hat[1]+y_hat[0]), 0.25/k, k*(y_hat[2]+z_hat[1]), k*(x_hat[2]-z_hat[0])};
    }

    // Z is largest
    else{
        k = 0.5/(sqrt(1.0+z_hat[2]-x_hat[0]-y_hat[1]));
        q_out = {k*(x_hat[2]+z_hat[0]), k*(y_hat[2]+z_hat[1]), 0.25/k, k*(y_hat[0]-x_hat[1])};
    }

    // Make sure it is normalized
    double mag = sqrt(q_out[0]*q_out[0]+q_out[1]*q_out[1]+q_out[2]*q_out[2]+q_out[3]*q_out[3]);
    q_out[0] = -q_out[0]/mag; q_out[1] = -q_out[1]/mag; q_out[2] = -q_out[2]/mag; q_out[3] = q_out[3]/mag;
}

static double obj_closest_surface(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    opt_data *d = (opt_data *) data;
    array<double,3> r;
    array<double,3> n_hat;
    array<double,3> r_u;
    array<double,3> r_v;
    d->surface.calculateSurfacePoint(x[0],x[1],r,n_hat,r_u,r_v);

    grad = std::vector<double>();
    grad.push_back(-2*(d->x-r[0])*r_u[0]-2*(d->y-r[1])*r_u[1]-2*(d->z-r[2])*r_u[2]);
    grad.push_back(-2*(d->x-r[0])*r_v[0]-2*(d->y-r[1])*r_v[1]-2*(d->z-r[2])*r_v[2]);

    return (r[0]-d->x)*(r[0]-d->x)+(r[1]-d->y)*(r[1]-d->y)+(r[2]-d->z)*(r[2]-d->z);
}

array<double,3> crossProduct(array<double,3> x, array<double,3> y){
    array<double,3> return_vec;
    return_vec[0]=x[1]*y[2]-x[2]*y[1];
    return_vec[1]=x[2]*y[0]-x[0]*y[2];
    return_vec[2]=x[0]*y[1]-x[1]*y[0];
    return return_vec;
}

void vfSurface(ros::NodeHandle n, string filename){
    std::string rospath = ros::package::getPath("dmp_deformations");
    BSplineSurface surface;

    // Find closest surface point if there is a surface to find!!
    if(filename!=""){    
        surface.loadSurface(rospath+"/../../devel/lib/dmp_deformations/"+filename+".csv");
        double u = 0.5;
        double v = 0.5;
        array<double,3> r;
        array<double,3> n_hat;
        array<double,3> r_u;
        array<double,3> r_v;

        // Find the closest point on the surface using NL-opt
        nlopt::opt opt(nlopt::LD_SLSQP, 2);
        
        // Bounds of surface are 0 to 1 in U,V directions
        std::vector<double> lb(2);
        lb[0] = 0.0; lb[1] = 0.0;
        std::vector<double> ub(2);
        ub[0] = 1.0; ub[1] = 1.0;
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);

        // Initial guess for parameters U,V
        std::vector<double> params(2);
        params[0] = u; params[1] = v;
        double minf;

        opt_data data;
        data.surface = surface;
        opt.set_xtol_rel(1e-2);
        opt.set_maxeval(20);

        ros::Publisher closest_pub = n.advertise<geometry_msgs::Vector3>("/vfclosest", 1);
        geometry_msgs::Vector3 loc;

        usleep(500000);

        while(1){
            auto start = high_resolution_clock::now(); 

            try{
                cout << "Start Opt" << endl;
                data.x=x; data.y=y; data.z=z;
                opt.set_min_objective(obj_closest_surface,&data);
                nlopt::result result = opt.optimize(params, minf);

                cout << "RES:" << result << endl;
                
                // Output new values if found
                u = params[0]; v=params[1];
            }
            catch(std::exception &e) {
                std::cout << "nlopt failed: " << e.what() << std::endl;
            }


            auto end = high_resolution_clock::now(); 
            auto duration = duration_cast<microseconds>(end - start);
            start = high_resolution_clock::now(); 
            surface.calculateSurfacePoint(u,v,r,n_hat,r_u,r_v);
            
            end = high_resolution_clock::now(); 
            duration = duration_cast<microseconds>(end - start);
            loc.x = r[0];
            loc.y = r[1];
            loc.z = r[2];
            cx= r[0]; cy = r[1]; cz=r[2];
            closest_pub.publish(loc);

            array<double,3> static_dir;

            if(filename=="layup2"){
                //switch direction for joint limits
                // currently rotates counterclockwise!
                static_dir[0] = -r_v[0];
                static_dir[1] = -r_v[1];
                static_dir[2] = -r_v[2];
            }

            else if(filename=="cowling4"){
                static_dir[0] = r_v[0];
                static_dir[1] = r_v[1];
                static_dir[2] = r_v[2];
            }

            
            rotationToQuaternion(static_dir,crossProduct(n_hat, static_dir), n_hat, q_normal);
            // cout << "X:" << r_v[0] << " " << r_v[1] << " "  << r_v[2] << endl; 
            // cout << "Y:" << temp[0] << " " << temp[1] << " "  << temp[2] << endl; 
            // cout << "Z:" << n_hat[0] << " " << n_hat[1] << " "  << n_hat[2] << endl; 
            cout << "Q:" << q_normal[0] << " " << q_normal[1] << " "  << q_normal[2] << " " << q_normal[3] << endl; 
            //cout << "CP:" << r[0] << " " << r[1] << " " << r[2] << " time:" << duration.count()/1000000.0 << endl;
            usleep(10000);
        }
    }
}

 
bool init_input() {
    cout <<"Setting up LibUSB\n";
    m_falconDevice.close();
    m_falconDevice.setFalconKinematic<FalconKinematicStamper>();
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>(); //Set Firmware

    if(!m_falconDevice.open(0)) //Open falcon @ index
    {
        cout << "Failed to find Falcon\n";
        return false;
    }
    else
    {
        cout << "Falcon Found\n";
    }

    bool skip_checksum = false;
    bool firmware_loaded = false;
    
    // MH: forcing the firmware to reload seems to solve the issue where
    // we had to keep re-plugging it in

    if(!firmware_loaded)
    {
        cout << "Loading firmware\n";

        uint8_t* firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
        long firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;

        for(int i = 0; i < 50; ++i)	//Attempt to load firmware 50 times
        {
            if(!m_falconDevice.getFalconFirmware()->loadFirmware(skip_checksum, firmware_size, firmware_block))
            {
                cout << "Firmware loading try failed\n";
            }
            else
            {
                firmware_loaded = true;
                break;
            }
        }
    }

    if(!firmware_loaded || !m_falconDevice.isFirmwareLoaded())
    {
        cout << "No firmware loaded to device, cannot continue\n";
        return false;
    }
    cout << "Firmware loaded\n";

    m_falconDevice.getFalconFirmware()->setHomingMode(true); //Set homing mode (keep track of encoders !needed!)
    cout << "Homing Set\n";

    m_falconDevice.runIOLoop(); //read in data

    bool stop = false;
    bool homing = false;
    usleep(100000);
    
    while(!stop)
    {
        if(!m_falconDevice.runIOLoop()) {
            continue;
        }
        if(!m_falconDevice.getFalconFirmware()->isHomed())
        {
            if(!homing)
            {
                m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
                cout << "Falcon not currently homed. Move control all the way out then push straight all the way in.\n";
            }
            homing = true;
        }

        if(homing && m_falconDevice.getFalconFirmware()->isHomed())
        {
            m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::GREEN_LED);
            cout << "Falcon homed.\n";
            stop = true;
        }
    }
    m_falconDevice.setFalconGrip<libnifalcon::FalconGripFourButton>();

    m_falconDevice.runIOLoop();
    
    return true;
}

void publishPose(ros::Publisher hybrid_pub, std::array<double, 7> panda_pose) {
    panda_ros_msgs::HybridPose hybridPose;
    hybridPose.pose.position.x = panda_pose[0];
    hybridPose.pose.position.y = panda_pose[1];
    hybridPose.pose.position.z = panda_pose[2];
    hybridPose.pose.orientation.x = panda_pose[3];
    hybridPose.pose.orientation.y = panda_pose[4];
    hybridPose.pose.orientation.z = panda_pose[5];
    hybridPose.pose.orientation.w = panda_pose[6];
    hybridPose.wrench.force.x = 0.0; hybridPose.wrench.force.y = 0.0; hybridPose.wrench.force.z = 0.0;
    hybridPose.constraint_frame.x=0.0;
    hybridPose.constraint_frame.y=0.0;
    hybridPose.constraint_frame.z=0.0;
    hybridPose.constraint_frame.w=1.0;
    hybridPose.sel_vector = {1, 1, 1, 1, 1, 1};
    hybrid_pub.publish(hybridPose);
}


void pollInput(ros::Publisher hybrid_pub, double* scaling_factors, double* offsets, bool* clutch, bool* reset_center, bool* freeze) {
    static bool lastCenterButton = false;
    array<double, 3> falconPos = {0,0,0};
    m_falconDevice.runIOLoop();
    falconPos = m_falconDevice.getPosition();

    std::array<double, 7> panda_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::array<double, 3> normalized_falcon = {0.0, 0.0, 0.0};
    
    // These convert the falcon to match the respective directions on the Kinova!
    normalized_falcon[0] = falconPos[2];
    normalized_falcon[1] = falconPos[0];
    normalized_falcon[2] = falconPos[1];

    // The first time the clutch is initiated, freeze the position of the falcon
    if (*freeze)
    {
        frozen_position[0] = normalized_falcon[0];
        frozen_position[1] = normalized_falcon[1];
        frozen_position[2] = normalized_falcon[2];
        *freeze = false;
    }

    // If still clutching, keep the position consistent based on the frozen position
    if(*clutch)
    {
        panda_pos[0] = scaling_factors[0] * frozen_position[0] + offsets[0];
        panda_pos[1] = scaling_factors[1] * frozen_position[1] + offsets[1];
        panda_pos[2] = scaling_factors[2] * frozen_position[2] + offsets[2];
        panda_pos[6] = 1;
    }

    else{ // Not clutching

        // If this is the first non-clutching sample, update the center of the workspace
        // based on the movement during the clutching action
        if(*reset_center)
        {
            offsets[0] = offsets[0]-scaling_factors[0]*(normalized_falcon[0]-frozen_position[0]);
            offsets[1] = offsets[1]-scaling_factors[1]*(normalized_falcon[1]-frozen_position[1]);
            offsets[2] = offsets[2]-scaling_factors[2]*(normalized_falcon[2]-frozen_position[2]);
            *reset_center = false;
        }

        // When not clutching, command based on the actual falcon position
        panda_pos[0] = scaling_factors[0] * normalized_falcon[0] + offsets[0];
        panda_pos[1] = scaling_factors[1] * normalized_falcon[1] + offsets[1];
        panda_pos[2] = scaling_factors[2] * normalized_falcon[2] + offsets[2];
        panda_pos[6] = 1;

        
    }

    // set the orientation based on the normal
    array<double,4> q_filt;
    q_filt = slerp(q_prev, q_normal,0.01);
    panda_pos[3] = q_filt[0];
    panda_pos[4] = q_filt[1];
    panda_pos[5] = q_filt[2];
    panda_pos[6] = q_filt[3];

    // save prev_orientation
    q_prev[0] = q_filt[0]; q_prev[1] = q_filt[1]; q_prev[2] = q_filt[2]; q_prev[3] = q_filt[3]; 

    // For NLopt thread
    x = panda_pos[0];
    y = panda_pos[1];
    z = panda_pos[2];
    
    publishPose(hybrid_pub,panda_pos);
}

void feedbackFalcon(geometry_msgs::Wrench wrench) {
    double scale = 0.1; // force reflection
    double viscous = 50; // friction
    double vf_stiffness = 0;
    double viscous_replay = 30; // TODO: want to get to 2 sqrt(stiffness)
    double stiffness = 100; // for replay
    double delta_T = 0.001;

    if(!replay_active){
        array<double, 3> falconPos = {0,0,0};
        m_falconDevice.runIOLoop();
        falconPos = m_falconDevice.getPosition();
        // Send force (bilateral + friction) to the falcon
        m_falconDevice.setForce({
                -wrench.force.y * scale-viscous*falcon_vel[0]+vf_stiffness*(y-cy), 
                wrench.force.z * scale-viscous*falcon_vel[1]-vf_stiffness*(z-cz), 
                wrench.force.x * scale-viscous*falcon_vel[2]+vf_stiffness*(x-cx)});
    }

    // For recording and hybrid replay
    fx = wrench.force.x;
    fy = wrench.force.y;
    fz = wrench.force.z;

}

void actualPose(geometry_msgs::Pose pose) {
    actual_pos[0] = pose.position.x;
    actual_pos[1] = pose.position.y;
    actual_pos[2] = pose.position.z;
}

int main(int argc, char **argv) {    
    ros::init(argc, argv, "Falcon");

    string filename="";
    if(argc>1){
        filename=argv[1];
    }

        //Setup the signal handler for exiting
    signal(SIGINT, signalHandler);

    std::vector<double> scaling_factors = {-4.0, -4.0, 4.0}; //-6,6,6
    std::vector<double> offsets = {0.4, 0.0, 0.25};

    // All of the required ros topics
    ros::init(argc, argv, "ForceDimensionDMP");
    ros::NodeHandle n("~");  
    ros::Publisher gripper_pub = 
        n.advertise<std_msgs::String>("/panda/commands", 5);
    ros::Subscriber force_sub = n.subscribe("/panda/wrench", 10, feedbackFalcon);
    ros::Publisher hybrid_pub = 
        n.advertise<panda_ros_msgs::HybridPose>("/panda/hybrid_pose", 1); 
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    if (!init_input()) {
        cout << endl << "Failed to init falcon" << endl;
        return -1;
    }

    bool buttonPressed = false;
    bool clutch = false;
    bool quit = false;
    bool freeze = false;
    bool reset_center = true;
    bool last_buttons[4] = {false, false, false, false};

    // Prevent Initial Discontinuity
    ros::spinOnce();

    try{
         geometry_msgs::TransformStamped transformStamped;
         transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_ee",ros::Time(0));
         offsets[0] = transformStamped.transform.translation.x;
         offsets[1] = transformStamped.transform.translation.y;
         offsets[2] = transformStamped.transform.translation.z;
         q_prev[0] = transformStamped.transform.rotation.x;
         q_prev[1] = transformStamped.transform.rotation.y;
         q_prev[2] = transformStamped.transform.rotation.z;
         q_prev[3] = transformStamped.transform.rotation.w;
         q_normal[0] = transformStamped.transform.rotation.x;
         q_normal[1] = transformStamped.transform.rotation.y;
         q_normal[2] = transformStamped.transform.rotation.z;
         q_normal[3] = transformStamped.transform.rotation.w;
       }

    catch(tf2::TransformException &ex){
        cout << "COULDN'T GET TF FROM PANDA" << endl << ex.what() << endl;
        }


    pollInput(hybrid_pub,scaling_factors.data(),offsets.data(), &clutch, &reset_center, &freeze);
    
    // Thread a process for the virtual fixture calculation
    std::thread t1(&vfSurface,n,filename);
    
    while(ros::ok() && (!quit)){      
        pollInput(hybrid_pub,scaling_factors.data(),offsets.data(), &clutch, &reset_center, &freeze);
        falconVelocity();
        
        // Check button presses
        array<bool,4> buttons = getButtons();
        
        if(buttons[0]!=last_buttons[0]){
            // Clutching functionality
            if(buttons[0]==1){
                clutch = true;
                freeze = true;
                cout << "Clutch On" << endl;
            }

            else{
                clutch = false;
                reset_center = true;
                cout << "Clutch Off" << endl;
            }
        }

        if(buttons[2]!=last_buttons[2]){
            if(buttons[2]==true){
                std_msgs::String temp_str;
                temp_str.data = "toggleGrip";
                gripper_pub.publish(temp_str);
            }
        }

        if (dhdKbHit()) {
            char keypress = dhdKbGet();
            if (keypress == 'q'){
                cout << "Quitting! " << endl;
                quit = true;
                } 
            }



        // Store previous buttons
        last_buttons[0] = buttons[0];
        last_buttons[1] = buttons[1];
        last_buttons[2] = buttons[2];
        last_buttons[3] = buttons[3];

        ros::spinOnce();
        usleep(1000);   
    }
    
    m_falconDevice.close();
    return 0;
}
