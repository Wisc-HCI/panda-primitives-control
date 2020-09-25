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

// Force Dimension
#include "dhdc.h"

#include "BSplineSurface.h"
#include "nlopt.hpp"
#include <iomanip>
#include <thread>
#include <ros/package.h>

using namespace std;
using namespace std::chrono;

std::array<double, 3> frozen_position = {0.0, 0.0, 0.0};
double x, y, z, fx, fy, fz;
double cx, cy, cz;

array<double,4> q_normal = {0.0, 0.0, 0.0, 1.0};
array<double,4> q_prev = {0.0, 0.0, 0.0, 1.0};

// For the NL optimization
typedef struct {
    double x, y, z;
    BSplineSurface surface;
} opt_data;

array<double, 3> actual_pos;

std::ofstream outputfile;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);


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
    
    if (filename=="layup2" || filename=="cowling4"){
        std::string rospath = ros::package::getPath("dmp_deformations");
        BSplineSurface surface;
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
    else if (filename=="fastener1")
    {
        array<double,3> pos;
        while (1){
            try{
                geometry_msgs::TransformStamped transformStamped;
                transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_ee",ros::Time(0));
                pos[0] = transformStamped.transform.translation.x;
                pos[1] = transformStamped.transform.translation.y;
                pos[2] = transformStamped.transform.translation.z;
            }

            catch(tf2::TransformException &ex){
                cout << "COULDN'T GET TF FROM PANDA" << endl << ex.what() << endl;
            }

            array<double,3> static_dir;
            array<double,3> n_hat = {0.0, 0.0, 1.0};
            if(pos[1]<0.1){
                static_dir = {0.0, 1.0, 0.0};
            }

            else{
                static_dir = {1.0, 0.0, 0.0};
            }

            rotationToQuaternion(static_dir,crossProduct(n_hat, static_dir), n_hat, q_normal);
            usleep(10000);
        }
    }
}

bool init_input() {
    cout <<"Setting up Force Dimension\n";
    if (dhdOpen () < 0) {
        cout << "Unable to open device" << endl;
        dhdSleep (2.0);
        return false;
    }

    // Turn on gravity compensation
    dhdEnableForce (DHD_ON);

    // Second button press to start control & forces
    bool buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    cout << "Press Button again to enable Forces and Panda Control" << endl;

    bool buttonPressed=false;
    while(!buttonPressed)
        if(dhdGetButton(0)==1) // button pressed
        {
            buttonPressed = true;
        }
    
    buttonReleased=false;
    while(!buttonReleased)
        if(dhdGetButton(0)==0) // button released
        {
            buttonReleased = true;
        }

    cout << "Force Dimension Setup Complete" << endl;
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
    array<double, 3> fdPos = {0,0,0};
    dhdGetPosition(&fdPos[0],&fdPos[1],&fdPos[2]);

    std::array<double, 7> panda_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::array<double, 3> normalized_falcon = {0.0, 0.0, 0.0};
    
    // These convert the falcon to match the respective directions on the FD - behind robot!
    //normalized_falcon[0] = fdPos[0];
    //normalized_falcon[1] = fdPos[1];
    //normalized_falcon[2] = fdPos[2];

    // in front of robot
    normalized_falcon[0] = -fdPos[0];
    normalized_falcon[1] = -fdPos[1];
    normalized_falcon[2] = fdPos[2];

    // The first time the clutch is initiated, freeze the position of the falcon
    if (*freeze)
    {
        frozen_position[0] = fdPos[0];
        frozen_position[1] = fdPos[1];
        frozen_position[2] = fdPos[2];
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
            offsets[0] = offsets[0]-scaling_factors[0]*(fdPos[0]-frozen_position[0]);
            offsets[1] = offsets[1]-scaling_factors[1]*(fdPos[1]-frozen_position[1]);
            offsets[2] = offsets[2]-scaling_factors[2]*(fdPos[2]-frozen_position[2]);
            *reset_center = false;
        }

        // When not clutching, command based on the actual falcon position
        panda_pos[0] = scaling_factors[0] * fdPos[0] + offsets[0];
        panda_pos[1] = scaling_factors[1] * fdPos[1] + offsets[1];
        panda_pos[2] = scaling_factors[2] * fdPos[2] + offsets[2];
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

double old_force_x = 0.0;
double old_force_y = 0.0;
double old_force_z = 0.0;


void feedbackInput(geometry_msgs::Wrench wrench) {
    double scale = 0.8; // force reflection
    double stiffness = 200; // for replay
    double viscous = 100; // friction

    // Filter forces
    double alpha = 0.9;
    double fx_interp = alpha*wrench.force.x + (1-alpha)*old_force_x;
    double fy_interp = alpha*wrench.force.y + (1-alpha)*old_force_y;
    double fz_interp = alpha*wrench.force.z + (1-alpha)*old_force_z;

    old_force_x = wrench.force.x;
    old_force_y = wrench.force.y;
    old_force_z = wrench.force.z;

    array<double, 3> forceDimensionPos = {0,0,0};
    array<double, 3> forceDimensionVel = {0,0,0};
    dhdGetLinearVelocity(&forceDimensionVel[0],&forceDimensionVel[1],&forceDimensionVel[2]);
    // Send force (bilateral + friction) to the falcon
    dhdSetForceAndTorque(-fx_interp * scale-viscous*forceDimensionVel[0], 
            -fy_interp * scale-viscous*forceDimensionVel[1], 
            fz_interp * scale-viscous*forceDimensionVel[2],0.0,0.0,0.0);
}

int main(int argc, char **argv) {
    //Setup the signal handler for exiting
    signal(SIGINT, signalHandler);

    std::vector<double> scaling_factors = {-4.0, -4.0, 4.0}; //-6,6,6
    std::vector<double> offsets = {0.4, 0.0, 0.25};

    // All of the required ros topics
    ros::init(argc, argv, "ForceDimensionDMP");

    string filename="";
    if(argc>1){
        filename=argv[1];
    }

    ros::NodeHandle n("~");  
    ros::Publisher gripper_pub = 
        n.advertise<std_msgs::String>("/panda/commands", 5);
    ros::Subscriber force_sub = n.subscribe("/panda/wrench", 10, feedbackInput);
    ros::Publisher hybrid_pub = 
        n.advertise<panda_ros_msgs::HybridPose>("/panda/hybrid_pose", 1); 


    if (!init_input()) {
        cout << endl << "Failed to init force dimension" << endl;
        return -1;
    }

    bool buttonPressed = false;
    bool clutch = false;
    auto start = high_resolution_clock::now(); 
    
    bool quit = false;
    bool freeze = false;
    bool reset_center = true;

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

    // Thread a process for the virtual fixture calculation
    std::thread t1(&vfSurface,n,filename);

    while (ros::ok() && !quit) {
        pollInput(hybrid_pub,scaling_factors.data(),offsets.data(), &clutch, &reset_center, &freeze);
    
        // If button pressed and released in less than 0.3 seconds,
        // it is a gripping action

        // If held greater than 0.3 seconds, it is a velocity control action

        if(!buttonPressed && dhdGetButton(0)==1) // button initially pressed
        {
            buttonPressed = true;
            start = high_resolution_clock::now(); 
        }

        
        if(buttonPressed) // button held
        {
            auto end = high_resolution_clock::now(); 
            auto duration = duration_cast<microseconds>(end - start); 

            if(duration.count()>=300000)
            {
                if (clutch==false)
                {
                    freeze = true;
                    clutch = true;
                }
               
            }
            
        }

        if(buttonPressed && dhdGetButton(0)==0) // button released
        {
            auto end = high_resolution_clock::now(); 
            auto duration = duration_cast<microseconds>(end - start); 

            if(duration.count()<300000)
            {
                cout << "Toggle Gripper" << endl;
                std_msgs::String temp_str;
                temp_str.data = "toggleGrip";
                gripper_pub.publish(temp_str);
            }

            buttonPressed=false;
        
            if (clutch==true){
                clutch=false;
                reset_center=true;
            }
        }

        if (dhdKbHit()) {
            char keypress = dhdKbGet();
            if (keypress == 'q'){
                cout << "Quitting! " << endl;
                quit = true;
                } 
        }
    
    ros::spinOnce();
    usleep(1000);
    }

    dhdEnableForce (DHD_OFF);
    dhdClose();
    return 0;
}
