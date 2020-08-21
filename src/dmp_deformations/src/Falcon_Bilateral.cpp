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

using namespace std;
using namespace libnifalcon;
using namespace StamperKinematicImpl;

FalconDevice m_falconDevice;

// For calculating the falcon velocity
array<double, 3> falcon_vel = {0.0, 0.0, 0.0};
std::array<double, 3> frozen_position = {0.0, 0.0, 0.0};
std::array<double, 3> last_falcon = {0.0, 0.0, 0.0};

array<double, 3> actual_pos;

bool last_falcon_updated = false;

// For recording
std::ofstream outputfile;
double x, y, z, fx, fy, fz;
double dmp_fx, dmp_fy, dmp_fz;

bool replay_active = false;

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

    std::array<double, 7> panda_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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

        // For recording
        x = panda_pos[0];
        y = panda_pos[1];
        z = panda_pos[2];
    }
    
    publishPose(hybrid_pub,panda_pos);
}

void feedbackFalcon(geometry_msgs::Wrench wrench) {
    double scale = 0.1; // force reflection
    double viscous = 50; // friction
    double viscous_replay = 30; // TODO: want to get to 2 sqrt(stiffness)
    double stiffness = 100; // for replay
    double delta_T = 0.001;

    if(!replay_active){
        array<double, 3> falconPos = {0,0,0};
        m_falconDevice.runIOLoop();
        falconPos = m_falconDevice.getPosition();
        // Send force (bilateral + friction) to the falcon
        m_falconDevice.setForce({
                -wrench.force.y * scale-viscous*falcon_vel[0], 
                wrench.force.z * scale-viscous*falcon_vel[1], 
                wrench.force.x * scale-viscous*falcon_vel[2]});
    }

    else{
        // zero displacement running in other process

        //  // zero displacement mode
        // // falcon has offset in z
        // m_falconDevice.setForce({
        //         -stiffness*falconPos[0]-viscous_replay*falcon_vel[0], 
        //         -stiffness*falconPos[1]-viscous_replay*falcon_vel[1], 
        //         -stiffness*(falconPos[2]-0.1)-viscous_replay*falcon_vel[2]});
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
       }

    catch(tf2::TransformException &ex){
        cout << "COULDN'T GET TF FROM PANDA" << endl << ex.what() << endl;
        }
    
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
