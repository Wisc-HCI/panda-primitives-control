#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <signal.h>
#include "PandaController.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include <relaxed_ik/JointAngles.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <csignal>
#include <deque>

using namespace std;

void signalHandler(int sig)
{
    std::cout << "Interrupt " << sig << " recieved in panda_ros.cpp\n";
    PandaController::stopControl();
    ros::NodeHandle n("~");
    ros::Publisher wrenchPub = n.advertise<geometry_msgs::Wrench>("/panda/wrench", 10);
    geometry_msgs::Wrench wrench;
    wrench.force.x = 0;
    wrench.force.y = 0;
    wrench.force.z = 0;
    wrench.torque.x = 0;
    wrench.torque.y = 0;
    wrench.torque.z = 0;

    wrenchPub.publish(wrench);   
    ros::shutdown();

    std::array<double, 6> data = {0.0,0.0,0.0,0.0,0.0,0.0};
    PandaController::writeCommandedVelocity(data);
    exit(sig);
}

void updateCallbackCartPos(const geometry_msgs::Pose::ConstPtr& msg){
    if (PandaController::isRunning()){
        std::array<double, 6> position;
        position[0] = msg->position.x;
        position[1] = msg->position.y;
        position[2] = msg->position.z;
        //TODO update with quat
        position[3] = 0;
        position[4] = 0;
        position[5] = 0;
        
        PandaController::writeCommandedPosition(position);
    }
}

void updateCallbackPath(const nav_msgs::Path::ConstPtr& msg) {
    if (PandaController::isRunning()){
        std::array<double, 7> commandedPath[msg->poses.size()]; 
        for (size_t i = 0; i < msg->poses.size(); i++) {
            auto poseStamped = msg->poses[i];
            long secs = poseStamped.header.stamp.sec;
            long nsecs = poseStamped.header.stamp.nsec;

            std::array<double, 7> command;
            command[0] = secs * 1000 + nsecs / 1000000;
            
            command[1] = poseStamped.pose.position.x;
            command[2] = poseStamped.pose.position.y;
            command[3] = poseStamped.pose.position.z;
            PandaController::EulerAngles angles = 
                PandaController::quaternionToEuler(Eigen::Quaternion<double>(
                    poseStamped.pose.orientation.w,
                    poseStamped.pose.orientation.x,
                    poseStamped.pose.orientation.y,
                    poseStamped.pose.orientation.z
                ));
            command[4] = angles.roll;
            command[5] = angles.pitch;
            command[6] = angles.yaw;
            commandedPath[i] = command;
        }
        
        PandaController::writeCommandedPath(commandedPath, msg->poses.size());
    }
}

void updateCallbackControlCamera(const std_msgs::Bool::ConstPtr & msg) {
    PandaController::setControlCamera(msg->data);
}

void updateCallbackJointPos(const relaxed_ik::JointAngles::ConstPtr& msg){
    if (PandaController::isRunning()){
        std::array<double, 7> position;
        position[0] = msg->angles.data[0];
        position[1] = msg->angles.data[1];
        position[2] = msg->angles.data[2];
        position[3] = msg->angles.data[3];
        position[4] = msg->angles.data[4];
        position[5] = msg->angles.data[5];
        position[6] = msg->angles.data[6];
        PandaController::writeJointAngles(position);
    }
}

void updateCallbackJointVel(const relaxed_ik::JointAngles::ConstPtr& msg){
    //TODO
    return;
}

void updateCallbackCartVel(const geometry_msgs::Twist::ConstPtr& msg){
    if (PandaController::isRunning()){
        std::array<double, 6> velocity;
        velocity[0] = msg->linear.x;
        velocity[1] = msg->linear.y;
        velocity[2] = msg->linear.z;
        velocity[3] = msg->angular.x;
        velocity[4] = msg->angular.y;
        velocity[5] = msg->angular.z;
        PandaController::writeCommandedVelocity(velocity);
    }
}

void callbackCommands(const std_msgs::String& msg){

    if(msg.data == "grasp"){
        cout<<"Grasping"<<endl;
        PandaController::graspObject();
    }
    if(msg.data == "release"){
        PandaController::releaseObject();
    }
    if(msg.data == "toggleGrip") {
        PandaController::toggleGrip();
    }
    
}

void publishJointState(franka::RobotState robot_state, ros::Publisher jointPub){
    const vector<string> joint_names{"panda_joint1", "panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7","panda_finger_joint1","panda_finger_joint2"};
    franka::GripperState gripperState = PandaController::readGripperState();

    sensor_msgs::JointState states;
    states.effort.resize(joint_names.size());
    states.name.resize(joint_names.size());
    states.position.resize(joint_names.size());
    states.velocity.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); i++) {
        states.name[i] = joint_names[i];
    }
    states.header.stamp = ros::Time::now();
    for (size_t i = 0; i < joint_names.size()-2; i++) {
        states.position[i] = robot_state.q[i];
        states.velocity[i] = robot_state.dq[i];
        states.effort[i] = robot_state.tau_J[i];
    }
    states.position[joint_names.size()-2] = gripperState.width/2.;
    states.position[joint_names.size()-1] = gripperState.width/2.;
    
    jointPub.publish(states);
}

void publishTf(franka::RobotState robot_state){
    static tf2_ros::TransformBroadcaster br;
    
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());
    // Align the orientation of the end-effector with panda_link0
    Eigen::Quaterniond rot(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()));
    orientation *= rot;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "panda_link0";
    transformStamped.child_frame_id = "end_effector";
    transformStamped.transform.translation.x = position[0];
    transformStamped.transform.translation.y = position[1];
    transformStamped.transform.translation.z = position[2];
    
    transformStamped.transform.rotation.x = orientation.coeffs()[0];
    transformStamped.transform.rotation.y = orientation.coeffs()[1];
    transformStamped.transform.rotation.z = orientation.coeffs()[2];
    transformStamped.transform.rotation.w = orientation.coeffs()[3];

    br.sendTransform(transformStamped);
}

void publishWrench(franka::RobotState robot_state, ros::Publisher wrenchPub){
    std::array<double, 6> forces;
    //forces = robot_state.O_F_ext_hat_K;
    forces = PandaController::readFTForces();

    geometry_msgs::Wrench wrench;
    wrench.force.x = forces[0];
    wrench.force.y = forces[1];
    wrench.force.z = forces[2];
    wrench.torque.x = forces[3];
    wrench.torque.y = forces[4];
    wrench.torque.z = forces[5];

    wrenchPub.publish(wrench);
}

void publishState(ros::Publisher wrenchPub, ros::Publisher jointPub){
    franka::RobotState robot_state = PandaController::readRobotState();
    publishJointState(robot_state, jointPub);
    publishTf(robot_state);
    publishWrench(robot_state, wrenchPub);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "PandaListener");
    ros::NodeHandle n("~");
    std::string mode_str;
    //Always specify parameter, it uses cached parameter instead of default value.
    n.param<std::string>("control_mode", mode_str, "none");
    PandaController::ControlMode mode;

    if(mode_str == "cartesian_velocity")
        mode = PandaController::ControlMode::CartesianVelocity;
    if(mode_str == "joint_velocity")
        mode = PandaController::ControlMode::JointVelocity;
    if(mode_str == "cartesian_position")
        mode = PandaController::ControlMode::CartesianPosition;
    if(mode_str == "joint_position")
        mode = PandaController::ControlMode::JointPosition;
    if(mode_str == "none")
        mode = PandaController::ControlMode::None;
        
    //Setup the signal handler for exiting, must be called after ros is intialized
    signal(SIGINT, signalHandler); 

    PandaController::initPandaController(mode);
    
    ros::Subscriber sub_commands = n.subscribe("/panda/commands", 10, callbackCommands);
    ros::Subscriber sub_position;
    ros::Subscriber sub_trajectory;
    ros::Subscriber sub_controlCamera;
    switch(mode){
        case PandaController::ControlMode::CartesianVelocity:
            sub_position = n.subscribe("/panda/cart_vel", 10, updateCallbackCartVel);
            break;
        case PandaController::ControlMode::JointVelocity:
            //TODO: we don't actually have anything that uses this, not set up correctly in PandaController
            sub_position = n.subscribe("/relaxed_ik/joint_angle_solutions", 10, updateCallbackJointVel);
            break;
        case PandaController::ControlMode::CartesianPosition:
            sub_position = n.subscribe("/panda/cart_pose", 10, updateCallbackCartPos);
            sub_trajectory = n.subscribe("/panda/path", 10, updateCallbackPath);
            sub_controlCamera = n.subscribe("/panda/controlCamera", 10, updateCallbackControlCamera);
            
            break;
        case PandaController::ControlMode::JointPosition:
            sub_position = n.subscribe("/panda/joint_angles", 10, updateCallbackJointPos);
            break;
        case PandaController::ControlMode::None:
            break;     
    }
    ros::Publisher wrenchPub = n.advertise<geometry_msgs::Wrench>("/panda/wrench", 10);
    ros::Publisher jointPub = n.advertise<sensor_msgs::JointState>("/panda/joint_states", 1);
    ros::Rate loopRate(1000);
    while (ros::ok() && PandaController::isRunning()) {
        publishState(wrenchPub,jointPub);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
