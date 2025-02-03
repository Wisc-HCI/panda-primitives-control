#!/usr/bin/env python

"""
This script is used to test many of the control options in panda_ros.
To run this, see README.md section 4 "Controller test". To test each desired
functionality, uncomment it in main.

Notes (please read BEFORE USING):
* Publishable:
    * /panda/cart_pose in test_cart_pos(): Bot goes to caresian pose (x, y, z). This is too jerky for long-distance movements. 
    * /panda/velocity_bound_path in test_vel_bound_path: Bot goes to array of cartesian pose with specified max velocity 
      (unit uknown but 0.1-0.5 is good speed range).
    * /panda/cart_velocity in test_vel: Bot goes to cartesian velocity (unit uknown but 0.1-0.5 is good speed range).
    * /panda/joint_pose in test_joint_pose: Bot goes to joint positions (in radians). Must specify header.stamp to be 
      the end time that the bot should reach position, otherwise will go crazy fast and reach velocity limits.
    * /panda/commands in  test_gripper(): Gripper "grasp", "release", etc based on string published. 
* Subscribable:
    * /panda/wrench in test_wrench_data(): Gets wrench data (force and torque in x, y, z direction) from force/torque
      sensor on end-effector. All will be 0 if sensor not plugged in.
    * /panda/hybrid_pose in test_hybrid_pose(): Moves to position while keeping given force/torque. Very jerky for movements
      larger than 1cm. May be better for succesive small movements. Requires force/torque sensor.
"""

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, TwistStamped, Wrench
from panda_ros_msgs.msg import VelocityBoundPath, JointPose, HybridPose


def test_cart_pos():
    pub = rospy.Publisher('/panda/cart_pose', Pose, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    # For some reason, continuosly sending messages makes it jerky while
    # only sending one does not go through. 5 at 10hz seems optimal
    for i in range(5):
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.1
        pose.position.z = 0.2
        
        rospy.loginfo(pose)
        pub.publish(pose)
        rate.sleep()
        

def test_vel_bound_path():

    pub = rospy.Publisher('/panda/velocity_bound_path', VelocityBoundPath, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    # For some reason, continuosly sending messages makes it jerky while
    # only sending one does not go through. 5 at 10hz seems optimal
    for i in range(5):
        pose1 = Pose()
        pose1.position.x = 0.3
        pose1.position.y = 0.1
        pose1.position.z = 0.2
        
        pose2 = Pose()
        pose2.position.x = 0.4
        pose2.position.y = 0.2
        pose2.position.z = 0.3

        vel_bound_path = VelocityBoundPath()
        vel_bound_path.poses = [pose1, pose2]
        vel_bound_path.maxV = 0.5 # NOT SURE THE UNITS but 0.1 is slow and 0.5 is fast.

        rospy.loginfo(vel_bound_path)
        pub.publish(vel_bound_path)
        rate.sleep()


def test_vel():
    pub = rospy.Publisher('/panda/cart_velocity', TwistStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    # For some reason, a single message does not go through so need to
    # send at least 5. Here send for 1 second and then stop.
    for i in range(10):
        twist = TwistStamped()
        twist.twist.linear.x=0.1
        twist.twist.linear.y=0.1
        twist.twist.linear.z=0.1

        twist.twist.angular.x=0.1
        twist.twist.angular.y=0.1
        twist.twist.angular.z=0.1

        twist.header = Header(stamp=rospy.Time.now())
        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()

    # Again, need to send at least 5 messages to stop
    for i in range(5):
        twist = TwistStamped()
        twist.twist.linear.x=0
        twist.twist.linear.y=0
        twist.twist.linear.z=0

        twist.twist.angular.x=0
        twist.twist.angular.y=0
        twist.twist.angular.z=0

        twist.header = Header(stamp=rospy.Time.now())
        rospy.loginfo(twist)
        pub.publish(twist)


def test_joint_pose():
    pub = rospy.Publisher('/panda/joint_pose', JointPose, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    # For some reason, a single message does not go through so need to
    # send at least 5.
    for i in range(5):
        joint_pose = JointPose()
        joint_pose.joint_pose = [0.0, -0.8 ,0.0, -1.3, 0.0, 3.5, 0.8]  # Almost vertical
        # joint_pose.joint_pose = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]  # Default position
        
        # Add desired end time (3 seconds from now) so doesn't have insane speed               
        joint_pose.header = Header(stamp=rospy.Time.now() + rospy.Duration(3))

        rospy.loginfo(joint_pose)
        pub.publish(joint_pose)
        rate.sleep()


def test_gripper():
    pub = rospy.Publisher('/panda/commands', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    # For some reason, a single message does not go through so need to
    # send at least 4.
    for i in range(4):
        string = String()
        string.data = "grasp"

        rospy.loginfo(string)
        pub.publish(string)
        rate.sleep()

    wait_rate = rospy.Rate(0.3)
    wait_rate.sleep() # Wait 3 second

    for i in range(4):
        string = String()
        string.data = "release"

        rospy.loginfo(string)
        pub.publish(string)
        rate.sleep()

def test_hybrid_pose():
    pub = rospy.Publisher('/panda/hybrid_pose', HybridPose, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    # For some reason, a single message does not go through so need to
    # send at least 4.
    for i in range(4):

        hybrid_pose = HybridPose()
        # Not exactly sure what this is for (perhaps restricting movement to x, y, z, and not rotation) 
        # but required for this controller to work
        hybrid_pose.sel_vector=[1,1,1,0,0,0] 
        pose = Pose()
        pose.position.x = 0.35
        pose.position.y = 0.1
        pose.position.z = 0.25
        wrench = Wrench()
        wrench.force.x = 0
        wrench.force.y = 1
        wrench.force.z = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0

        hybrid_pose.pose = pose
        hybrid_pose.wrench = wrench
        # Also includes selection vector sel_vector (which I'm not sure what that is) 
        # and constraint_frame which is optional I think 

        rospy.loginfo(hybrid_pose)
        pub.publish(hybrid_pose)
        rate.sleep()


def test_wrench_data():
    """ Prints wrench data (torque + force of EE)"""

    counter = 1
    def callback(data):
        # Only print data every 0.5 seconds so doesn't get behind
        nonlocal counter
        if counter % 500 == 0:
            rospy.loginfo("Wrench Data:\n %s", data)
            counter = 1
        counter += 1
    
    rospy.Subscriber("/panda/wrench", Wrench, callback)


if __name__ == '__main__':
    rospy.init_node('controller_tester', anonymous=True)
    
    ##### UNCOMMENT THE FOLLOWING INDIVIDUALLY TO TEST #####
    # test_cart_pos()
    # test_vel_bound_path()
    # test_vel()
    # test_joint_pose()
    test_gripper()
    # test_hybrid_pose() 

    ### Data
    # test_wrench_data()

    rospy.spin()
