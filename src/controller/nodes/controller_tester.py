#!/usr/bin/env python

"""
This script is used to test many of the control options in panda_ros.
To run this, see README.md section 4 "Controller test". To test the desired
functionality, uncomment it in main.

Notes:
* /panda/cart_pose: Bot goes to caresian pose (x, y, z). This is too jerky for long-distance movements. 
* /panda/velocity_bound_path: Bot goes to array of cartesian pose with specified max velocity 
  (unit uknown but 0.1-0.5 is good speed range).
* /panda/cart_velocity: Bot goes to cartesian velocity (unit uknown but 0.1-0.5 is good speed range).
"""

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, TwistStamped
from panda_ros_msgs.msg import VelocityBoundPath


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




if __name__ == '__main__':
    rospy.init_node('controller_tester', anonymous=True)
    
    ##### UNCOMMENT THE FOLLOWING TO TEST #####
    #test_cart_pos()
    #test_vel_bound_path()
    test_vel()

