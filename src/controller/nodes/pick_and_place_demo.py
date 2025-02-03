#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from panda_ros_msgs.msg import VelocityBoundPath

def publish_path(path_pub, poses, max_v=0.1, repeat=5):
    vel_bound_path = VelocityBoundPath()
    vel_bound_path.maxV = max_v
    vel_bound_path.poses = poses

    for i in range(repeat):
        path_pub.publish(vel_bound_path)
        rospy.sleep(0.1)

def publish_gripper(cmd_pub, command, repeat=4):
    for i in range(repeat):
        cmd_pub.publish(command)
        rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('pick_and_place_demo', anonymous=True)

    path_pub = rospy.Publisher('/panda/velocity_bound_path', VelocityBoundPath, queue_size=10)
    gripper_pub = rospy.Publisher('/panda/commands', String, queue_size=10)

    pose1 = Pose()
    pose1.position.x = 0.7
    pose1.position.y = 0.0
    pose1.position.z = 0.0

    pose2 = Pose()
    pose2.position.x = 0.7
    pose2.position.y = 0.0
    pose2.position.z = -0.2

    pose3 = Pose()
    pose3.position.x = 0.7
    pose3.position.y = -0.2
    pose3.position.z = 0.0

    pose4 = Pose()
    pose4.position.x = 0.7
    pose4.position.y = -0.2
    pose4.position.z = -0.2

    # 1. First move to pose1
    rospy.loginfo("Step 1: Move to pose1...")
    publish_path(path_pub, [pose1])
    rospy.sleep(5)

    # 2. Then move down to pick position
    rospy.loginfo("Step 2: Move down to pose2...")
    publish_path(path_pub, [pose2])
    rospy.sleep(3)

    # 3. Close the gripper
    rospy.loginfo("Step 3: Pick object...")
    publish_gripper(gripper_pub, "grasp")
    rospy.sleep(3)

    # 4. Move to the place position
    rospy.loginfo("Step 4: Move to pose4...")
    publish_path(path_pub, [pose1, pose3, pose4])
    rospy.sleep(7)

    # 5. Open the gripper
    rospy.loginfo("Step 5: Release...")
    publish_gripper(gripper_pub, "release")
    rospy.sleep(2)

    rospy.loginfo("Pick and place sequence completed.")
    