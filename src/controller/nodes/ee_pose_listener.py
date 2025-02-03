#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('ee_pose_listener', anonymous=True)

    # Create TF Buffer and TF Listener
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        try:
            # Run: rostopic echo /tf, we see three related messages: panda_camera, panda_gripper, panda_ee
            # And panda_ee is what we want, wo we query the transformation from world to ee.
            transform = tfBuffer.lookup_transform(
                'panda_link0',  # world frame
                'panda_ee',  # ee link
                rospy.Time(0)  
            )

            # print ee position and rotation
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            rospy.loginfo("EE Position: x=%.6f, y=%.6f, z=%.6f;  "
                          "Orientation (quat): x=%.6f, y=%.6f, z=%.6f, w=%.6f" %
                          (translation.x, translation.y, translation.z,
                           rotation.x, rotation.y, rotation.z, rotation.w))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup exception: %s", e)

        rate.sleep()
