#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def get_transform():
    rospy.init_node('battery_to_camera_frame')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Publisher for the transformation topic
    transform_pub = rospy.Publisher('/battery_to_camera_transform', TransformStamped, queue_size=10)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            # Lookup transform from battery pack frame to camera frame
            trans = tfBuffer.lookup_transform('battery_pack_frame', 'camera_rgb_optical_frame', rospy.Time(0), rospy.Duration(1.0))

            # Publish the transform
            transform_msg = TransformStamped()
            transform_msg.header.stamp = rospy.Time.now()
            transform_msg.header.frame_id = 'battery_pack_frame'
            transform_msg.child_frame_id = 'camera_rgb_optical_frame'
            transform_msg.transform.translation = trans.transform.translation
            transform_msg.transform.rotation = trans.transform.rotation

            transform_pub.publish(transform_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform not available yet")
            continue
        rate.sleep()

if __name__ == '__main__':
    try:
        get_transform()
    except rospy.ROSInterruptException:
        pass
