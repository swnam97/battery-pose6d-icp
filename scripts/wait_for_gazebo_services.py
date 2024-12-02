#!/usr/bin/env python3
import rospy

rospy.init_node("wait_for_gazebo_services")
rospy.loginfo("Waiting for Gazebo spawn service...")
rospy.wait_for_service("/gazebo/spawn_urdf_model")
rospy.loginfo("Gazebo spawn service is ready.")
