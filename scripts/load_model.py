#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Quaternion, Point

rospy.init_node("spawner")

rospy.wait_for_service("/gazebo/spawn_urdf_model")

with open('/home/ollie/mmp_ws/src/mmp_grasp_detect/models/acronym/2Shelves/model.sdf','r') as file:
    text = file.read()

try:
    spawner = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    spawner(
        model_name="test", 
        model_xml=text, 
        robot_namespace="", 
        initial_pose=Pose(position=Point(0.711,0,0.017), orientation=Quaternion(0,0,0,0)),
        reference_frame="world"
    )
except rospy.ServiceException as e:
    rospy.loginfo("Service call failed: " + e.message)

rospy.spin()
