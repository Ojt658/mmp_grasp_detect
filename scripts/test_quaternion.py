#! /usr/bin/env python

import h5py
import os
import numpy as np
import rospy
import tf.transformations as transformations
import tf2_ros
import geometry_msgs.msg


class GraspTransformBroadcaster(object):
    _br = tf2_ros.TransformBroadcaster()
    _msg = geometry_msgs.msg.TransformStamped()

    def __init__(self):
        grasp_location = "/home/ollie/Documents/Major project/acronym data/grasps/"
        os.chdir(grasp_location)

        files = [f for r, d, f in os.walk(os.getcwd())]

        grasp_file = h5py.File(grasp_location + files[0][0])
        grasp_tf = grasp_file['grasps/transforms'][()]
        # print(grasp_tf[0])

        arr = np.array(grasp_tf[0])

        # Decode 4x4 transformation matrix
        scale, shear, rpy_angles, translation_vector, perspective = transformations.decompose_matrix(arr)
        # print(translation_vector)
        q = transformations.quaternion_from_euler(rpy_angles[0], rpy_angles[1], rpy_angles[2])

        self._msg.header.stamp = rospy.Time.now()
        self._msg.header.frame_id = "panda_link0"
        self._msg.child_frame_id = "target"
        self._msg.transform.translation.x = translation_vector[0]
        self._msg.transform.translation.y = translation_vector[1]
        self._msg.transform.translation.z = translation_vector[2]

        self._msg.transform.rotation.x = q[0]
        self._msg.transform.rotation.y = q[1]
        self._msg.transform.rotation.z = q[2]
        self._msg.transform.rotation.w = q[3]


    def broadcast(self):
        self._br.sendTransform(self._msg)


rospy.init_node('test_q')
tb = GraspTransformBroadcaster()
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    tb.broadcast()
    rospy.loginfo("BROADCASTING")
    # rate.sleep()
