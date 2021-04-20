#! /usr/bin/env python

import os
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Quaternion, Point, TransformStamped
import tf.transformations as transformations
from random import randint, uniform
from time import sleep, time

from tf2_ros import StaticTransformBroadcaster


class LoadModel:
    def __init__(self, models):
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/delete_model")

        self.models = models
        self.model_name = ""
        self.spawner = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.del_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    def spawn(self, name):
        self.model_name = name
        try:
            with open('/home/ollie/mmp_ws/src/mmp_grasp_detect/models/acronym3/{n}/model.sdf'.format(n=name), 'r') as file:
                text = file.read()

            r = uniform(0, 3.14)
            p = uniform(0, 3.14)
            y = uniform(0, 3.14)
            q = transformations.quaternion_from_euler(0, 0, y)
            self.spawner(
                model_name=name, 
                model_xml=text, 
                robot_namespace="", 
                initial_pose=Pose(position=Point(0.7, 0, 0.8), orientation=Quaternion(q[0], q[1], q[2], q[3])),
                reference_frame="base_link"
            )
            self.tfBroadcaster(0.7, 0, 0.39)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " + e.message)

    def delete(self):
        try:
            self.del_model(self.model_name)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " + e.message)


    def randomise_grasp_objects(self, num):
        timeout = 20
        for m in range(num):
            r_num = randint(0, len(self.models)) # Get random model to spawn
            self.spawn(self.models[r_num])

            rospy.set_param("/loaded", 1) # Attempt to grasp
            while rospy.get_param("/loaded") == 1 and not rospy.is_shutdown():
                # Arm grasping the loaded object
                rospy.loginfo("Grasping random object")

            start_time = time()
            while rospy.get_param("/loaded") == 2 and not rospy.is_shutdown():
                # Arm going to default location
                rospy.loginfo("Waiting for success")
                if time() > start_time + timeout:
                    rospy.set_param("/grasp", 0)
                    rospy.set_param("/loaded", 0)

            self.delete() # Remove object from simulation

    @staticmethod
    def tfBroadcaster(x, y, z):
        br = StaticTransformBroadcaster()
        transformStamped = TransformStamped()

        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = "base_link"
        transformStamped.child_frame_id = "target_object"

        transformStamped.transform.translation.x = x
        transformStamped.transform.translation.y = y
        transformStamped.transform.translation.z = z

        transformStamped.transform.rotation.x = 0
        transformStamped.transform.rotation.y = 0
        transformStamped.transform.rotation.z = 0
        transformStamped.transform.rotation.w = 0

        br.sendTransform(transformStamped)


def main():
    rospy.init_node("spawner")
    models = [m for m in os.listdir("/home/ollie/mmp_ws/src/mmp_grasp_detect/models/acronym3/")]
    load = LoadModel(models)

    load.randomise_grasp_objects(rospy.get_param("/num_models"))
    rospy.loginfo("Done")


if __name__ == "__main__":
    main()