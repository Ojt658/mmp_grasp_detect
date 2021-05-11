#! /usr/bin/env python

import os
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Quaternion, Point, TransformStamped
import tf.transformations as transformations
from random import randint, uniform
from time import sleep, time
from std_msgs.msg import Bool

from tf2_ros import StaticTransformBroadcaster


class LoadModel:
    def __init__(self, models):
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/delete_model")

        self.models = models
        self.model_name = ""
        self.spawner = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.del_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.model_loaded_pub = rospy.Publisher("/model_loaded", Bool, queue_size=1)

    def spawn(self, name):
        self.model_name = name
        self.add_name(name + ', ')
        try:
            with open('/home/ollie/mmp_ws/src/mmp_grasp_detect/models/acronym3/{n}/model.sdf'.format(n=name), 'r') as file:
                text = file.read()

            r = uniform(0, 3.14)
            p = uniform(0, 3.14)
            y = uniform(0, 3.14)
            q = transformations.quaternion_from_euler(0, 0, 0)
            self.spawner(
                model_name=name, 
                model_xml=text, 
                robot_namespace="", 
                initial_pose=Pose(position=Point(0.7, 0, 0.39), orientation=Quaternion(q[0], q[1], q[2], q[3])),
                reference_frame="base_link"
            )
            # self.tfBroadcaster(0.7, 0, 0.39) ## Replaced with depth detection node
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " + e.message)

    def delete(self):
        try:
            self.del_model(self.model_name)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " + e.message)

    def add_name(self, name):
        with open("/home/ollie/mmp_ws/src/mmp_grasp_detect/results/custom_results2.csv", 'a') as f:
            f.write(name)

    def randomise_grasp_objects(self, num):
        loaded = Bool()
        timeout = 15
        
        for m in range(num):
            t = False
            r_num = randint(0, len(self.models)-1) # Get random model to spawn
            self.spawn(self.models[r_num])
            loaded.data = True

            # rospy.set_param('/initial', 0)
            rospy.set_param("/loaded", 0) # Initialise each node.
            while rospy.get_param("/loaded") == 0 or rospy.get_param("/initial") == 0 and not rospy.is_shutdown():
                self.model_loaded_pub.publish(loaded)

            rospy.set_param("/grasp", 1) # Start grasping process
            # Attempt to grasp
            while rospy.get_param("/loaded") == 1 and not rospy.is_shutdown():
                # Arm grasping the loaded object
                # rospy.loginfo("Grasping random object")
                pass

            loaded.data = False
            self.model_loaded_pub.publish(loaded)
            start_time = time()
            while rospy.get_param("/loaded") == 2 and not rospy.is_shutdown():
                # Arm going to default location
                # rospy.loginfo("Waiting for success")
                if time() > start_time + timeout:
                    rospy.set_param("/grasp", 3)
                    if not t:
                        self.add_name('0\n')
                        t = True

            self.delete() # Remove object from simulation

    @staticmethod
    def tfBroadcaster(x, y, z):  ## Replaced with depth detection node
        br = StaticTransformBroadcaster()
        transformStamped = TransformStamped()

        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = "base_link"
        transformStamped.child_frame_id = "target_object"

        transformStamped.transform.translation.x = x
        transformStamped.transform.translation.y = y
        transformStamped.transform.translation.z = z

        q = transformations.quaternion_from_euler(0, 0, 0)
        transformStamped.transform.rotation.x = q[0]
        transformStamped.transform.rotation.y = q[1]
        transformStamped.transform.rotation.z = q[2]
        transformStamped.transform.rotation.w = q[3]

        br.sendTransform(transformStamped)


def main():
    rospy.init_node("spawner")
    models = [m for m in os.listdir("/home/ollie/mmp_ws/src/mmp_grasp_detect/models/acronym3/")]
    load = LoadModel(models)

    load.randomise_grasp_objects(rospy.get_param("/num_models"))
    rospy.loginfo("Done")
    rospy.loginfo("Grasp Average: {a}".format(a=str(rospy.get_param("/successful_grasps")/rospy.get_param("/num_models")*100)))


if __name__ == "__main__":
    main()