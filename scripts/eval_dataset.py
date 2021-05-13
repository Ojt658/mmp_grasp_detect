#! /usr/bin/env python

"""
This ROS node is used to assess the ground truth/original dataset used to train the CNN.
It works alongside the depth detection, grasping, and grasp success nodes.
Running this shows that the oriinal data may not be the best as the grasps aren't always very successful.
"""

import rospy
import os
import pandas as pd
from gazebo_msgs.srv import SpawnModel, DeleteModel
from random import uniform, randint
import tf.transformations as transformations
from geometry_msgs.msg import Pose, Quaternion, Point, TransformStamped
from tf2_ros import StaticTransformBroadcaster
import numpy as np
from time import sleep, time
import re
from std_msgs.msg import Bool


class EvalData:
    def __init__(self):
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/delete_model")

        self.spawner = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.del_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.model_loaded_pub = rospy.Publisher("/model_loaded", Bool, queue_size=1)

        self.obj_loc = '/home/ollie/mmp_ws/src/mmp_grasp_detect/models/acronym'
        self.models = {m for m in os.listdir(self.obj_loc)}
        self.data = pd.read_csv('/home/ollie/Documents/Major project/scw1780/dataset.csv')
        # print(self.data['image'].value_counts())
        self.models = self.get_obj_grasp_list()
        self.model_list = self.models[0]
        self.model_grasp_dict = self.models[1]

    def get_obj_grasp_list(self):
        names = []
        grasps = self.data[['x', 'y', 'z', 'roll', 'pitch', 'yaw']]

        #Get object name from the dataset
        for obj in self.data['image'].values:
            obj = obj.split('.')[0].split('_')[0]
            split = re.split(r'(\d+)', obj)

            if split[0] == '': # First character is number
                obj = split[1].join(split[2])
                num = 3
            else:
                obj = split[0]
                num = 1

            if num < len(split):# Has end number
                obj = obj + '_' + str(split[num])

            names.append(obj)

        both = {}

        # Check for models in the dataset
        for index, n in enumerate(names):
            if n in self.models and n not in both:
                both[n] = grasps.iloc[index,:].values
            elif n in self.models:
                if len(both[n]) < 10:
                    both[n] = np.vstack((both[n], grasps.iloc[index,:].values))

        keys = both.keys()
        return keys, both

    def spawn(self, model):
        try:
            with open('{loc}/{n}/model.sdf'.format(loc=self.obj_loc, n=model), 'r') as file:
                text = file.read()

            r = uniform(0, 3.14)
            p = uniform(0, 3.14)
            y = uniform(0, 3.14)
            q = transformations.quaternion_from_euler(0, 0, 0)
            self.spawner(
                model_name=model, 
                model_xml=text, 
                robot_namespace="", 
                initial_pose=Pose(position=Point(0.7, 0, 0.39), orientation=Quaternion(q[0], q[1], q[2], q[3])),
                reference_frame="base_link"
            )
            # self.tfBroadcaster(0.7, 0, 0.39)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " + e.message)

    
    def delete(self, model):
        try:
            self.del_model(model)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " + e.message)

    def add_name_grasp(self, name):
        with open("/home/ollie/mmp_ws/src/mmp_grasp_detect/results/results.csv", 'a') as f:
            f.write(name)


    def setGrasp(self, grasp):
        br = StaticTransformBroadcaster()
        transformStamped = TransformStamped()

        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = "target_object"
        transformStamped.child_frame_id = "grasp"

        transformStamped.transform.translation.x = grasp[0]
        transformStamped.transform.translation.y = grasp[1]
        transformStamped.transform.translation.z = grasp[2] + 0.05

        q = transformations.quaternion_from_euler(grasp[3], grasp[4], grasp[5])
        transformStamped.transform.rotation.x = q[0]
        transformStamped.transform.rotation.y = q[1]
        transformStamped.transform.rotation.z = q[2]
        transformStamped.transform.rotation.w = q[3]

        br.sendTransform(transformStamped)

    def randomiseObjects(self, num):
        loaded = Bool()
        timeout = 15
        
        for m in range(num):
            t = False
            r_num = randint(0, len(self.models)-1) # Get random model to spawn

            model_name = self.model_list[r_num]

            ## Loop over each grasp in dictionary
            for grasp in self.model_grasp_dict[model_name]:
                self.add_name_grasp(model_name + ', ' + str(grasp) + ', ')  # Add name and grasp to the results file
                self.spawn(model_name)
                loaded.data = True
                start_time = time()
                rospy.set_param('/initial', 0)
                rospy.set_param("/loaded", 0) # Initialise each node.
                while rospy.get_param("/loaded") == 0 or rospy.get_param("/initial") == 0 and not rospy.is_shutdown():
                    self.model_loaded_pub.publish(loaded)
                    if time() > start_time + timeout:
                        break

                rospy.set_param("/grasp", 1) # Start grasping process
                self.setGrasp(grasp) ## Set grasp transform

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
                            self.add_name_grasp('0\n')
                            t = True

                self.delete(model_name) # Remove object from simulation


def main():
    rospy.init_node('eval_dataset')
    ed = EvalData()

    ed.randomiseObjects(rospy.get_param("/num_models"))
    rospy.loginfo("Done")
    rospy.loginfo("Grasp Average: {a}".format(a=str(rospy.get_param("/successful_grasps")/rospy.get_param("/num_models")*100)))


if __name__ == '__main__':
    main()
