#! /usr/bin/env python

import rospy
import os
import pandas as pd
from gazebo_msgs.srv import SpawnModel, DeleteModel
from random import uniform
import tf.transformations as transformations
from geometry_msgs.msg import Pose, Quaternion, Point, TransformStamped
from tf2_ros import StaticTransformBroadcaster
import numpy as np
from time import sleep


class EvalData:
    def __init__(self):
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/delete_model")

        self.spawner = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.del_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

        self.obj_loc = '/home/ollie/mmp_ws/src/mmp_grasp_detect/models/acronym3'
        self.models = {m for m in os.listdir(self.obj_loc)}
        self.data = pd.read_csv('/home/ollie/Documents/Major project/scw1780/dataset.csv')

    def get_obj_grasp_list(self):
        names = []
        grasps = self.data[['x', 'y', 'z', 'roll', 'pitch', 'yaw']]

        for obj in self.data['image'].values:
            obj = obj.split('.')[0].split('_')[0]
            try:
                num = int(obj[-3:])
                obj = obj[:-3]
            except ValueError:
                try:
                    num = int(obj[-2:])
                    obj = obj[:-2]
                except ValueError:
                    try:
                        num = int(obj[-1:])
                        obj = obj[:-1]
                    except ValueError:
                        num = 0

            names.append(obj + '_' + str(num))

        # print(names[:5])
        both = {}
        for index, n in enumerate(names):
            if n in self.models and n not in both:
                both[n] = grasps.iloc[index,:].values
            elif n in self.models:
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
            self.tfBroadcaster(0.7, 0, 0.39)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " + e.message)

    
    def delete(self, model):
        try:
            self.del_model(model)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " + e.message)

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

        q = transformations.quaternion_from_euler(0, 0, 0)
        transformStamped.transform.rotation.x = q[0]
        transformStamped.transform.rotation.y = q[1]
        transformStamped.transform.rotation.z = q[2]
        transformStamped.transform.rotation.w = q[3]

        br.sendTransform(transformStamped)


    def grasp(self, grasp):
        sleep(1)


if __name__ == '__main__':
    rospy.init_node('eval_dataset')
    ed = EvalData()
    models = ed.get_obj_grasp_list()
    for index, model in enumerate(models[0]):
        for grasp in models[1][model]:
            ed.spawn(model)
            ed.grasp(models[1][model])
            ed.delete(model)

    rospy.spin()