#! /usr/bin/env python

import os
import rospy
import cv2
from opencv_apps.msg import MomentArrayStamped
from sensor_msgs.msg import JointState


class GraspSuccess:
    def __init__(self):
        self.total_grasps = rospy.get_param("/num_models")
        self.moment_sub = rospy.Subscriber('/moment/moments', MomentArrayStamped, self.momentCb, queue_size=1)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.jointCb, queue_size=1)
        self.gripper_closed = False
        self.initial = True
        self.initialX = 0.0
        self.initialY = 0.0

    def momentCb(self, msg):
        if rospy.get_param('/loaded') == 0:
            self.initial = True
        else:
            self.initial = False
        
        if len(msg.moments) > 0 and self.initial:
            rospy.loginfo("INITIALISE")
            moment = msg.moments[0]
            self.initialX = moment.center.x
            self.initialY = moment.center.y
            self.initial = False
            
        elif len(msg.moments) > 0 and not self.initial and rospy.get_param('/loaded') == 2 and not rospy.get_param('/grasp') == 3:
            rospy.loginfo("DIFFERENCE")
            moment = msg.moments[0]
            x = moment.center.x
            y = moment.center.y
            difference = self.checkDifference(x, y)
            if difference[1] > 0 and not self.gripper_closed:
                #Moved up :: Grasp successful
                rospy.set_param('/successful_grasps', float(rospy.get_param('/successful_grasps')) + 1)
                self.add_success(1)
            elif difference[1] > 0 and not self.gripper_closed:
                #Moved up :: Semi successful :: object thrown up
                rospy.set_param('/successful_grasps', float(rospy.get_param('/successful_grasps')) + 0.75)
                self.add_success(0.75)
            elif difference[0] > 100 or difference[0] < -100 or difference[1] < 100:
                #moved left or right :: Semi successful :: Grasp in right area or dropped soon after grasp
                rospy.set_param('/successful_grasps', float(rospy.get_param('/successful_grasps')) + 0.25)
                self.add_success(0.25)
            #Else add nothing due to failed grasp

            rospy.set_param('/grasp', 3)
        # elif len(msg.moments) == 0 and self.initial:
        #     # Object not spawned in view :: so skip
        #     rospy.set_param('/grasp', 3)


    def jointCb(self, msg):
        positions = msg.position ## Gripper joint values at index: 13, 14
        l_gripper = positions[13]
        r_gripper = positions[14]
        if l_gripper > 0.5 or r_gripper > 0.5:
            self.gripper_closed = False
        else:
            self.gripper_closed = True


    def checkDifference(self, x, y):
        return (self.initialX - x, self.initialY - y)

    
    def add_success(self, success):
        with open("/home/ollie/mmp_ws/src/mmp_grasp_detect/results/results.txt", 'a') as f:
            f.write(str(success) + '\n')

def main():
    rospy.init_node("grasp_success")
    rospy.set_param("/successful_grasps", 0)

    gs = GraspSuccess()

    # rospy.loginfo("Grasp success: {p}%".format(p=str(rospy.get_param("/successful_grasps")/rospy.get_param("/num_models"))))

    rospy.spin()


if __name__ == "__main__":
    main()