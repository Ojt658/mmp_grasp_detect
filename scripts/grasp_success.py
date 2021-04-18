#! /usr/bin/env python

import os
import rospy
import cv2
from opencv_apps.msg import MomentArrayStamped


class GraspSuccess:
    def __init__(self):
        self.total_grasps = rospy.get_param("/num_models")
        self.bridge = CvBridge()
        self.moment_sub = rospy.Subscriber('/moment/moments', MomentArrayStamped, self.momentCb, queue_size=1)
        self.initial = True
        self.initialX = 0.0
        self.initialY = 0.0

    def momentCb(self, msg):
        if rospy.get_param('/loaded') == 1:
            self.initial = True
        else:
            self.initial = False
        
        if len(msg.moments) > 0 and self.initial:
            moment = msg.moments[0]
            self.initialX = moment.center.x
            self.initialY = moment.center.y
            self.initial = False
        else if len(msg.moments) > 0 and not self.initial and rospy.get_param('/grasp') == 3:
            moment = msg.moments[0]
            x = moment.center.x
            y = moment.center.y
            difference = self.checkDifference(x, y)
            if difference[1] < 0:
                #Moved up :: Grasp successful
                rospy.set_param('/successful_grasps', float(rospy.get_param('/succussful_grasps')) + 1)
            else if difference[0] > 10 or difference < -10:
                #moved left or right :: Semi successful
                rospy.set_param('/successful_grasps', float(rospy.get_param('/succussful_grasps')) + 0.5)
            else:
                #Grasp failed

            rospy.set_param('/loaded', 0)
            self.initial = True


    def checkDifference(x, y):
        return (self.initialX - x, self.initialY - y)

def main():
    rospy.init_node("grasp_success")
    rospy.set_param("/successful_grasps", 0)

    gs = GraspSuccess()

    rospy.loginfo("Grasp success: {p}%".format(p=str(rospy.get_param("/successful_grasps")/rospy.get_param("/num_models"))))

    rospy.spin()


if __name__ == "__main__":
    main()