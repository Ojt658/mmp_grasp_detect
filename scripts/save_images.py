#! /usr/bin/env python

"""
Spike work for saving images from the Gazebo scene.
"""

import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class SaveImage:
    def __init__(self):
        self.found = False
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageCb, queue_size=1)

    def imageCb(self, msg):
        # print("test")
        try:
            bridge = CvBridge()
            image = bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            print(e)

        # (rows,cols,channels) = cv_depth.shape
        if not self.found:
            found = True
            cv2.imshow("Test", image)
            cv2.waitKey(3)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            cv2.imwrite("Test.png", image)

def main():
    print("main")
    rospy.init_node("save_images")
    si = SaveImage()
    rospy.spin()


if __name__ == "__main__":
    main()