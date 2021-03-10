#! /usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class SaveImage:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/depth/raw_image", Image, self.imageCb, queue_size=1)
        self.found = True

    def imageCb(self, msg):
        print("test")
        try:
            cv_depth = CvBridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_depth.shape
        if found:
            found = False
            cv2.imshow("Test", cv_depth)
            cv2.waitKey(3)

def main():
    print("main")
    rospy.init_node("save_images")
    si = SaveImage()
    rospy.spin()


if __name__ == "__main__":
    main()