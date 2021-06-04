#!/usr/bin/env python
import rospy, sys

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np


class Writer:

    def __init__(self):
        # Input
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image", Image, self.callback)

        # File format
        self.format_string = "frame%04d.png"
        if rospy.has_param("~filename_format"):
            self.format_string = rospy.get_param("~filename_format")
            print("Using format: {}".format(self.format_string))
        self.count = 0

    def callback(self, depth_image):
        # Convert ros uint16 image to np float32 matrix
        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth_image, "mono16")
        except CvBridgeError as e:
            print(e)
            return

        # Save as png image
        filepath = self.format_string % self.count
        cv2.imwrite(filepath, cv_image)
        self.count = self.count +1

        print("Saved {}".format(filepath))


def main(args):
    rospy.init_node('writer', anonymous=True)
    l = Writer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)