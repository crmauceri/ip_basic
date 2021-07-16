#!/usr/bin/env python
import rospy, sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2, os

from sensor_msgs.msg import CompressedImage

import numpy as np


class PngWriter:

    def __init__(self):
        self.save_root = ""
        if rospy.has_param("save_root"):
            self.save_root = rospy.get_param("save_root")
            rospy.loginfo("Save to directory: {}".format(self.save_root))

        # File format
        self.format_string = "frame%s.png"
        if rospy.has_param("~filename_format"):
            self.format_string = rospy.get_param("~filename_format")
            rospy.loginfo("Using format: {}".format(self.format_string))
        self.compressed = False
        if rospy.has_param("~compressed"):
            self.compressed = rospy.get_param("~compressed")
            rospy.loginfo("Using compressed image messages: {}".format(self.compressed))

        if self.compressed:
            self.image_sub = rospy.Subscriber("/image", CompressedImage, self.callback)
        else:
            # Input
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber("/image", Image, self.callback)
        rospy.loginfo("Writer initialized")

    def callback(self, img_msg):
        stamp = img_msg.header.stamp
        time_str = '%d.%06d' % (stamp.secs, stamp.nsecs)
        filepath = os.path.join(self.save_root, (self.format_string % time_str))

        # Convert ros image to cv matrix using same encoding
        if self.compressed:
            # remove header from raw data
            depth_header_size = 12
            raw_data = img_msg.data[depth_header_size:]
            np_arr = np.fromstring(raw_data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_ANYDEPTH)
            if cv_image is None:
                rospy.loginfo("Failed to load {}".format(filepath))
                return
        else:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono16')
            except CvBridgeError as e:
                print(e)
                return

        # Save as png image
        cv2.imwrite(filepath, cv_image)

        rospy.logdebug("Saved {}".format(filepath))


def main(args):
    rospy.init_node('writer', anonymous=True)
    node = PngWriter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main(sys.argv)