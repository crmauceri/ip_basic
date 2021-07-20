#!/usr/bin/env python
import rospy, sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2, os, struct

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
            cv_image = self.compressedDepthDecode(img_msg)
        else:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono16')
            except CvBridgeError as e:
                print(e)
                return

        # Save as png image
        cv2.imwrite(filepath, cv_image)

        rospy.logdebug("Saved {}".format(filepath))

    def compressedDepthDecode(self, img_msg):
        # Code from https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
        # 'msg' as type CompressedImage
        depth_fmt, compr_type = img_msg.format.split(';')
        # remove white space
        depth_fmt = depth_fmt.strip()
        compr_type = compr_type.strip()
        if compr_type != "compressedDepth":
            raise Exception("Compression type is not 'compressedDepth'."
                            "You probably subscribed to the wrong topic.")

        # remove header from raw data
        depth_header_size = 12
        raw_data = img_msg.data[depth_header_size:]

        depth_img_raw = cv2.imdecode(np.fromstring(raw_data, np.uint8), cv2.IMREAD_UNCHANGED)
        if depth_img_raw is None:
            # probably wrong header size
            raise Exception("Could not decode compressed depth image."
                            "You may need to change 'depth_header_size'!")

        if depth_fmt == "16UC1":
            # write raw image data
            return depth_img_raw
        elif depth_fmt == "32FC1":
            raw_header = img_msg.data[:depth_header_size]
            # header: int, float, float
            [compfmt, depthQuantA, depthQuantB] = struct.unpack('iff', raw_header)
            depth_img_scaled = depthQuantA / (depth_img_raw.astype(np.float32) - depthQuantB)
            # filter max values
            depth_img_scaled[depth_img_raw == 0] = 0

            # depth_img_scaled provides distance in meters as f32
            # for storing it as png, we need to convert it to 16UC1 again (depth in mm)
            depth_img_mm = (depth_img_scaled * 1000).astype(np.uint16)
            return depth_img_mm
        else:
            raise Exception("Decoding of '" + depth_fmt + "' is not implemented!")

def main(args):
    rospy.init_node('writer', anonymous=True)
    node = PngWriter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main(sys.argv)