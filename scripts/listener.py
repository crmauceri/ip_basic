#!/usr/bin/env python
import rospy, sys

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
from ip_basic import depth_map_utils


class Listener:

    def __init__(self, fill_type='fast', extrapolate=False, blur_type='gaussian'):
        # Input
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/depth_image", Image, self.callback)

        # Output
        self.image_pub = rospy.Publisher("/completed_depth", Image, queue_size=10)

        # Class variables
        self.fill_type = fill_type
        self.extrapolate = extrapolate
        self.blur_type = blur_type

    def callback(self, depth_image):
        # Convert ros uint16 image to np float32 matrix
        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth_image, "mono16")
        except CvBridgeError as e:
            print(e)
            return
        projected_depths = np.float32(cv_image / 256.0)

        # Fill in
        if self.fill_type == 'fast':
            final_depths = depth_map_utils.fill_in_fast(
                projected_depths, extrapolate=self.extrapolate, blur_type=self.blur_type)
        elif self.fill_type == 'multiscale':
            final_depths, process_dict = depth_map_utils.fill_in_multiscale(
                projected_depths, extrapolate=self.extrapolate, blur_type=self.blur_type,
                show_process=False)
        else:
            raise ValueError('Invalid fill_type {}'.format(self.fill_type))

        completed_image = (final_depths * 256).astype(np.uint16)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(completed_image, "mono16"))
        except CvBridgeError as e:
            print(e)


def main(args):
    rospy.init_node('listener', anonymous=True)
    l = Listener()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)