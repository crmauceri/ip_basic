#!/usr/bin/env python
import rospy, sys

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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
            rospy.loginfo(e)
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
            img_msg = self.bridge.cv2_to_imgmsg(completed_image, "mono16")
            img_msg.header = depth_image.header
            self.image_pub.publish(img_msg)
        except CvBridgeError as e:
            rospy.loginfo(e)


def main(args):
    rospy.init_node('listener', anonymous=True)

    if rospy.has_param("~fill_type"):
        fill_type = rospy.get_param("~fill_type")
    else:
        fill_type = 'fast'
    if rospy.has_param("~extrapolate"):
        extrapolate = rospy.get_param("~extrapolate")
    else:
        extrapolate = False
    if rospy.has_param("~blur_type"):
        blur_type = rospy.get_param("~blur_type")
    else:
        blur_type = 'gaussian'
    rospy.loginfo('Starting IP-Basic Node with fill type={}, extrapolate={}, blur_type={}'.format(fill_type, extrapolate, blur_type))

    l = Listener(fill_type, extrapolate, blur_type)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main(sys.argv)