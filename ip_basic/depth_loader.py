import cv2
import numpy as np
from PIL import Image
import numpy as np

# Conversion from https://github.com/mcordts/cityscapesScripts see `disparity`
# See https://github.com/mcordts/cityscapesScripts/issues/55#issuecomment-411486510
def cityscapes_disparity_to_depth(disparity_path):
    disparity_arr = cv2.imread(disparity_path, cv2.IMREAD_ANYDEPTH)
    disparity_arr[disparity_arr > 0] = (disparity_arr[disparity_arr > 0] - 1.0) / 256.
    depth_arr = np.zeros(disparity_arr.shape)
    depth_arr[disparity_arr > 0] = 0.2 * 2262 / disparity_arr[disparity_arr > 0]
    depth_arr = depth_arr.astype(np.uint8)

    return depth_arr

# From kitti depth completion devkit
def kitti_depth_read(filename):
    # loads depth map D from png file
    # and returns it as a numpy array,
    # for details see readme.txt

    depth_png = np.array(Image.open(filename), dtype=int)
    # make sure we have a proper 16bit depth map here.. not 8bit!
    assert(np.max(depth_png) > 255)

    depth = depth_png.astype(np.float) / 256.
    depth[depth_png == 0] = 0
    return depth