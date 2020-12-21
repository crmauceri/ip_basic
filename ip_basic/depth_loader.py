import cv2
import numpy as np

def cityscapes_disparity_to_depth(disparity_path):
    disparity_arr = cv2.imread(disparity_path, cv2.IMREAD_ANYDEPTH)
    # Conversion from https://github.com/mcordts/cityscapesScripts see `disparity`
    # See https://github.com/mcordts/cityscapesScripts/issues/55#issuecomment-411486510
    disparity_arr[disparity_arr > 0] = (disparity_arr[disparity_arr > 0] - 1.0) / 256.
    depth_arr = np.zeros(disparity_arr.shape)
    depth_arr[disparity_arr > 0] = 0.2 * 2262 / disparity_arr[disparity_arr > 0]
    depth_arr = np.astype(np.uint8)
    
    return depth_arr