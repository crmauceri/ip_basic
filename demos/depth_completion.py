import glob
import os
import sys
import time

import cv2
import numpy as np
import png

from ip_basic import depth_map_utils
from ip_basic import vis_utils
from ip_basic.depth_loader import cityscapes_disparity_to_depth, kitti_depth_read


def main(input_depth_dir, output_depth_dir, mode="gaussian", save_output=True, subsample=0.05, dataset="cityscapes"):
    """Depth maps are saved to the 'output_depth_dir' folder.
    """

    ##############################
    # Options
    ##############################

    # Fast fill with Gaussian blur @90Hz (paper result)
    if mode == "gaussian":
        fill_type = 'fast'
        extrapolate = True
        blur_type = 'gaussian'

    # Fast Fill with bilateral blur, no extrapolation @87Hz (recommended)
    elif mode == "fast_bilateral":
        fill_type = 'fast'
        extrapolate = False
        blur_type = 'bilateral'

    # Multi-scale dilations with extra noise removal, no extrapolation @ 30Hz
    elif mode == "multiscale_bilateral":
        fill_type = 'multiscale'
        extrapolate = False
        blur_type = 'bilateral'

    else:
        raise ValueError("Mode not implemented: " + mode)

    ##############################
    # Processing
    ##############################
    if save_output:
        # Save to Disk
        show_process = False
        save_depth_maps = True
    else:
        if fill_type == 'fast':
            raise ValueError('"fast" fill does not support show_process')

        # Show Process
        show_process = True
        save_depth_maps = False

    # Get images in sorted order
    print("Input dir: " + input_depth_dir)
    if dataset == "cityscapes":
        images_to_use = sorted(glob.glob(input_depth_dir + '/*/*/*.png'))
    elif dataset == "kitti":
        images_to_use = sorted(glob.glob(input_depth_dir +'*/*/proj_depth/velodyne_raw/image_*/*.png'))
    else:
        images_to_use = sorted(glob.glob(input_depth_dir + '/*.png'))
    print("%d images found".format(len(images_to_use)))

    # Create output folder
    if save_output:
        finished_images = []
        if not os.path.exists(output_depth_dir):
            os.makedirs(output_depth_dir)
        elif dataset == "kitti":
            img_dirs = set([os.path.dirname(x.replace('velodyne_raw', 'ip_complete')) for x in images_to_use])
            for x in img_dirs:
                if not os.path.exists(x):
                    os.makedirs(x)
            finished_images = [x.replace('velodyne_raw', 'ip_complete') for x in images_to_use if
                               os.path.exists(x.replace('velodyne_raw', 'ip_complete'))]
        else:
            finished_images = [x for x in images_to_use if
                               os.path.exists(x.replace(input_depth_dir, output_depth_dir))]
        images_to_use = list(set(images_to_use).difference(finished_images))
        print('Output dir:', output_depth_dir)
        print("%d unprocessed images".format(len(images_to_use)))

    # Rolling average array of times for time estimation
    avg_time_arr_length = 10
    last_fill_times = np.repeat([1.0], avg_time_arr_length)
    last_total_times = np.repeat([1.0], avg_time_arr_length)

    num_images = len(images_to_use)
    for i in range(num_images):

        depth_image_path = images_to_use[i]

        # Calculate average time with last n fill times
        avg_fill_time = np.mean(last_fill_times)
        avg_total_time = np.mean(last_total_times)

        # Show progress
        sys.stdout.write('\rProcessing {} / {}, '
                         'Avg Fill Time: {:.5f}s, '
                         'Avg Total Time: {:.5f}s, '
                         'Est Time Remaining: {:.3f}s'.format(
                             i, num_images - 1, avg_fill_time, avg_total_time,
                             avg_total_time * (num_images - i)))
        sys.stdout.flush()

        # Start timing
        start_total_time = time.time()

        # Load depth projections from uint16 image
        if dataset == "cityscapes":
            projected_depths = cityscapes_disparity_to_depth(depth_image_path)
        elif dataset == "kitti":
            projected_depths = kitti_depth_read(depth_image_path)
        else:
            depth_image = cv2.imread(depth_image_path, cv2.IMREAD_ANYDEPTH)
            projected_depths = np.float32(depth_image / 256.0)

        # Simulate sparse depth
        if subsample < 1.0:
            n = int(projected_depths.shape[0] * projected_depths.shape[1])
            index = np.random.choice(n, int(np.floor(n * subsample)), replace=False)
            projected_depths.flat[index] = 0

        # Fill in
        start_fill_time = time.time()
        if fill_type == 'fast':
            final_depths = depth_map_utils.fill_in_fast(
                projected_depths, extrapolate=extrapolate, blur_type=blur_type)
        elif fill_type == 'multiscale':
            final_depths, process_dict = depth_map_utils.fill_in_multiscale(
                projected_depths, extrapolate=extrapolate, blur_type=blur_type,
                show_process=show_process)
        else:
            raise ValueError('Invalid fill_type {}'.format(fill_type))
        end_fill_time = time.time()

        # Display images from process_dict
        if fill_type == 'multiscale' and show_process:
            img_size = (570, 165)

            x_start = 80
            y_start = 50
            x_offset = img_size[0]
            y_offset = img_size[1]
            x_padding = 0
            y_padding = 28

            img_x = x_start
            img_y = y_start
            max_x = 1900

            row_idx = 0
            for key, value in process_dict.items():

                image_jet = cv2.applyColorMap(
                    np.uint8(value / np.amax(value) * 255),
                    cv2.COLORMAP_JET)
                vis_utils.cv2_show_image(
                    key, image_jet,
                    img_size, (img_x, img_y))

                img_x += x_offset + x_padding
                if (img_x + x_offset + x_padding) > max_x:
                    img_x = x_start
                    row_idx += 1
                img_y = y_start + row_idx * (y_offset + y_padding)

                # Save process images
                cv2.imwrite('process/' + key + '.png', image_jet)

            cv2.waitKey()

        # Save depth images to disk
        if save_depth_maps:
            # Save depth map to a uint16 png (same format as disparity maps)
            if dataset == "kitti":
                file_path = depth_image_path.replace('velodyne_raw', 'ip_complete')
            else:
                file_path = depth_image_path.replace(input_depth_dir, output_depth_dir)

            if not os.path.exists(os.path.dirname(file_path)):
                os.makedirs(os.path.dirname(file_path))

            with open(file_path, 'wb') as f:
                depth_image = (final_depths * 256).astype(np.uint16)

                # pypng is used because cv2 cannot save uint16 format images
                writer = png.Writer(width=depth_image.shape[1],
                                    height=depth_image.shape[0],
                                    bitdepth=16,
                                    greyscale=True)
                writer.write(f, depth_image)

        end_total_time = time.time()

        # Update fill times
        last_fill_times = np.roll(last_fill_times, -1)
        last_fill_times[-1] = end_fill_time - start_fill_time

        # Update total times
        last_total_times = np.roll(last_total_times, -1)
        last_total_times[-1] = end_total_time - start_total_time


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="""
        Depth completion for all sparse depth images in directory. 
        """)
    parser.add_argument("input_dir", help="Path to sparse depth images")
    parser.add_argument("output_dir", help="Path to output dir")
    parser.add_argument("mode", help="Upsampling mode: (gaussian, fast_bilateral, multiscale_bilateral)", default="fast_bilateral")
    parser.add_argument("--dataset", default='cityscapes', required=False,
                        help="Name of dataset being processed [cityscapes, kitti, ...]")
    args = parser.parse_args()

    main(args.input_dir, args.output_dir, args.mode, dataset=args.dataset)
