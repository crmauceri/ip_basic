import glob
import os
import sys
import time

import cv2, png
import numpy as np

from ip_basic import depth_map_utils
from ip_basic.depth_loader import cityscapes_disparity_to_depth, kitti_depth_read, sunrgbd_depth_read

def main(input_depth_dir=None, output_depth_dir=None, textfile=None, fill_type='fast', extrapolate=True,
        blur_type='gaussian', save_output=True, subsample=0.05, dataset="cityscapes"):
    """Depth maps are saved to the 'output_depth_dir' folder.
    """

    ##############################
    # Options
    ##############################
    print("Save output: {}\nSubsample: {}\nDataset: {}".format(save_output, subsample, dataset))

    ##############################
    # Processing
    ##############################
    if textfile is not None:
        with open(textfile, 'r') as f:
            images_to_use = [x.strip() for x in f.readlines()]
            images_to_use.sort()
    elif os.path.isfile(input_depth_dir):
        images_to_use = [input_depth_dir]
    else:
        # Get images in sorted order
        print("Input dir: " + input_depth_dir)
        if dataset == "cityscapes":
            images_to_use = sorted(glob.glob(input_depth_dir + '/*/*/*.png'))
        elif dataset == "kitti":
            images_to_use = sorted(glob.glob(input_depth_dir +'/*/*/proj_depth/velodyne_raw/image_*/*.png'))
        elif dataset == "sunrgbd":
            images_to_use = glob.glob(input_depth_dir + '/*/*/*/depth/*.png')
            images_to_use.extend(glob.glob(input_depth_dir + '/*/*/*/*/*/depth/*.png'))
            images_to_use.sort()
        else:
            images_to_use = sorted(glob.glob(input_depth_dir + '/*.png'))

    print("{} images to process".format(len(images_to_use)))

    # Filter already processed images
    if save_output:
        if dataset == "kitti":
            finished_images = [x.replace('velodyne_raw', 'ip_complete') for x in images_to_use if
                               os.path.exists(x.replace('velodyne_raw', 'ip_complete'))]
        elif dataset == "sunrgbd":
            finished_images = [x.replace('depth', 'ip_complete') for x in images_to_use if
                               os.path.exists(x.replace('depth', 'ip_complete'))]
        else:
            print('Output dir:', output_depth_dir)
            finished_images = [x for x in images_to_use if
                               os.path.exists(x.replace(input_depth_dir, output_depth_dir))]
        images_to_use = list(set(images_to_use).difference(finished_images))
        print("{} unprocessed images".format(len(images_to_use)))

    # Rolling average array of times for time estimation
    avg_time_arr_length = 10
    last_fill_times = np.repeat([1.0], avg_time_arr_length)
    last_total_times = np.repeat([1.0], avg_time_arr_length)

    num_images = len(images_to_use)
    for i in range(num_images):
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

        depth_image_path = images_to_use[i]
        if dataset == "kitti":
            output_depth_path = depth_image_path.replace('velodyne_raw', 'ip_complete')
        elif dataset == "sunrgbd":
            output_depth_path = depth_image_path.replace('depth', 'ip_complete')
        elif dataset == "cityscapes":
            output_depth_path = depth_image_path.replace('disparity', 'completed_depth')
        else:
            output_depth_path = depth_image_path.replace(input_depth_dir, output_depth_dir)

        # Process the image
        start_fill_time, end_fill_time = complete_image(depth_image_path, output_depth_path,
                                                        fill_type, extrapolate, blur_type,
                                                        save_output, subsample, dataset)
        end_total_time = time.time()

        # Update fill times
        last_fill_times = np.roll(last_fill_times, -1)
        last_fill_times[-1] = end_fill_time - start_fill_time

        # Update total times
        last_total_times = np.roll(last_total_times, -1)
        last_total_times[-1] = end_total_time - start_total_time


def complete_image(depth_image_path, output_depth_path, fill_type='fast', extrapolate=True,
                   blur_type='gaussian', save_output=True, sample_percentage=0.05, dataset="cityscapes"):

    # Load depth projections from uint16 image
    if dataset == "cityscapes":
        projected_depths = cityscapes_disparity_to_depth(depth_image_path)
    elif dataset == "kitti":
        projected_depths = kitti_depth_read(depth_image_path)
    elif dataset == "sunrgbd":
        projected_depths = sunrgbd_depth_read(depth_image_path)
    else:
        depth_image = cv2.imread(depth_image_path, cv2.IMREAD_ANYDEPTH)
        projected_depths = np.float32(depth_image / 256.0)

    # Simulate sparse depth
    if sample_percentage < 1.0:
        n = int(projected_depths.shape[0] * projected_depths.shape[1])
        index = np.random.choice(n, int(np.floor(n * sample_percentage)), replace=False)
        projected_depths.flat[index] = 0

    # Fill in
    start_fill_time = time.time()
    if fill_type == 'fast':
        final_depths = depth_map_utils.fill_in_fast(
            projected_depths, extrapolate=extrapolate, blur_type=blur_type)
    elif fill_type == 'multiscale':
        final_depths, process_dict = depth_map_utils.fill_in_multiscale(
            projected_depths, extrapolate=extrapolate, blur_type=blur_type,
            show_process=False)
    else:
        raise ValueError('Invalid fill_type {}'.format(fill_type))
    end_fill_time = time.time()

    # Save depth images to disk
    if save_output:
        if not os.path.exists(os.path.dirname(output_depth_path)):
            os.makedirs(os.path.dirname(output_depth_path))

        with open(output_depth_path, 'wb') as f:
            depth_image = (final_depths * 256).astype(np.uint16)

            # pypng is used because cv2 cannot save uint16 format images
            writer = png.Writer(width=depth_image.shape[1],
                                height=depth_image.shape[0],
                                bitdepth=16,
                                greyscale=True)
            writer.write(f, depth_image)

    return start_fill_time, end_fill_time

if __name__ == "__main__":

    opt = input('This script can process: \n\t (1) all images in a directory or \n\t (2) a single image. or \n\t (3) a list of files in a text file\n'
          'Enter an option: ')
    if opt != '1' and opt != '2' and opt != '3':
        print('Valid options are 1, 2, or 3. User choose {}. Exiting program.'.format(opt))
        exit(1)

    mode = raw_input('Choose a mode: (1) gaussian (default), (2) fast_bilateral, (3) multiscale_bilateral\n')
    # Fast fill with Gaussian blur @90Hz (paper result)
    if mode in ["gaussian", '1', '']:
        fill_type = 'fast'
        extrapolate = True
        blur_type = 'gaussian'

    # Fast Fill with bilateral blur, no extrapolation @87Hz (recommended)
    elif mode in ["fast_bilateral", '2']:
        fill_type = 'fast'
        extrapolate = False
        blur_type = 'bilateral'

    # Multi-scale dilations with extra noise removal, no extrapolation @ 30Hz
    elif mode in ["multiscale_bilateral", '3']:
        fill_type = 'multiscale'
        extrapolate = False
        blur_type = 'bilateral'

    else:
        raise ValueError("Mode not implemented: " + mode)

    dataset = raw_input('Choose a dataset: (1) kitti, (2) cityscapes (default), (3) sunrgbd\n')
    if dataset in ['kitti', '1']:
        dataset = 'kitti'
    elif dataset in ['cityscapes', '2', '']:
        dataset = 'cityscapes'
    elif dataset in ['sunrgbd', '3']:
        dataset = 'sunrgbd'
    else:
        dataset = ''

    if opt == '1':
        input_dir = raw_input('Enter the input directory\n')
        output_dir = raw_input('Enter the output directory\n')
        main(input_dir, output_dir, dataset=dataset, fill_type=fill_type, extrapolate=extrapolate, blur_type=blur_type)
    elif opt == '2':
        in_file = raw_input('Enter the input image path\n')
        out_file = raw_input('Enter the output image path\n')
        complete_image(in_file, out_file, dataset=dataset, fill_type=fill_type, extrapolate=extrapolate, blur_type=blur_type)
    elif opt == '3':
        in_file = input('Enter the image list text file path\n')
        main(textfile=in_file, dataset=dataset, fill_type=fill_type, extrapolate=extrapolate,
                       blur_type=blur_type)
