import numpy as np
import cv2, argparse, os, re
from PIL import Image
from ip_basic.depth_loader import cityscapes_disparity_to_depth, kitti_depth_read, sunrgbd_depth_read
from matplotlib import pyplot as plt


def MAE(pred, gt):
    # Mean Absolute Error
    num_samples = np.count_nonzero(gt>0)
    mae = np.mean(np.abs(pred - gt))
    return mae, num_samples

def RMSE(pred, gt):
    # Root Mean Squared Error
    num_samples = np.count_nonzero(gt > 0)
    mse = np.mean(np.power(pred - gt, 2))
    rmse = np.sqrt(mse)
    return rmse, mse, num_samples

def eval_dataset(pred_filelist, gt_filelist, dataset='sunrgbd', root=''):
    num_samples_total = 0
    mae_total = 0
    mse_total = 0

    gt_pred_grid = np.zeros((2000, 2000))

    for ii, pred_file in enumerate(pred_filelist):
        # Load completed depth
        pred_depth = cv2.imread(os.path.join(root, pred_file), cv2.IMREAD_ANYDEPTH)

        # Load ground truth
        gt_file = os.path.join(root, gt_filelist[ii])

        if dataset == "cityscapes":
            gt_depth = cityscapes_disparity_to_depth(gt_file)
        elif dataset == "kitti":
            gt_depth = kitti_depth_read(gt_file)
        elif dataset == "sunrgbd":
            gt_depth = sunrgbd_depth_read(gt_file)
            pred_depth = pred_depth / 256.0
        else:
            gt_depth = cv2.imread(gt_file, cv2.IMREAD_ANYDEPTH) / 10.0
            pred_depth = pred_depth / 10.0

        mae, num_samples = MAE(pred_depth, gt_depth)
        rmse, mse, num_samples = RMSE(pred_depth, gt_depth)

        gt_pred_grid[[gt_depth.flatten().astype(int), pred_depth.flatten().astype(int)]] += 1

        mae_total += mae*num_samples
        mse_total += mse*num_samples
        num_samples_total += num_samples

        if ii%100==0:
            print("Completed {} of {}".format(ii, len(pred_filelist)))

    mae_final = mae_total / num_samples_total
    mse_final = mse_total / num_samples_total
    rmse_final = np.sqrt(mse_final)

    return mae_final, rmse_final, num_samples_total, gt_pred_grid

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Calculate MAE and RMSE for a list of depth images')
    parser.add_argument('pred_filelist', type=str,
                        help='path to txt file listing completed depth images')
    parser.add_argument('gt_filelist', type=str,
                        help='path to txt file listing ground truth depth images')
    parser.add_argument('--dataset', type=str, default='',
                        help='String dataset name: [cityscapes, kitti, sunrgbd]')
    parser.add_argument('--root', type=str, default='',
                        help='Any absolute path prefix required to locate files in filelists')
    parser.add_argument('--find_closest', action='store_true',
                        help='For filepaths with timestamps (rosbag extracted images), compare to closest timestamp')
    args = parser.parse_args()

    with open(args.pred_filelist, 'r') as f:
        pred_filelist = [s.strip() for s in f.readlines()]
    with open(args.gt_filelist, 'r') as f:
        gt_filelist = [s.strip() for s in f.readlines()]

    if args.find_closest:
        float_pattern = re.compile('\d+\.\d+')
        pred_stamps = np.array([float(float_pattern.findall(x)[0]) for x in pred_filelist])
        gt_stamps = np.array([float(float_pattern.findall(x)[0]) for x in gt_filelist])

        closest_preds = []
        for gt_stamp in gt_stamps:
            index = np.argmin(np.abs(pred_stamps-gt_stamp))
            closest_preds.append(pred_filelist[index])
        pred_filelist = closest_preds

    mae, rmse, num_samples, grid = eval_dataset(pred_filelist, gt_filelist, args.dataset, args.root)
    print('MAE: {}'.format(mae))
    print('RMSE: {}'.format(rmse))
    print('Num Samples: {}'.format(num_samples))
    np.savetxt("gt_vs_pred.csv", grid, delimiter=",")