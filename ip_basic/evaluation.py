import numpy as np
import cv2, argparse
from PIL import Image
from ip_basic.depth_loader import cityscapes_disparity_to_depth, kitti_depth_read, sunrgbd_depth_read

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

def eval_dataset(pred_filelist, gt_filelist, dataset='sunrgbd'):
    num_samples_total = 0
    mae_total = 0
    mse_total = 0

    for ii, pred_file in enumerate(pred_filelist):
        # Load completed depth
        pred_depth = cv2.imread(pred_file, cv2.IMREAD_ANYDEPTH)

        # Load ground truth
        gt_file = gt_filelist[ii]

        if dataset == "cityscapes":
            gt_depth = cityscapes_disparity_to_depth(gt_file)
        elif dataset == "kitti":
            gt_depth = kitti_depth_read(gt_file)
        elif dataset == "sunrgbd":
            gt_depth = sunrgbd_depth_read(gt_file)
        else:
            gt_depth = cv2.imread(gt_file, cv2.IMREAD_ANYDEPTH)

        mae, num_samples = MAE(pred_depth, gt_depth)
        rmse, mse, num_samples = RMSE(pred_depth, gt_depth)

        mae_total += mae*num_samples
        mse_total += mse*num_samples
        num_samples_total += num_samples

    mae_final = mae_total / num_samples_total
    mse_final = mse_total / num_samples_total
    rmse_final = np.sqrt(mse_final)

    return mae_final, rmse_final

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Calculate MAE and RMSE for a list of depth images')
    parser.add_argument('pred_filelist', type=str,
                        help='path to txt file listing completed depth images')
    parser.add_argument('gt_filelist', type=str,
                        help='path to txt file listing ground truth depth images')
    parser.add_argument('dataset', type=str,
                        help='String dataset name: [cityscapes, kitti, sunrgbd]')
    args = parser.parse_args()

    with open(args.pred_filelist, 'r') as f:
        pred_filelist = [s.strip() for s in f.readlines()]
    with open(args.gt_filelist, 'r') as f:
        gt_filelist = [s.strip() for s in f.readlines()]

    mae, rmse = eval_dataset(pred_filelist, gt_filelist, args.dataset)
    print('MAE: {}'.format(mae))
    print('RMSE: {}'.format(rmse))