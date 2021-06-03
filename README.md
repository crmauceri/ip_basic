# Image Processing for Basic Depth Completion (IP-Basic)

This fork of [ip_basic]() adds 

- A python package of the ip_basic code
- Support for a number of datasets to the demos/depth_completion.py script including 
    - Cityscapes
    - KITTI 
    - SUNRGBD
- A ROS node which can be used in combination with [lidar2depth](crmauceri/lidar2depth)

If you use this code, please cite the original author's paper:
[In Defense of Classical Image Processing: Fast Depth Completion on the CPU](https://arxiv.org/abs/1802.00036)

```
@inproceedings{ku2018defense,
  title={In Defense of Classical Image Processing: Fast Depth Completion on the CPU},
  author={Ku, Jason and Harakeh, Ali and Waslander, Steven L},
  booktitle={2018 15th Conference on Computer and Robot Vision (CRV)},
  pages={16--22},
  year={2018},
  organization={IEEE}
}
```

For more details on the algorithm, see the paper or the [ip_basic readme]().

The rest of this readme is specific to this fork

## Setup and Installation

This fork has been tested with [ROS Melodic]() and Python 2.7 on a machine running Ubuntu 18.1

1. Install python dependancies. There is a requirements.txt that you can run with 

    ```
   pip install -r requirements.txt
    ```
   
2. Install python package

    ```
   pip install -e .
    ``` 
   
3. (for ROS node) 
   If you didn't clone the repo in your catkin workspace, make a symlink
   
   ```
   ln -s <repo_filepath> <catkin_workspace>/src/ 
   ```
   
   Compile catkin project

   ```
   cd <catkin_workspace>
   catkin_make
   ```

## Using the ROS Node

Node subscribes to topic `/depth_image` and publishes to topic `/completed_depth`. Both the input and output images should be `uint16` depth images.

To run, use 

```
rosrun ip_basic listener.py
```

If you want to make changes to the node, the code is in scripts/listener.py

## Using the demo scripts

... more details soon ... 