# DTU Robocup 24

This package is based on ROS2 Humble. It aims is to provide the behaviour plans and image processing parts for the DTU Robocup 2024.
The repository organisation is under the supervision of **Geoffrey Côte**, but others students are participating in it.

This package heavily use the raubase_ros package as core.

This project is distributed under the MIT license.
Copyright © 2017 -2024 by DTU 

## Maintainer

This package is maintained by:

  - [Geoffrey Côte](https://github.com/Meltwin) ECN (General Engineering - Robotics) & DTU (MSc Autonomous Sytem) student 
  
## Dependencies

|   Package    |                                         Link                                         |
| :----------: | :----------------------------------------------------------------------------------: |
| ament_cmake  |                                    ROS2 Standard                                     |
|    rclpy     |                                    ROS2 Standard                                     |
| sensor_msgs  |                                    ROS2 Standard                                     |
|    OpenCV    |                    [OpenCV website](https://opencv.org/releases/)                    |
|  cv_bridge   |       [Vision OpenCV Github](https://github.com/ros-perception/vision_opencv)        |
| raubase_msgs | [Baxterminator - RobotBot MSG Github](https://github.com/Baxterminator/raubase_msgs) |
| raubase_ros  | [Baxterminator - RobotBot MSG Github](https://github.com/Baxterminator/raubase_ros)  |
| ultralytics  |          [Ultralytics Website](https://docs.ultralytics.com/fr/quickstart/)          |
|    Numpy     |                         [Numpy Website](https://numpy.org/)                          |

## Installation

To use this ROS2 package, you'll have install dependencies beforehand.

1. Install OpenCV by following the instructions on [this link](https://opencv.org/get-started/)
2. Install Python libraries:
```shell
python3 -m pip install numpy scipy ultralytics
```
3. Install ROS packages (if not already present) (one line at the time)
```shell
mkdir dtu_ws/src
cd dtu_ws/src
git clone git@github.com:ros-perception/vision_opencv.git -b humble
git clone git@github.com:Baxterminator/raubase_ros.git
git clone git@github.com:Baxterminator/raubase_msgs.git
cd ..
colcon build --symlink-install
```
4. Install this package
```shell
cd dtu_ws/src
git clone git@github.com:Baxterminator/dtu-robocup-24.git
cd ..
colcon build --symlink-install
```

You are now ready to go :)

## Running the executables

The first thing you'll have to do is to start the Raubase Stack on the robot. For that run the following commands on the robot:

```shell
ros2 launch raubase_ros full_stack.launch.py
```

Then you can run the commands from this package. All commands can be run from inside the robot, or on your computer (as long as you [configured the discovery server](https://baxterminator.github.io/raubase_ros/installation/multicast/)).

### Running the behavior plan

To run the behavior plan, you have to run the following command line:

```shell
ros2 launch dtu_robocup_24 client.launch.py
```

### Running the image processing server

To run the part that will process the images, run the following command:

```shell
ros2 launch dtu_robocup_24 server.launch.py
```