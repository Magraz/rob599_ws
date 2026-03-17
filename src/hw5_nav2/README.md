# ROB599 Homework 4

This repository the code for the individual part of assignment  of ROB599.

## Writeup

The write up is located at:

`~/hw5_nav2/HW4_Individual_Assignment_Writeup.pdf`

## Video demonstration

Video submission can be found at `~/hw5_nav2/videos/rob599_hw5.mp4`

## Setup

Make sure to place the `hw5_nav2` directory into a ros2 kilted workspace `src` folder that already has the stage simulator in it, then:

```bash
cd <ROS2_WORKSPACE>
colcon build --symlink-install
source install/setup.bash
```

### Running the Monte Carlo Localization

Run MCL launch file, teleop node, and the localization error plotter node, each in a different terminal.
In the terminal where the teleop node is running: Move the robot around to see the localization most-likely pose (red arrow) and particles (green arrows) in rviz:

```bash
ros2 launch hw5_nav2 mcl.launch.py world:=big_world
ros2 run stage_teleop teleop
ros2 run hw5_nav2 mcl_error_plot
```

### Pose resetting

To reset the pose estimation and randomize the particles. Use the 2D pose estimate tool inside RVIZ and set a pose on the map.
This will reset the pose estimation and show the new particles spread centered on the new pose estimate.


