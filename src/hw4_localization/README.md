# ROB599 Homework 4

This repository the code for the individual part of assignment 4 of ROB599.

## Writeup

The write up is located at:

`~/hw4_localization/HW4_Individual_Assignment_Writeup.pdf`

## Video demonstration

Video submission can be found at `~/hw4_localization/videos/rob599_hw4.mp4`

## Setup

Make sure to place the `hw4_localization` directory into a ros2 kilted workspace `src` folder that already has the stage simulator in it, then:

```bash
cd <ROS2_WORKSPACE>
colcon build
source install/setup.bash
```

Finally, install scipy:
```bash
pip install scipy
```
### Running the Monte Carlo Localization

Run MCL launch file, teleop node, and the localization error plotter node, each in a different terminal.
In the terminal where the teleop node is running: Move the robot around to see the localization most-likely pose (red arrow) and particles (green arrows) in rviz:

```bash
ros2 launch hw4_localization mcl.launch.py world:=big_world
ros2 run stage_teleop teleop
ros2 run hw4_localization mcl_error_plot
```

### Pose resetting

To reset the pose estimation and randomize the particles. Use the 2D pose estimate tool inside RVIZ and set a pose on the map.
This will reset the pose estimation and show the new particles spread centered on the new pose estimate.


