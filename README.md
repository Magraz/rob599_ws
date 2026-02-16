<!-- # ROB599 Homework 2

This repository the code for assignment 2 of ROB599. I used the VFH algorithm.

## Writeup

The write up is located at:

`HW2_Individual Assignment_Writeup.pdf`

## Video Demonstration

Video submission can be found at:

`/videos/rob599_hw3.mp4`

## Setup

Make sure to place the ``src`` directory into a ros2 kilted workspace, then:

```bash
cd <ROS2_WORKSPACE>
colcon build
source install/setup.bash
```

## Running the Simulations

### Polkadot World

To run the robot navigation in the Polkadot environment:


```bash
ros2 launch hw2 polkadot.launch.py

```

### Graf201 World

To run the robot navigation in the Graf201 environment:

```bash
ros2 launch hw2 graf201.launch.py
```

## Starting the VFH Follower

After launching either world, start the VFH follower by calling the service:

```bash
ros2 service call /start_vfh_follower std_srvs/srv/SetBool "{data: true}"
``` -->

# ROB599 Homework 3

This repository the code for assignment 3 of ROB599.

## Writeup

The write up is located at:

`~/rob599_ws/writeups/HW3_Individual Assignment_Writeup.pdf`

## Video demonstration

Video submission can be found at:

`~/rob599_ws/videos/rob599_hw3.mp4`

## Setup

Make sure to place the ``src`` directory into a ros2 kilted workspace, then:

```bash
cd <ROS2_WORKSPACE>
colcon build
source install/setup.bash
```

## Running the simulations

### Mapping

Run mapper launch file and teleop node. Move the robot around to see the map update in rviz:


```bash
ros2 launch hw3 graf201_mapper.launch.py
ros2 run stage_teleop teleop

```

Once done mapping call the /save_map service to save the map:
```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

### Planning and following path

Rename the saved map and all related files (pgm, png, yaml) to "graf201". Then run the launch file:

```bash
ros2 launch hw3 graf201_load_and_plan.launch.py
```

Then load the map:

```bash
ros2 service call /load_map std_srvs/srv/Trigger
```

Once RVIZ loads up the map select a pose using RVIZ's 2D Goal Pose to set a goal to plan a path for. Then call the planning service:

```bash
ros2 service call /plan_path std_srvs/srv/Trigger
```

The path will be visualized in RVIZ marked by blue arrows. An image of the path will also be save to the `~/robws_599/src/hw3/maps/planned_path.png` file. Finally to follow the path run the waypoint follower service:

```bash
ros2 service call /start_vfh_follower std_srvs/srv/Trigger
```
