# ROB599 Homework 5

This repository the code for the individual part of assignment  of ROB599.

## Writeup

The write up is located at:

`hw5_nav2/HW5_Individual_Assignment.pdf`

## Video demonstration

Video submission can be found at `hw5_nav2/videos/rob599_hw5.mp4`

## Setup

Make sure to place the `hw5_nav2` directory into a ros2 kilted workspace `src` folder that already has the stage simulator in it, then:

```bash
cd <ROS2_WORKSPACE>
colcon build --symlink-install
source install/setup.bash
```

### Mapping Results
The generated house map using SLAM can be found in `hw5_nav2/maps/house.pgm`

### Running the Gazebo House environment with a Turtlebot3 Waffle.

The following launch file instantiates a Gazebo world with the house environment, and opens RVIZ for visualization of the generated map. IMPORTANT: Before any navigation, don't forget to set the initial pose for localization using the RVIZ "2D Pose Estimate Tool".

```bash
ros2 launch hw5_nav2 nav2_house.launch.py
```

### Running the /navigate_to_pose action client

With the Gazebo environment running, the go_to_goal node takes a coordinate and sends it to the /navigate_to_pose action server. This makes the agent navigate to the set goal.

```bash
ros2 run hw5_nav2 go_to_goal --ros-args -p x:=-4.0 -p y:=3.6
```

### Visiting all rooms and spinning in them.

With the Gazebo environment running, the visit_rooms node creates two action clients, one for the /navigate_to_pose action server, which takes care of reaching the room waypoint and another for the /spin action server, which takes care of spinning inside a room once the waypoint is reached.

```bash
ros2 run hw5_nav2 visit_rooms
```
