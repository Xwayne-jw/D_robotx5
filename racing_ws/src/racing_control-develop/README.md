English| [简体中文](./README_cn.md)

# Function Introduction

This package controls the car for line following and obstacle avoidance by simultaneously receiving messages from the track recognition node and obstacle recognition node.

# Instructions

## Preparation

1. Have a real robot or robot simulation module, including a motion base, camera, and RDK kit, and ensure that it operates normally.
2. Have ROS low-level driver installed, so the robot can receive "/cmd_vel" commands for motion and move correctly based on the commands.

## Installation of the Package

**1. Install the Package**

After starting the robot, connect to the robot via SSH or VNC in the terminal, click the "One-Click Deployment" button at the top right of this page, copy and run the following command on the RDK system to install the related nodes.

```bash
sudo apt update
sudo apt install -y tros-racing-control
```

**2. Run Automatic Line Following and Obstacle Avoidance**

```shell
source /opt/tros/local_setup.bash

# Simulation
ros2 launch racing_control racing_control_simulation.launch.py

# Real scenario (parameters can be set via command line at startup)
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=0.2 avoid_linear_speed:=0.1 follow_angular_ratio:=-1.0 follow_linear_speed:=0.1
```


# Interface Description

## Topics

### Published Topics

| Name                          | Message Type                                                | Description                                            |
| ----------------------------- | ----------------------------------------------------------- | ------------------------------------------------------ |
| /cmd_vel                     | geometry_msgs/msg/Twist                                    | Publishes control messages for the car                 |

### Subscribed Topics
| Name                          | Message Type                                                | Description                                            |
| ----------------------------- | ----------------------------------------------------------- | ------------------------------------------------------ |
| racing_track_center_detection | ai_msgs::msg::PerceptionTargets                            | Receives messages about the position of the track center |
| racing_obstacle_detection     | ai_msgs/msgs/PerceptionTargets                              | Receives position information of the detected obstacles    |## Parameters

| Parameter Name          | Type       | Description                                               |
| ----------------------- | ----------- | ----------------------------------------------------- |
| pub_control_topic        | string    | The name of the control message to publish, please configure according to the actual topic name, default value is /cmd_vel |
| avoid_angular_ratio      | float     | The ratio of angular velocity during obstacle avoidance, please configure according to the actual situation, default value is 1.1 |
| avoid_linear_speed       | float     | The linear speed during obstacle avoidance, please configure according to the actual situation, default value is 0.25 |
| follow_angular_ratio     | float     | The ratio of angular velocity during line following, please configure according to the actual situation, default value is -1.0 |
| follow_linear_speed      | float     | The linear speed during line following, please configure according to the actual situation, default value is 1.5 |
| bottom_threshold         | int       | The coordinate threshold (y-value of the target bottom coordinate) to trigger obstacle avoidance, please configure according to the actual situation, default value is 340 |
| confidence_threshold     | float     | The confidence threshold of obstacles to trigger obstacle avoidance, please configure according to the actual situation, default value is 0.5 |