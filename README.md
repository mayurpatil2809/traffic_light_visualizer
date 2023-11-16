# traffic_light_visualizer
# Spat Profile Publisher

## Overview

- This ROS package, `spat_profile_pub`, is designed for publishing/visualizing traffic lights along a route. It includes the functionality of publishing traffic light state, duration and next state, and traffic light visualization. 
- The Traffic Light programs are taken from a calibrated SUMO traffic model 
- The assumption is every traffic light phase duration adds up to 90 seconds and your vehicle can interact with different phases/profiles in 90 ways. So, you can launch this program by inputting a profile_number between 0-90.

## Features

- Traffic light state publishing to ROS topics
- Visual representation of traffic light states

### Dependencies

- ROS Noetic
- rospy
- tkinter

### Building from Source

#### ROS Noetic

1. Ensure that ROS Noetic is installed on your system. Follow the instructions [here](http://wiki.ros.org/noetic/Installation).

2. Create a catkin workspace (if you haven't already):

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    ```

3. Clone this repository into your catkin workspace:

    ```bash
    git clone https://github.com/mayurpatil2809/traffic_light_visualizer.git spat_profile_pub
    ```

4. Build your workspace:

    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

5. Source the setup script:

    ```bash
    source devel/setup.bash
    ```

## Usage

Run the package using:

```bash
roslaunch spat_profile_pub start_traffic_lights.launch profile_number:=<profile_number>
```
Replace <profile_number> with the desired profile number to start the simulation.

## Configuration
The traffic_lights_config.yaml file in the config folder can be modified to change the traffic light states and durations.

## Nodes
# spat_publisher
Published Topics
/spat_data ([spat_profile_pub/SpaT])

Publishes the state of each traffic light.

## Visualizing Traffic Light States
The package includes a GUI-based traffic light visualizer. When you launch the package, the visualizer displays the current state of each traffic light in a window.
