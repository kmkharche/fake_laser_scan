# Fake Laser Scan Publisher

## Overview

This package publishes a fake 2D laser scan message and simultaneously broadcasts a dynamic transform between two frames. The node publishes two laser scans and one dynamic transform:
- The first fake scan is a circular laser scan with an increasing and decreasing radius, which is easy to visualize.
- The second fake scan is a more random laser scan which is more similar to a typical real laser scan, when the robot is surrounded by objects at different distances.
- The published dynamic transform moves the robot frame and the laser frame along the four sides of a square one by one.

The package has been tested with Ubuntu 16.04 with a ROS Kinetic setup.

**Keywords**: 2D laser scan, dynamic transform

### Installation

Set up a ROS workspace, and clone the package into the src folder of the workspace. Switch to the folder containing the workspace and:
```sh
cd workspace/
catkin build
```

You might need to install the catkin_tools package if catkin build does not work:
```sh
sudo apt-get install python-catkin-tools
```

### Usage

The main code which runs both the fake laser scan and dynamic transform can be run as follows:
```sh
roslaunch fake_laser_scan scan_with_tf.launch
```

## Package Information

### Config files

**scan_params.yaml**: The file contains parameters of the laser scanner, specifically the number of readings in one scan, the laser frequency at which the points are scanned, the minimum range of the laser, the maximum range of the laser and the frame id of the laser.

**tf_params.yaml**: The file contains the parameters to define the frames between which the dynamic transform is published.

**rviz_config.rviz**: The configuration file for rviz, which shows the TF tree and the fake laser scans.

### Launch files

**scan_with_tf.launch**: The launch file starts all the relevant nodes **static_transform_publisher**, **scan_with_tf_publisher** and **rviz** as described below.

### Nodes
**static_tranform_publisher**: Publishes a static transform between the robot and the laser scanner. Currently, the static transform is set up between the frames "base_link" and "laser". These frames will have to be changed if the frame names in the config files are changed.

**scan_with_tf_publisher**: Publishes the dynamic transform between "map" and "base_link". Publishes the fake laser scans on the topics "/scan" and "/scan_random".

**rviz**: Starts rviz with the given config file. The rviz config has the random laser scan added to the topics list in the left toolbar of rviz, but it is not being displayed by default. You can display the scan, by ticking the check mark next to the second laser scan topic in the rviz list.

### Published Topics

* **`/scan`**: The circular laser scan is published on this topic.
* **`/scan_random`**: The randomized laser scan is published on this topic.

