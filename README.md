### Motivation

Official Basler ROS driver is a troublemaker, so I decided to switch to this. However, I am using hardware triggering (right/slave camera is synchronized from a hardware trigger from the left/master camera) instead of synchronous free run.

![Used setup](setup.svg)

### Prerequisites

Add support for hardware trigger configuration in Pylon by following similar steps than in: https://github.com/basler/pylon-ros-camera/pull/46

### Run

Provides configuration files for launching a pair of cameras in hardware trigger mode. Has been tested with two Basler acA1920-50gc GigE cameras and Pylon 7.2.1 on Ubuntu 18.04 / ROS Melodic.

   ```bash
   roslaunch basler_camera stereo.launch
   ```
   
### Todo

I will soon that additionally IEEE1588 timestamping.
