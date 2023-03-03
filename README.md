"Official" Basler ROS driver is a troublemaker, so I decided to switch to this. I added support for hardware trigger configuration following similar steps than in: https://github.com/basler/pylon-ros-camera/pull/46

Provides configuration files for launching a pair of cameras in hardware trigger mode. Has been tested with two Basler acA1920-50gc GigE cameras and Pylon 7.2.1 on Ubuntu 18.04 / ROS Melodic.

   ```bash
   roslaunch basler_camera stereo.launch
   ```
