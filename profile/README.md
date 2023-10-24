# Overview
This document provides a high-level **overview of the hardware and software components** developed by the members of the **Collective Dynamics and Control Lab (CDCL)** at the **University of Maryland**, College Park (UMD) and **deployed on a BostonDynamics Spot®**.

The <ins>overall goal of this project</ins> is to have <ins>Spot coordinate with a swarm of UAVs to autonomously search an area for so-called "targets" (cars, trucks, people, etc.)</ins>.

## Hardware Overview
![Image of Spot Standing in Lab with labels]()

_Figure 1: Labelled image of BostonDynamics Spot® with hardware payload._

The following section provides an overview of the hardware payload mounted on the back of the BostonDynamics Spot®.

### BostonDynamics Spot® Robot
[]()

### BostonDynamics SpotCORE® Companion Computer
[Boston Dynamics documentation](https://support.bostondynamics.com/s/article/Spot-CORE-payload-reference


### NVIDIA Jetson Xavier® Companion Computer
- [ ] create a table of links
[Jetson FAQ](https://developer.nvidia.com/embedded/faq)


### Emlid M+® RTK Unit

If not configured before, the M+ module will create a hotspot and you can connect to it with password `emlidreach` for setup. Additional notes and steps [here](https://docs.emlid.com/reach/before-you-start/first-setup/) for referece.

The Emlid M+ can then be configured on the Emlid Flow app to connect to a WiFi network (which is recommended for firmware updates). At the time of writing, we have configured the M+ as follows:
- Correction input: Off
- Base output: Off
- Base settigs (Default): Average SINGLE, Antenna height 0m, Averaging time 2 minutes.
- Settings > GNSS Settings (Default): KINEMATIC, Elevation mask angle 15, SNR mask 35, GPS AR mode Fix-and-hold, GLONASS AR mode off, Vert. acc. 1, Hor. acc. 1, All GNSS systems except BEIDOU, GNSS update rate 5Hz.
- Settings > Postion streaming 1: Serial with USB to PC, 38400 baud, NMEA.
- Settings > Position streaming 2: Off


### Doolde Labs Embedded Mesh Rider® Radio


### Velodyne Puck® LiDAR
[](https://velodynelidar.com/products/puck/)

### Intel RealSense D435i® Depth Camera
[]()


## Software Overview

## Logging into NVIDIA Jetson and SpotCORE
1. `ssh nvidia@?.?.?.?`
2. `ssh spot-user@192.168.2.2`

## Controlling Spot from Command Line
- start ```driver_no_lidar.launch``` on SpotCore
- Relinquish control on tablet
- Run ```rosservice call /spot/claim``` on NVIDIA or SpotCore
- Run ```rosservice call /spot/power_on``` (should see motors powering on on tablet)
- Run ```rosservice call /spot/stand``` (spot should then stand up)
- Run ```rosservice call /spot/allow_motion``` (one should now be able to command velocities from terminal to move spot)

### ```ros1_bridge``` Usage in Project
This project depends on both ROS1 and ROS2. Some ROS1 nodes depend on data published by ROS2 nodes, and some ROS2 nodes depend on data published by ROS1 nodes. Additionally, the requirements of the project necessitate publishing some data over ROS2. Therefore, it is necessary to make use of the ```ros1_bridge``` ROS2 package to translate ROS1 messages into ROS2 messages and vice versa.

This project makes use of the ```ros1_bridge``` package in two distinct ways: (1) to translate specific target messages from ROS1/ROS2 into ROS2/ROS1, and (2) and to define the translation of custom messages that have been created by this project, which need to be translated between ROS versions.

For the first use case, it is sufficient to define one executable per translation one would like to make. Such executables may be found in the ```ros1_bridge``` repository in the SPOT-CDCL org in GitHub, which was forked from the official repository.

For the second use case, it is common practice to fork the official ```ros1_bridge``` repository on GitHub and define the custom message translations in the forked version of the repo. This is necessary, because ```ros1_bridge``` will only translate messages whose translation has been defined at compile-time. In this way, it is necessary to bulid the ```ros1_bridge``` from source after having made some modifications to the code base defining those custom translations.

### Viewing URDF Files  
Run `roslaunch urdf_tutorial display.launch model:=filename.urdf.xacro` to open a GUI (Rviz) showing the URDF model and all coordinate frames associated with it.
![Screenshot from 2023-05-30 17-53-06](https://github.com/SPOT-UMD/.github/assets/27888732/a266352e-36d1-448e-b6bd-702525050b30)

### Frames
#### TF Tree when only ```spot_ros``` Runs
[tf_tree.pdf](https://github.com/SPOT-UMD/.github/files/11669388/tf_tree.pdf)

#### TF Tree when both ```spot_ros``` and ```spot_nav``` Run
[tf_tree_spot_nav.pdf](https://github.com/SPOT-UMD/.github/files/11669389/tf_tree_spot_nav.pdf)


### Software Dependencies
Spot ROS, <Description>, Link: https://github.com/heuristicus/spot_ros
  
### Networking Overview
The following section summarizes Spot's computer networks. Things we need to add: name each network, what data is shared, why that data needs to be shared (for what purpose is it shared), ips/ports, hardware interface (ethernet, wlan)
  
  ![Notes_230530_190655](https://github.com/SPOT-UMD/.github/assets/19653313/e3319560-2b42-4ae1-a440-1895a373f48a)
  
  ### Networking Configuration: SpotCore ```rostopics``` over "RAL_wifi"
  The goal of this section is to describe the procedure for configuring the [NETWORK_NAME_1] network on the SpotCore and the [NETWORK_NAME_2] network on your personal machine in order to "see" ```rostopics``` published by the SpotCore from your personal computer over "RAL_wifi". There are two steps to the procedure. See [this](https://answers.ros.org/question/256070/problems-with-communication-between-multiple-machines/) for details.
    
  > NOTE: "Step 1" is already done, so it is only necessary to do "Step 2", which is specific to your personal machine. Both steps are included for completeness and clarity.

  1. ```ssh``` into the SpotCore as the ```spot``` user (which has administrator privelages) and execute the following command to see the list of routes:
  ```bash
  route 
  ```
  You should get something like this if Step 1 has not been completed yet:
  ![spotcore_routes_original](https://github.com/SPOT-UMD/.github/assets/19653313/d1981cd1-f1d8-435d-a4ee-7184b18547ee)
  
  Now, execute the following command to add a route on the SpotCore to [NETWORK_NAME_2] network through the gateway ```192.168.2.3``` (which is the Jetson Xavier IP on the [NETWORK_NAME_1] network):
  ```bash
  sudo route add -net 192.168.79.0 netmask 255.255.255.0 gw 192.168.2.3
  ```
  Use the ```route``` command again to see the updated list of routes. It should look like this (IMAGE 2):
  
  Finally, re-```ssh``` into the SpotCore as the ```spot-user``` user, and start ```roscore``` on the SpotCore in the standard way:
  ```bash
  source /opt/ros/melodic/setup.bash
  source /home/spot/catkin_ws/devel/setup.bash  # NOTE: this is NOT in the "/home/spot-user/" sub-directory
  roslaunch spot_ros driver_no_lidar.launch
  ```
  
  2. In a terminal on your local machine, execute the following command to add a route on your machine to  [NETWORK_NAME_1] network through the gateway ```192.168.79.219``` (which is the Jetson Xavier IP on the [NETWORK_NAME_2] network):
  ```bash
  sudo route add -net 192.168.2.0 netmask 255.255.255.0 gw 192.168.79.219
  ```
  

  
