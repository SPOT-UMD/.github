# Overview
The documentation for the University of Maryland Spot (UMD-Spot) research project may be found here. 

## Viewing URDF Files
  
  Run `roslaunch urdf_tutorial display.launch model:=filename.urdf.xacro` to open a GUI (Rviz) showing the URDF model and all coordinate frames associated with it.
![Screenshot from 2023-05-30 17-53-06](https://github.com/SPOT-UMD/.github/assets/27888732/a266352e-36d1-448e-b6bd-702525050b30)

## Frames
### TF Tree when only ```spot_ros``` Runs
[tf_tree.pdf](https://github.com/SPOT-UMD/.github/files/11669388/tf_tree.pdf)

### TF Tree when both ```spot_ros``` and ```spot_nav``` Run
[tf_tree_spot_nav.pdf](https://github.com/SPOT-UMD/.github/files/11669389/tf_tree_spot_nav.pdf)


## Dependencies
Spot ROS, <Description>, Link: https://github.com/heuristicus/spot_ros
  
## Networking Overview
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
  

  
