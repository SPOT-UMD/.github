Overview
========

Dependencies
------------
Spot ROS, <Description>, Link: https://github.com/heuristicus/spot_ros

## Viewing URDF Files
  
  Run `roslaunch urdf_tutorial display.launch model:=filename.urdf.xacro` to open a GUI (Rviz) showing the URDF model and all coordinate frames associated with it.
![Screenshot from 2023-05-30 17-53-06](https://github.com/SPOT-UMD/.github/assets/27888732/a266352e-36d1-448e-b6bd-702525050b30)

  
## Networking 
  
  
  ![Notes_230530_190655](https://github.com/SPOT-UMD/.github/assets/19653313/e3319560-2b42-4ae1-a440-1895a373f48a)
  
  ### how to
  in order to get ```rostopics``` published by the SpotCore to your local machine over the RAL_wifi network, it is necessary to do the following:

  
  ssh into the spotcore in as the ```spot``` user and execute the following command to see the list of routes:
  ```bash
  route 
  ```
  You should get something like this:
  ![spotcore_routes_original](https://github.com/SPOT-UMD/.github/assets/19653313/d1981cd1-f1d8-435d-a4ee-7184b18547ee)
  
  ssh into the spotcore in as the ```spot``` user and execute the following command to see add __ to ___:
  ```bash
  sudo route add -net 192.168.79.0 netmask 255.255.255.0 gw 192.168.2.3
  ```
  
  Finally, start the roscore on the spotcore in the standard way (need to document):
  ```bash
  roslaunch spot_ros driver_no_lidar.launch
  ```
  
  In a terminal on your local machine, execute the following command to add __ to __:
  ```bash
  # sudo route add -net 192.168.2.0 netmask 255.255.255.0 gw <IP of XAVIER on RAL_wifi>
  sudo route add -net 192.168.2.0 netmask 255.255.255.0 gw 192.168.79.219
  ```
  

  
