# Training Robot V1

This is the repository for the SmartSystems Lab's custom autonomous robot : Main Controler part.

## Before Using 

The robot automatically connects to the TP-Link_SSL WiFi on boot up. Make sure the Wifi is working or adjust robot and your computer at same network.

The base environment settings is in python 2.

The IP address for the robot is static: 192.168.1.107. Using ```ssh bobdabot@192.168.1.107``` on the terminal or connect to server directly with ```sftp://bobdabot@192.168.1.107/``` at Other Locations. The password of it is ```password1234```

## Start Setting

After connecting to the robot, set your terminal workspace using ```cd training_robot_ws``` in terminal and do the ```catkin_make``` before you first using this file. 

Set variable ROS_MASTER_URI AND ROS_IP using ```export ROS_MASTER_URI=http://(your ip address):11311``` and ```export ROS_IP=(your ip address)``` on the terminal of your computer and set variable ROS_MASTER_URI AND ROS_IP using ```export ROS_MASTER_URI=http://(your ip address):11311``` and ```export ROS_IP=(robot's ip address)``` on the terminal of the robot. You can check the ip address through ```ip addr``` . This option is required for all the terminals that you want to communicate with others.

Before the programe running one terminals should run ```roscore``` to start ros for the robot. **Directly running ```roslaunch rplidar_ros odrive_view_rplidar.launch``` may also work but it may not convience for debugging**

## Running the Programe

Every time before running, I recommand you to do ```source devel/setup.bash``` at your workspace. Then you can running ```roslaunch rplidar_ros odrive_view_rplidar.launch``` to get the visiable rplidar data, odom and map.

## TODO

### URDF
The urdf description of the robot may have a little error with the size, you can remeasure it and figure it out. But this error has less affect on program working.

### Motor-controller-interface
The transport part from cmd_vel to odom does not adjust value to the real robot odom, you can measure it and change the value through variable ```wheelbase_radius_meters``` and ```encoder_postions_per_meter``` in the motor-controller-interface.py to adjust it. It will take a long time, but not hard.

**This problem may contact with the problem at robot part [odirve_interface](https://github.com/NicoPowers/odrive_interface) package.**

### Gmapping
Map chould be created, but the path planning may not work smoothly. 

**This problem may also contact with the problem at robot part [odirve_interface](https://github.com/NicoPowers/odrive_interface) package.**

