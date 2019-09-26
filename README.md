# This is the Mantra manipulator's ROS package repository, which contains the driver of Mantra. 

#### Mantra is a 7-DOF manipulator developed and designed by Intelligent Robot Laboratory of HUST AIA.

###### Function list：

​	1. Communicate with robot through Ethernet and use Modbus-TCP protocol

​	2. Communicate with gripper through SerialPort and use RS485 protocol

​	3. Communicate with ROS Moveit！through ROS Topic and Action

​	4. Communicate with HMI through ROS Topic



#### Some commands to use this package:

1、Start the Mantra's driver.

```
roslaunch mantra_driver mantra_driver.launch
```

2、Build the driver package only.

```
cd ~/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="mantra_driver"
```

