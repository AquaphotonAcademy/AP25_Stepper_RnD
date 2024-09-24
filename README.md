# AP25_Stepper_RnD

## Control Stepper motor movement 


### About The Project 

This project controls stepper motor movement using publisher and subscriber. A Ros2 publisher node "stepper_node" sends desired position(in degrees) to microcontroller through topic "/stepper_control" . The microcontroller, running a micro-ROS subscriber, receives these commands and controls the stepper motor accordingly.

### Key Features

* Publish motor position commands in degrees via ROS 2.
* Subscriber on the microcontroller that moves the stepper motor based on received commands.
* Homing functionality for the stepper motor.
* Easily configurable for any stepper motor and microcontroller.

### Prerequisities
Ensure that the following are installed
* **ROS2 humble:** you can follow this tutorial on [Ros2 documentation](https://docs.ros.org/en/humble/Installation.html)
* **Micro ros:** follow this tutorial to install it [micro ros installation](https://micro.ros.org/docs/tutorials/core/first_application_linux/)


###  Installation

1. Clone the repo `git clone https://github.com/AquaphotonAcademy/AP25_Stepper_RnD.git`

2. Build The repo `colcon build`
3. source `source install/setup.bash`

### Setup 
#### To run publisher node
1. make sure you are in directory `cd AP25_Stepper_RnD`
2. run the node `ros2 run AP25_Stepper_RnD stepper_command`

#### To run subscriber 
1. make sure that micro ros agent is installed
2. upload subscriber code on microcontroller
3. run micro ros agent `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0`




   
>>>>>>> origin/main
