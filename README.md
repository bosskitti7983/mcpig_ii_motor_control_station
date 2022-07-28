## This Folder contained dynamixel commands velocity code


## Installation guide:

This controller uses the latest version of the Dynamixel Workbench driver (May 2019). The following instructions are meant for Ubuntu 18 and ROS Melodic.

Install Main packages:

```sh
$ cd catkin_ws/src
$ git clone https://github.com/bosskitti7983/db_alpha_interface.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
``` 

Install dependent packages:

```sh
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git

Compile and build the pkgs:

```sh
$ cd ../
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

**NOTE:**

Before running any of the above code, the **USB latency between the computer and the robot** must be lowered. Run the following terminal commands to resolve this:

```sh
# Option A (recommended)
$ sudo usermod -aG dialout $USER && echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# Option B
$ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"1\" > 99-dynamixelsdk-usb.rules
$ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger --action=add
```

Then verify that the latency has been set to 1 with the following command:

```sh
$ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

Finally, update the permissions so that the serial connection can actually send and receive values:

```sh
$ sudo chmod a+rw /dev/ttyUSB0
```

See [Mathias' guide](https://github.com/MathiasThor/my_dynamixel_workbench/wiki/MORF-Software-Installation-Guide) for setting the automatic start of the of the serial port. 


## Configuration

 Before starting the driver, you need to configure the setup, including motor IDs, operating mode and return delay time. It is also possible to set up gains and acceleration and velocity profiles from the YAM file profile. See examples of configuration files in mcpig_ii_motor_control_station/config/_. 

 ```yaml
# Example of YAML configuration file

id_1:
  ID: 1
  Return_Delay_Time: 0
  Operating_Mode: 1
  Position_P_Gain: 400
  Position_I_Gain: 200
  Position_D_Gain: 50
  Profile_Velocity: 200
  Profile_Acceleration: 50
```

#### Official documentation:

 - **Install:** Go to [Official Dynamixel Workbench Manual](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/).


 - **Online Motor Control Table:** Go to [ROBOTIS e-manual Dynamixel XM430-W350](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#indirect-data).

 - **Online Motor Datasheet:** Go to [ROBOTIS XM430-W350 support](http://support.robotis.com/en/product/actuator/dynamixel_x/xm_series/xm430-w350.htm).


To start the velocity control driver interfaces 
run:

```
roslaunch mcpig_ii_motor_control_station velocity_controller.launch
```


<!-- ### image of files are in folders -->

<!-- ![Imgur](https://i.imgur.com/RA3aToz.png) -->



