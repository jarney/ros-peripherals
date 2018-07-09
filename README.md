# USB ROS Peripherals

This package contains a set of ros-friendly peripherals.  The package contains
both the source-code for the ROS system as well as the physical
and firmware design for the devices.  The purpose is to provide an
architecture for devices to interconnect in a plug-and-play way
with the ROS system, thus providing an extensible platform
upon which further developments can take place.

The goal of the architecture is to attempt de-coupling the logical
operation of the peripherals from the physical operation by standardizing
on the USB bus with a serial protocol for device control.  Each
device is then controlled at a low-level by an Atmel microcontroller
(Arduino) and acts as the "standard" de-coupling mechanism allowing
the logical ROS system to operate at a logical level while allowing
device drivers to operate at a physical level.

# Building ROS package

```
cd ~/catkin_ws/src
git clone https://github.com/jarney/ros-peripherals.git
cd ..
source devel/setup.bash
catkin_make
```

# Building Devices

To build the Arduino firmware and the hardware, refer to each subdirectory
where more detailed instructions can be found.

# Installation
The node needs to run as the user root to access GPIO, hence the
following unconventional steps:

```
sudo chown root ~/catkin_ws/devel/lib/ros-peripherals/*
sudo chmod 4755 ~/catkin_ws/devel/lib/ros-peripherals/*
```

# Running

```
roslaunch ros-peripherals ros-peripherals.launch
```

