# rt_usb_9axis_sensor

## About

ROS Package for RT-USB-9axisIMU and [RT-USB-9axisIMU2](https://www.rt-net.jp/products/9axisimu2/)

## Requirements

Setup [RT-USB-9axisIMU2](https://www.rt-net.jp/products/9axisimu2/) as ASCII output mode.  
[It seems](https://github.com/rt-net/rt_usb_9axisimu_driver/wiki#13-ver20%E3%81%A7%E3%81%AE%E3%81%94%E5%88%A9%E7%94%A8%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6) that it is already in ASCII output mode by default.  
Package for binary output mode has been published here: [rt-net/rt_usb_9axisimu_driver](https://github.com/rt-net/rt_usb_9axisimu_driver).

## Installation

Download this repository into `~/catkin_ws/src` and build it.

```
cd ~/catkin_ws/src
git clone https://github.com/AtsushiSaito/rt_usb_9axis_sensor.git
cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash
```

## Usage

Simply, connect the device to your PC and launch `rt_usb_9axis_sensor.launch`.

```
roslaunch rt_usb_9axis_sensor rt_usb_9axis_sensor.launch
```

### Options

By default, USB device is set to `/dev/ttyACM0`.  
If your device file is not `/dev/ttyACM0`, you can modify like the following:

```
roslaunch rt_usb_9axis_sensor rt_usb_9axis_sensor.launch port:="/dev/ttyACM1"
```

## License

This repository is licensed under the MIT license, see [LICENSE]( ./LICENSE ).

Unless attributed otherwise, everything in this repository is licensed under the MIT license.
