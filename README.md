# rt_usb_9axis_sensor

## About

ROS Package for RT-USB-9axisIMU and [RT-USB-9axisIMU2](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3416)

## Requirements



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
