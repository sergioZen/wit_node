# wit_node

wit_node fork that fixes erros for working with TF2, and WT901B 9600 bauds


This is a fork from:

https://github.com/yowlings/wit_node



## Dependencies and Install

1. ROS
2. ros-<distro>-ecl

Install ecl by

```bash
sudo apt install ros-<distro>-ecl
```



## Usage

Launch the only ROS launch file:

```bash
roslaunch wit_node wit.launch
```

About parameter:

-port

This is the port that device name in Linux system, for example the default port name is "/dev/ttyUSB0"







