#!/bin/bash
sudo -S chmod 666 /dev/ttyUSB*
#sudo -S chmod a+rw /dev/input/js*
source devel/setup.bash
roslaunch smartcar smartcar.launch
read -p "按回车键返回"
