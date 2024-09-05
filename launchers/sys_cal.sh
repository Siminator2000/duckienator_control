#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
roslaunch my_package sys_cal.launch

# wait for app to end
dt-launchfile-join