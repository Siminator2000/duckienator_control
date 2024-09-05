#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
roslaunch my_package car_position.launch

# wait for app to end
dt-launchfile-join