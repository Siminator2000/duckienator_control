#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
roslaunch my_package main.launch

# wait for app to end
dt-launchfile-join