#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p dreamer_hand/urdf

echo Generating dreamer_hand/urdf/dreamer_hand.urdf
rosrun xacro xacro.py dreamer_hand/xacro/dreamer_hand.xacro -o dreamer_hand/urdf/dreamer_hand.urdf

echo Done!