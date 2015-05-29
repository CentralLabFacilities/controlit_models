#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p dreamer_testbed/urdf

echo Generating dreamer_testbed/urdf/dreamer_testbed.urdf
rosrun xacro xacro.py dreamer_testbed/xacro/dreamer_testbed.xacro -o dreamer_testbed/urdf/dreamer_testbed.urdf

echo Done!