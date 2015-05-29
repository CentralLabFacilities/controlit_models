#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p dreamer/urdf

echo Generating dreamer/urdf/dreamer_gazebo.urdf
rosrun xacro xacro.py dreamer/xacro/dreamer_gazebo.xacro -o dreamer/urdf/dreamer_gazebo.urdf

echo Generating dreamer/urdf/dreamer_linked_to_world.urdf
rosrun xacro xacro.py dreamer/xacro/dreamer_linked_to_world.xacro -o dreamer/urdf/dreamer_linked_to_world.urdf

echo Done!