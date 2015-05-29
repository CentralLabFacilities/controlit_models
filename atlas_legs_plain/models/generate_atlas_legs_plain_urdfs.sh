#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p atlas_legs_plain/urdf

echo Generating atlas_legs_plain/urdf/atlas_legs_plain_gazebo.urdf
rosrun xacro xacro.py atlas_legs_plain/xacro/atlas_legs_plain_gazebo.xacro -o atlas_legs_plain/urdf/atlas_legs_plain_gazebo.urdf

echo Done!