#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p valkyrie/urdf

echo Generating valkyrie/urdf/valkyrie_gazebo.urdf
rosrun xacro xacro.py valkyrie/xacro/valkyrie_gazebo.xacro -o valkyrie/urdf/valkyrie_gazebo.urdf

echo Done!