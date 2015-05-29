#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p valkyrie_upperbody/urdf

echo Generating valkyrie_upperbody/urdf/valkyrie_upperbody_gazebo.urdf
rosrun xacro xacro.py valkyrie_upperbody/xacro/valkyrie_upperbody_gazebo.xacro -o valkyrie_upperbody/urdf/valkyrie_upperbody_gazebo.urdf

echo Done!