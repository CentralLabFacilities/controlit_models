#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p stickbot_upperbody_10dof/urdf

echo Generating stickbot_upperbody_10dof/urdf/stickbot_upperbody_10dof.urdf
rosrun xacro xacro.py stickbot_upperbody_10dof/xacro/stickbot_upperbody_10dof.xacro -o stickbot_upperbody_10dof/urdf/stickbot_upperbody_10dof.urdf

echo Done!