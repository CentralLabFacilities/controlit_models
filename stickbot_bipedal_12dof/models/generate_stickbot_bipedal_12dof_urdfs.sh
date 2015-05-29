#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p stickbot_bipedal_12dof/urdf

echo Generating stickbot_bipedal_12dof/urdf/stickbot_bipedal_12dof.urdf
rosrun xacro xacro.py stickbot_bipedal_12dof/xacro/stickbot_bipedal_12dof.xacro -o stickbot_bipedal_12dof/urdf/stickbot_bipedal_12dof.urdf

echo Done!