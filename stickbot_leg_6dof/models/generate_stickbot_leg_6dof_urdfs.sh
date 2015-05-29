#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p stickbot_leg_6dof/urdf

echo Generating stickbot_leg_6dof/urdf/stickbot_leg_6dof.urdf
rosrun xacro xacro.py stickbot_leg_6dof/xacro/stickbot_leg_6dof.xacro -o stickbot_leg_6dof/urdf/stickbot_leg_6dof.urdf

echo Done!