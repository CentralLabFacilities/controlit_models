#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p stickbot_lowerleg_3dof/urdf

echo Generating stickbot_lowerleg_3dof/urdf/stickbot_lowerleg_3dof.urdf
rosrun xacro xacro.py stickbot_lowerleg_3dof/xacro/stickbot_lowerleg_3dof.xacro -o stickbot_lowerleg_3dof/urdf/stickbot_lowerleg_3dof.urdf

echo Done!