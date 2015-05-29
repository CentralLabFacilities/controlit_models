#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p stickbot_humanoid_32dof_pinned/urdf

echo Generating stickbot_humanoid_32dof_pinned/urdf/stickbot_humanoid_32dof_pinned.urdf
rosrun xacro xacro.py stickbot_humanoid_32dof_pinned/xacro/stickbot_humanoid_32dof_pinned.xacro -o stickbot_humanoid_32dof_pinned/urdf/stickbot_humanoid_32dof_pinned.urdf

echo Done!