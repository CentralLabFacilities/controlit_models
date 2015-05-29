#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p atlas_plain/urdf

echo Generating atlas_plain/urdf/atlas_plain_with_sandia_hands_gazebo.urdf
rosrun xacro xacro.py atlas_plain/xacro/atlas_plain_with_sandia_hands_gazebo.xacro -o atlas_plain/urdf/atlas_plain_with_sandia_hands_gazebo.urdf

echo Generating atlas_plain/urdf/atlas_plain_with_sandia_hands_linked_to_world.urdf
rosrun xacro xacro.py atlas_plain/xacro/atlas_plain_with_sandia_hands_linked_to_world.xacro -o atlas_plain/urdf/atlas_plain_with_sandia_hands_linked_to_world.urdf

echo Done!