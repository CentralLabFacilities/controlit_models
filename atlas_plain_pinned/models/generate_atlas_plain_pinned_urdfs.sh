#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p atlas_plain_pinned/urdf

echo Generating atlas_plain_pinned/urdf/atlas_plain_pinned_with_sandia_hands.urdf
rosrun xacro xacro.py atlas_plain_pinned/xacro/atlas_plain_pinned_with_sandia_hands.xacro -o atlas_plain_pinned/urdf/atlas_plain_pinned_with_sandia_hands.urdf

echo Done!