#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p trikey/urdf

echo Generating trikey/urdf/trikey.urdf
rosrun xacro xacro.py trikey/xacro/trikey.xacro -o trikey/urdf/trikey.urdf

echo Done!