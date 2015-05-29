#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p omni_wheel/urdf

echo Generating omni_wheel/urdf/omni_wheel.urdf
rosrun xacro xacro.py omni_wheel/xacro/omni_wheel.urdf.xacro -o omni_wheel/urdf/omni_wheel.urdf

# echo Generating trikey/urdf/trikey_rapid.urdf
# rosrun xacro xacro.py trikey/xacro/trikey_rapid.xacro -o trikey/urdf/dreamer_rapid.urdf

echo Done!