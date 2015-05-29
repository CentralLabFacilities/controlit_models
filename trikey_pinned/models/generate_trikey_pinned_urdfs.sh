#!/bin/bash

echo Creating directory \'urdf\'
mkdir -p trikey_pinned/urdf

echo Generating trikey_pinned/urdf/trikey_pinned.urdf
rosrun xacro xacro.py trikey_pinned/xacro/trikey_pinned.xacro -o trikey_pinned/urdf/trikey_pinned.urdf

echo Done!