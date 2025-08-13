#!/bin/bash

#version_check
echo "left arm"
python3 script/version/can0.py
echo "right arm"
python3 script/version/can1.py