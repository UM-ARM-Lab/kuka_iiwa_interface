#!/bin/bash

for lcmdef in *.lcm
do
    echo "Generating C++11 type for LCM def $lcmdef"
    ~/lcm/lcmgen/lcm-gen -x --cpp-std c++11 $lcmdef
    echo "Generating Java type for LCM def $lcmdef"
    ~/lcm/lcmgen/lcm-gen -j $lcmdef
done

echo "Removing previously generated types"
rm -r ../lcm_types/wiktor_hardware_interface

echo "Moving generated types"
mv ./wiktor_hardware_interface ../lcm_types/