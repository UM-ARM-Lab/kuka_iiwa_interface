#!/bin/bash

for lcmdef in *.lcm
do
    lcm-gen -x --cpp-std c++11 $lcmdef
    lcm-gen -p $lcmdef
    lcm-gen -j $lcmdef
done

echo "Removing previously generated types"
rm -r ../lcm_types/victor_lcm_interface

echo "Moving generated types"
mv ./victor_lcm_interface ../lcm_types/
echo "Moving python packages to match ament python package structure"
mv ../lcm_types/victor_lcm_interface/*.py ../victor_hardware/victor_lcm_interface/
