#!/bin/bash

for lcmdef in *.lcm
do
    lcm-gen -x --cpp-std c++11 $lcmdef
    lcm-gen -j $lcmdef
done

echo "Removing previously generated types"
rm -r ../lcm_types/victor_lcm_interface

echo "Moving generated types"
mv ./victor_lcm_interface ../lcm_types/
