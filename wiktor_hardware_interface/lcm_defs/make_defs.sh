#!/bin/bash

for lcmdef in *.lcm
do
    ~/lcm/lcmgen/lcm-gen -x --cpp-std c++11 $lcmdef
    ~/lcm/lcmgen/lcm-gen -j $lcmdef
done

for lcmcpp in *.hpp
do
    mv $lcmcpp ../lcm_types/wiktor_hardware_interface/
done

for lcmjava in *.java
do
    mv $lcmjava ../lcm_types/wiktor_hardware_interface/
done
