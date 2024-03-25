// This file uses pybind11 to create a python interface to the specific LCM types we use to talk to the robot.
// In contrast to generating python messages using lcm-gen, this file allows us to use the same C++ code to
// publish and subscribe to messages in python.
// When I tried regenerating them and using the python LCM library, I could get it to work.