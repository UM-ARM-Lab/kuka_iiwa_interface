## Usage

This will set the IP address in use for recieving data from the robot to `10.10.10.108`. Requires changes in the Java
code as well to use this, as the robot uses hard coded values for `kukacontrol.local`'s IP address.

`ros2 launch victor_hardware dual_arm_lcm_bridge.launch left_arm_recv_url_override:=true left_arm_recv_url:=udp://10.10.10.108:30002 right_arm_recv_url_override:=true right_arm_recv_url:=udp://10.10.10.108:3001`
