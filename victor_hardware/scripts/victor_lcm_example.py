import lcm
from victor_lcm_interface import motion_command, motion_status

class VictorLCMExample:

    def __init__(self, send_lc, recv_lc):
        self.send_lc = send_lc
        self.recv_lc = recv_lc

        self.recv_lc.subscribe("motion_status", self.callback)

    def callback(self, channel, data):
        status = motion_status.decode(data)

        cmd = motion_command()

        cmd.control_mode = status.active_control_mode
        cmd.joint_position = status.measured_joint_position
        cmd.joint_position[-1] += 0.1
        cmd.joint_velocity = [0] * 7
        cmd.joint_velocity[-1] = 1.0

        input("Press enter to move the final joint by 0.1 radians")

        self.send_lc.publish("motion_command", cmd.encode())

def main():
    # The IP address here controls which arm we are sending commands to.
    send_lcm_url = "udpm://10.10.10.11"
    # This is our own IP address on the lab network.
    recv_lcm_url = "udpm://10.10.10.108"

    send_lc = lcm.LCM(send_lcm_url)
    recv_lc = lcm.LCM(recv_lcm_url)
    e = VictorLCMExample(send_lc, recv_lc)

    try:
        while True:
            recv_lc.handle()
            send_lc.handle()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()