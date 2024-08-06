import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from victor_python.victor import Victor, Side, ROBOTIQ_OPEN, ROBOTIQ_CLOSED
from threading import Thread
import time

def main():
    rclpy.init()
    node = Node('test_get_controller')
    victor = Victor(node)
    mode = "async" # sync async
    if mode=="async":
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        spin_thread = Thread(target=executor.spin)
        spin_thread.start()
        print("Started spin thread")
    print("Getting active")
    t0 = time.time()
    for i in range(100):
        print(i)
        t1 = time.time()
        x = victor.left.get_all_controllers()
        t2 = time.time()
        y = victor.right.get_all_controllers()
        active_controller_name = victor.right.get_active_controller_names()
        # control_mode = victor.right.get_control_mode_for_controller(active_controller_name)
        t3 = time.time()
        print(f"{i}-th  Left: {t2-t1}, Right: {t3-t2}")
    print("Total time ", time.time()-t0)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()