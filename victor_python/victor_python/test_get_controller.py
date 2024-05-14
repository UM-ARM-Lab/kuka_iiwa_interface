import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from victor_python.victor import Victor, Side, ROBOTIQ_OPEN, ROBOTIQ_CLOSED

def main():
    rclpy.init()
    node = Node('test_get_controller')

    # executor = MultiThreadedExecutor()
    # executor.add_node(node)

    victor = Victor(node)
    print("Getting active")
    x = victor.left.get_active_controllers()
    y = victor.right.get_active_controllers()
    print(f"Left: {x}, Right: {y}")
    # rclpy.spin_once(node)
    # rclpy.spin(node)

    # for i in range(100):
    # t1 = time.time()
    # victor.left.get_active_controllers()
    # t2 = time.time()
    # victor.right.get_active_controllers()
    # t3 = time.time()
    # print(f"{i}-th  Left: {t2-t1}, Right: {t3-t2}")




if __name__ == '__main__':
    main()