import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MazeSolver(Node):

    def __init__(self):
        super().__init__('mazesolver')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.distances = []

    def scan_callback(self, msg):
        self.distances = msg.ranges


def main(args=None):
    rclpy.init(args=args)
    mazesolver = MazeSolver()

    rclpy.spin(mazesolver)
    mazesolver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()