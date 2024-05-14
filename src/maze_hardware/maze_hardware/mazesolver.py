import rclpy, time, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

FORWARD = 0
LEFT = 1
RIGHT = 2

FORWARD_POS = 359
LEFT_POS = 539
RIGHT_POS = 179
BACK_POS = 0

TURN_SPEED = math.pi/16


class MazeSolver(Node):

    def __init__(self):
        super().__init__('mazesolver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        #self.create_timer(1.0, ) # Maze logic called every second
        
        self.cmdvel = Twist()
        self.distances = []
        self.prev_distances = []
        self.diff = []
        self.state = FORWARD
        self.forward_distance = 0

    def scan_callback(self, msg):
        self.prev_distances = self.distances
        self.distances = msg.ranges

        if len(self.prev_distances) > 0:
            self.diff = self.dis_diff(self.distances, self.prev_distances)

        if self.distances[FORWARD_POS] < .7 and self.state == FORWARD:
            self.forward_distance = self.distances[FORWARD_POS]
            print(f'Forward: {self.distances[FORWARD_POS]} Left: {self.distances[LEFT_POS]} Right: {self.distances[RIGHT_POS]} Values: {len(self.distances)}')
            if self.distances[RIGHT_POS] > self.distances[LEFT_POS]:      
                self.state = RIGHT
            else:
                self.state = LEFT

            return
        
        if self.state == FORWARD and len(self.diff) > 0 and self.diff[RIGHT_POS] > .1:
            self.turnRight()
        
        if self.state == FORWARD:
            self.moveForward()
        elif self.state == LEFT:
            self.turnLeft()
            #if abs(self.distances[RIGHT_POS] - self.forward_distance) < 1:
            self.state = FORWARD
        elif self.state == RIGHT:
            self.turnRight()
            #if abs(self.distances[LEFT_POS] - self.forward_distance) < 1:
            self.state = FORWARD

    def dis_diff(self, curr, prev):
        diff = []
        for idx, val in range(1, len(curr)):
            diff[idx] = curr[idx] - prev[idx]

        return diff

    def turnLeft(self): # Turns left very close to 90 degrees
        print('Turning Left!')
        self.cmdvel.angular.z = TURN_SPEED
        for i in range(44):
            self.pub.publish(self.cmdvel)
            time.sleep(0.2)
        self.cmdvel.angular.z = 0.0
        self.pub.publish(self.cmdvel)

    def moveForward(self): # Moves forward a bit
        print('Moving forward!')
        self.cmdvel.linear.x = 0.3
        for i in range(5):
            self.pub.publish(self.cmdvel)
            time.sleep(0.1)
        self.cmdvel.linear.x = 0.0
        self.pub.publish(self.cmdvel)

    def turnRight(self): # Turns right very close to 90 degrees
        print('Turning Right!')
        self.cmdvel.angular.z = -TURN_SPEED
        for i in range(44):
            self.pub.publish(self.cmdvel)
            time.sleep(0.2)
        self.cmdvel.angular.z = 0.0
        self.pub.publish(self.cmdvel)



def main(args=None):
    rclpy.init(args=args)
    mazesolver = MazeSolver()



    rclpy.spin(mazesolver)
    mazesolver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()