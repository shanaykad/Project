import rclpy, time, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MazeSolver(Node):

    def __init__(self):
        super().__init__('mazesolver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmdvel = Twist()
        self.distances = []
        self.sightdistance = 0.7 # maximum distance tb3 looks to see if wall exists
        self.leftWall = False
        self.frontWall = False
        self.rightWall = False
        self.create_timer(1.0, self.solver) # Maze logic called every second

    def scan_callback(self, msg):
        self.distances = msg.ranges

        # print('left dist: ' + str(self.distances[89])) # Left
        # print('front dist: ' + str(self.distances[0])) # Front
        # print('right dist: ' + str(self.distances[269])) # Right
        # print('rear dist: ' + str(self.distances[179])) # Rear

        if self.distances[89] < self.sightdistance: # Checks for a wall to the left
            self.leftWall = True
        else: 
            self.leftWall = False

        if self.distances[0] < self.sightdistance: # Checks for a wall in front
            self.frontWall = True
        else:
            self.frontWall = False

        if self.distances[269] < self.sightdistance: # Checks for a wall to the right
            self.rightWall = True
        else:
            self.rightWall = False


    def solver(self): # Put Logic here
        if self.frontWall == False:
            self.moveForward()
        elif self.frontWall == True and self.leftWall == False:
            self.turnLeft()
            self.moveForward()
        elif self.frontWall == True and self.rightWall == False:
            self.turnRight()
            self.moveForward()


    def turnLeft(self): # Turns left very close to 90 degrees)
        print('Turning Left!')
        self.cmdvel.angular.z = math.pi/8
        for i in range(23):
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
        self.cmdvel.angular.z = -math.pi/8
        for i in range(23):
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