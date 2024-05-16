# Approach: Follow right wall until maze has been exited
# as of rn this shit does not work at all, only spent an hour on it
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

        self.rightTol = 0.4
        self.frontTol = 0.4

        self.leftWallDist = 10.0
        self.frontWallDist = 10.0
        self.rightWallDist = 10.0
        self.finished = False # True when maze has been exited

        self.create_timer(1.0, self.solver) # Maze logic called every second
        

    def scan_callback(self, msg):
        self.distances = msg.ranges

        # print('left dist: ' + str(self.distances[89])) # Left
        # print('front dist: ' + str(self.distances[0])) # Front
        # print('right dist: ' + str(self.distances[269])) # Right
        # print('rear dist: ' + str(self.distances[179])) # Rear
        self.frontWallDist = min([self.distances[0:4], self.distances[354:359]])
        self.leftWallDist  = min(self.distances[89-5:89+5])
        self.rearWallDist = min(self.distances[179-5:179+5])
        self.rightWallDist = min(self.distances[269-5:269+5])


    def solver(self): # Put Logic here
        if self.distances == []:
            print('Array Empty')
            pass
        elif self.distances[0] == self.distances[89] and self.distances[0] == self.distances[269] or self.finished == True: # Checks if maze has been exited
            self.stop()
        elif self.rightWallDist < self.rightTol:
            self.turnLeft()
        elif self.frontWallDist > self.frontTol:
            self.moveForward()
        else:
            print('Confused')
            print(self.frontWall)
            print(self.leftWall)
            print(self.rightWall)


    def turnLeft(self): # Turns left very close to 90 degrees)
        print('Turning Left!')
        self.cmdvel.angular.z = math.pi/8
        for i in range(5):
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
        for i in range(5):
            self.pub.publish(self.cmdvel)
            time.sleep(0.2)
        self.cmdvel.angular.z = 0.0
        self.pub.publish(self.cmdvel)

    def stop(self):
        print("Stopping!")
        self.finished = True
        self.cmdvel.linear.x = 0.0
        self.cmdvel.angular.z = 1.0
        self.pub.publish(self.cmdvel)

    def corrector(self):
        linvel = 0.2
        angvel = 0.2

        if self.distances[0] < self.sightdistance:
            pass
        elif self.distances[89] < 0.4:
            print('Correcting!')
            self.cmdvel.angular.z = angvel
            self.cmdvel.linear.x = linvel
            self.pub.publish(self.cmdvel)
            time.sleep(0.5)
            self.cmdvel.angular.z = -angvel
            self.cmdvel.linear.x = linvel
            self.pub.publish(self.cmdvel)
            time.sleep(0.3)
        elif self.distances[269] < 0.4:
            print('Correcting!')
            self.cmdvel.angular.z = -angvel
            self.cmdvel.linear.x = linvel
            self.pub.publish(self.cmdvel)
            time.sleep(0.5)
            self.cmdvel.angular.z = angvel
            self.cmdvel.linear.x = linvel
            self.pub.publish(self.cmdvel)
            time.sleep(0.3)    




def main(args=None):
    rclpy.init(args=args)
    mazesolver = MazeSolver()



    rclpy.spin(mazesolver)
    mazesolver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()