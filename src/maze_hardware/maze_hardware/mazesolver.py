# Approach: Move forward until a wall is reached, check if robot can go left or right and proceed in that direction until maze has been exited

import rclpy, time, math, random
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
        self.sightdistance = 0.8 # maximum distance tb3 looks to see if wall exists
        self.stuckCounter = 0

        self.turning = False
        self.previousTurn = 'Right'

        self.leftWall = False
        self.frontWall = False
        self.rightWall = False

        self.shouldTurnRight = False # Extra check to prevent bias from which code runs first
        self.finished = False # True when maze has been exited
        self.finishTol = 2.0
        self.create_timer(0.01, self.solver) # Maze logic called every tenth of a second

    def scan_callback(self, msg):
        self.distances = msg.ranges

        # print('left dist: ' + str(self.distances[539])) # Left
        # print('front dist: ' + str(self.distances[359])) # Front
        # print('right dist: ' + str(self.distances[179])) # Right
        # print('rear dist: ' + str(self.distances[0])) # Rear

        if min(self.distances[(539-120):(539+30)]) < self.sightdistance - 0.1: # Checks for a wall to the left
            self.leftWall = True
        else: 
            self.leftWall = False
        if min(self.distances[(359-60):(359+60)]) < self.sightdistance - 0.2: # Checks for a wall in front
            self.frontWall = True
        else:
            self.frontWall = False
        if min(self.distances[(179-30):(179+120)]) < self.sightdistance - 0.1: # Checks for a wall to the right
            self.rightWall = True
        else:
            self.rightWall = False

        if sum(self.distances[(179+40):(179+100)]) > sum(self.distances[(539-100):(539-40)]): # Checks to see if robot should turn left or right next
            self.shouldTurnRight = True
        else:
            self.shouldTurnRight = False

    def solver(self): # Put Logic here
        if self.distances == []:
            print('Array Empty')
            pass
        elif self.finished_directions() or self.finished == True: # Checks if maze has been exited
            self.turning = False
            self.stop()
        elif self.frontWall == False or self.stuckCounter > 3:
            self.turning = False
            self.moveForward()
            self.stuckCounter = 1
        elif self.frontWall == True and self.rightWall == True and self.shouldTurnRight == False:
            self.turning = False
            self.turnLeft()
        elif self.frontWall == True and self.leftWall == True:
            self.turning = False
            self.turnRight()
        elif self.frontWall == True and self.leftWall == False and self.rightWall == False:
            self.turning = True
            if self.previousTurn == 'Right':
                self.turnLeft()
            elif self.previousTurn == 'Left':
                self.turnRight()
        else:
            self.pause()
            print('Confused')
            print(self.frontWall)
            print(self.leftWall)
            print(self.rightWall)
            self.stuckCounter += 1

    def turnLeft(self): # Turns left a bit
        print('Turning Left!')
        self.cmdvel.linear.x = 0.0
        self.cmdvel.angular.z = 1.3
        self.pub.publish(self.cmdvel)
        if self.stuckCounter % 1 == 0: # Prevents tb3 getting stuck deciding which direction to turn
            self.stuckCounter += 1
        if self.turning == False:
            self.previousTurn = 'Left'

    def moveForward(self): # Moves forward a bit
        print('Moving forward!')
        self.cmdvel.linear.x = 0.3
        self.cmdvel.angular.z = 0.0
        self.pub.publish(self.cmdvel)

    def turnRight(self): # Turns right a bit
        print('Turning Right!')
        self.cmdvel.linear.x = 0.0
        self.cmdvel.angular.z = -1.3
        self.pub.publish(self.cmdvel)
        if self.stuckCounter % 2 == 0: # Prevents tb3 getting stuck deciding which direction to turn
            self.stuckCounter += 1
        if self.turning == False:
            self.previousTurn = 'Right'

    def stop(self): # Turns around 180 degrees once maze has been exited
        if self.finished == True:
            return
        else:
            print("Stopping!")
            for i in range(0,160):
                self.cmdvel.linear.x = 0.0
                self.cmdvel.angular.z = 2.0
                self.pub.publish(self.cmdvel)
                time.sleep(0.01)
            self.cmdvel.linear.x = 0.0
            self.cmdvel.angular.z = 0.0
            self.pub.publish(self.cmdvel)
            self.finished = True
            print('Finished Maze!')
            exit()

    def pause(self):
        print("Pausing!")
        self.cmdvel.linear.x = 0.0
        self.cmdvel.angular.z = 0.0
        self.pub.publish(self.cmdvel)  

    def finished_directions(self):
        dirs = [179, 269, 359, 449, 539]

        finished = True

        for dir in dirs:
            finished = finished and self.distances[dir] > self.finishTol

        return finished



def main(args=None):
    rclpy.init(args=args)
    mazesolver = MazeSolver()



    rclpy.spin(mazesolver)
    mazesolver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()