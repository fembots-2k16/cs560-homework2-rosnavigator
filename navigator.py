#!/usr/bin/env python
# license removed for brevity (lol)
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

###   TODO::
###     pose #1 works
###     pose #2 does not work, it gets stuck in loop in the room on the right.
###         possible solution, make it turn right faster out the door???

class Robort:
    def __init__(self):
        self.pose = None
        self.prev_pose = None
        self.laser = ()
        self.initial_pose = None
        self.angle_sensitivity = 30
        self.proximity_sensitivity = 0.4
        self.is_wall_searching = False
        self.is_spinning = 0
        self.is_stuck = 0
        self.was_i_turning_left = False
        self.get_off_the_wall = False
        self.getting_off_the_wall = 0
        self.non_wall_following_turn_direction = "left"
        self.moving_forward = 0
        self.queue = []
        self.prev_queue = []
        self.angle = 90
        self.looking_for_right = False

    def setPose(self, pose):
        if self.initial_pose == None:
            self.initial_pose = pose

        if (self.prev_pose != None):
            dist = self.getDistanceTraveledDelta()
            if dist < 0.001:
                self.is_stuck += 1
            else: self.is_stuck = 0

        self.prev_pose = self.pose
        self.pose = pose

    def switchTurnDirection(self):
        if self.non_wall_following_turn_direction == "left":
            self.non_wall_following_turn_direction = "right"
        else:
            self.non_wall_following_turn_direction = "left"

    def getDistanceTraveled(self):
        x1 = self.initial_pose.pose.position.x
        y1 = self.initial_pose.pose.position.y

        x2 = self.pose.pose.position.x
        y2 = self.pose.pose.position.y

        return self.dist(x1, y1, x2, y2)

    def getDistanceTraveledDelta(self):
        x1 = self.prev_pose.pose.position.x
        y1 = self.prev_pose.pose.position.y

        x2 = self.pose.pose.position.x
        y2 = self.pose.pose.position.y

        #print "x1:",x1,",y1:",y1,",x2:",x2,",y2:",y2

        return self.dist(x1, y1, x2, y2)

    def dist(self, x1, y1, x2, y2):
        return math.sqrt(math.pow(x2-x1, 2) + math.pow(y2-y1, 2))

    def hasFrontObstacle(self):
        return self.hasObstacle(90, self.angle_sensitivity, self.proximity_sensitivity)
    def hasLeftObstacle(self):
        return self.hasObstacle(180, 45, self.proximity_sensitivity)
    def hasRightObstacle(self):
        return self.hasObstacle(0, 45, self.proximity_sensitivity)
    def hasObstacle(self, angle, offset, s):
        has_obstacle = False
        i = angle - offset
        while i < angle + offset:
            if i < 0:
                i += 1
                continue
            if i >= len(self.laser.ranges): break
            has_obstacle = has_obstacle or (self.laser.ranges[i] < s)
            i += 1
        return has_obstacle

    def printQueue(self):
        if len(self.queue) != len(self.prev_queue):
            print self.queue
        self.prev_queue = self.queue[0:]

    def orientationToAngle(self, orient):
        return 2*math.degrees(math.asin(orient))

    def angleToOrientation(self, angle):
        angle = angle % 360
        return math.sin(math.radians(angle/2))

    def turn(self, amt, dir):
        print "turn,",amt,self.angle,self.angleToOrientation(self.angle+amt)
        return ["turn", False, self.angleToOrientation(self.angle+amt), dir]
    def leftTurn(self):
        return self.turn(90, 1)
    def rightTurn(self):
        return self.turn(-90, -1)

    def moveForward(self, dist):
        return ["move", False, dist, self.pose.pose.position]

    def handleQueue(self, twist):
        item = self.queue[0]
        command = item[0]

        #print "command:",command
        if command == "turn":
            dir = self.queue[0][3]
            twist.angular.z = dir
            curr_angle = self.orientationToAngle(self.pose.pose.orientation.z)
            desired_angle = self.orientationToAngle(self.queue[0][2])
            print curr_angle, desired_angle
            diff = 0
            if dir > 1 and diff > 0:
                twist.angular.z = -self.angleToOrientation(diff)
                self.queue[0][1] = True
            if dir < 1 and diff < 0:
                twist.angular.z = -self.angleToOrientation(diff)
                self.queue[0][1] = True
        if command == "move":
            twist.linear.x = 0.25
            x1 = self.pose.pose.position.x
            y1 = self.pose.pose.position.y
            x2 = item[3].x
            y2 = item[3].y
            dist = self.dist(x1, y1, x2, y2)
            if dist < 0.25:
                twist.linear.x = dist
                self.queue[0][1] = True
            if item[2] > 0 and self.hasFrontObstacle():
                twist.linear.x = 0
                self.queue[0][1] = True

        if self.queue[0][1]:
            self.queue = self.queue[1:]
        self.printQueue()
        return twist


    def navigate(self, twist):
        #print self.pose.pose.orientation.z
        #return twist
        if len(self.queue) > 0:
            return self.handleQueue(twist)

        if not self.is_wall_searching:
            if not self.hasFrontObstacle():
                twist.linear.x = 0.5
            else:
                self.queue.append(self.leftTurn())
                self.is_wall_searching = True

        #WALL SEARCHING
        else:
            if self.hasRightObstacle():
                self.looking_for_right = False
                if not self.hasFrontObstacle():
                    twist.linear.x = 0.25
                else:
                    print "QUEUE LEFT TURN"
                    self.queue.append(self.leftTurn())
            else:
                if self.looking_for_right:
                    if not self.hasFrontObstacle():
                        twist.linear.x = 0.25
                    else:
                        print "QUEUE LEFT TURN"
                        self.queue.append(self.leftTurn())
                        if self.is_stuck < 30:
                            self.looking_for_right = False
                else:
                    print "QUEUE FORWARD, RIGHT TURN"
                    self.queue.append(self.moveForward(0.5))
                    self.queue.append(self.rightTurn())
                    self.looking_for_right = True



        if len(self.queue) > 0:
            return self.handleQueue(twist)
        else: return twist

robbo = Robort()

def processSonar(sonar_msg):
    #print("sonar:", len(sonar_msg.ranges))
    pass

def processLaser(laser_msg):
    global robbo
    robbo.laser = laser_msg

def processPose(pose_msg):
    global robbo
    pose = pose_msg.pose
    robbo.setPose(pose)
    #print "distance traveled: " + str(robbo.getDistanceTraveled())
    #rospy.loginfo("linear: x=%0.2f, y=%0.2f, z=%0.2f" %(
    #    pose.position.x, pose.position.y, pose.position.z
    #))
    #rospy.loginfo("angular: x=%0.2f, y=%0.2f, z=%0.2f, w=%0.2f" %(
    #    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    #))

def navigator():
    global robbo
    ns = "r0"
    pose_sub = rospy.Subscriber('/odom', Odometry, processPose)
    sonar_sub = rospy.Subscriber('/base_scan_0', LaserScan, processSonar)
    laser_sub = rospy.Subscriber('/base_scan_1', LaserScan, processLaser)

    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    time = 0

    rospy.init_node('navigator')
    rate = rospy.Rate(10) # 10hz
    max_dist = -999
    print "begin!"
    while not rospy.is_shutdown():
        time += 1
        #print "time passed: " + str(time)
        twist = Twist() #values default to 0 when new instance initiated

        twist = robbo.navigate(twist)

        #publish the message
        vel_pub.publish(twist)
        rate.sleep()

        dist = robbo.getDistanceTraveled()
        if dist > max_dist:
            max_dist = dist
        if time == 3000:# or dist >= 10.0:
            print "max_dist", max_dist
            if max_dist >= 10.0:
                print "SUCCESS"
            else: print "FAILURE"
            return

if __name__ == "__main__":
    try:
        navigator()
    except rospy.ROSInterruptException:
        pass
