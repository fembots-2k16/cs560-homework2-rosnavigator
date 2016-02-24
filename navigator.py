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
        self.angle_sensitivity = 15
        self.proximity_sensitivity = 0.7
        self.is_wall_searching = False
        self.is_spinning = 0
        self.is_stuck = 0
        self.was_i_turning_left = False
        self.get_off_the_wall = False
        self.getting_off_the_wall = 0
        self.non_wall_following_turn_direction = "left"
        self.moving_forward = 0

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

        angle = self.pose.pose.orientation.z
        if angle < 0.71 and angle > 0.70:
            self.is_wall_searching = False
            self.non_wall_following_turn_direction = "left"

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
        return self.hasObstacle(180, self.angle_sensitivity, self.proximity_sensitivity)
    def hasRightObstacle(self):
        return self.hasObstacle(0, self.angle_sensitivity, self.proximity_sensitivity)
    def hasRightObstacleManual(self, s):
        return self.hasObstacle(0, self.angle_sensitivity, s)
    def hasFrontRightObstacle(self):
        return self.hasObstacle(45, self.angle_sensitivity, self.proximity_sensitivity)

    def isWallSearching(self):
        return self.is_wall_searching


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
    print "distance traveled: " + str(robbo.getDistanceTraveled())
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
    while not rospy.is_shutdown():
        time += 1
        print "time passed: " + str(time)
        twist = Twist() #values default to 0 when new instance initiated

        #wall following but we got too close to the wall, try backin up a bit
        if robbo.get_off_the_wall:
            robbo.moving_forward = 0
            rospy.loginfo("GETTING OFF THE WALL")
            robbo.getting_off_the_wall += 1
            if robbo.getting_off_the_wall < 10:
                #move backward
                twist.linear.x = -0.5
                #turn left
                twist.angular.z = -0.5
            elif robbo.getting_off_the_wall < 18:
                #turn right
                twist.angular.z = 0.5
            else:
                robbo.get_off_the_wall = False
        #INITIAL STATE, MOVING FORWARD UNTIL OBSTACLE IS ENCOUNTERED
        elif not robbo.is_wall_searching:
            if not robbo.hasFrontObstacle():
                if not robbo.was_i_turning_left:
                    if robbo.is_stuck >= 5:
                        rospy.loginfo("tlnws adjust")
                        twist.angular.z = 1.0
                        twist.linear.x = -0.5
                    else:
                        rospy.loginfo("moving forward, not wall searching")
                        twist.linear.x = 0.5
                else:
                    rospy.loginfo("SWITCHING TO WALL SEARCHING")
                    robbo.is_wall_searching = True
                    robbo.is_spinning = 0
                    robbo.was_i_turning_left = False
            #        robbo.switchTurnDirection()
            else:
                if robbo.non_wall_following_turn_direction == "left":
                    if robbo.is_stuck >= 5 and robbo.is_stuck < 10:
                        rospy.loginfo("tlnws adjust")
                        twist.angular.z = -1.0
                        twist.linear.x = -0.5
                    elif robbo.is_stuck >= 10:
                        rospy.loginfo("tlnws adjust2")
                        twist.linear.x = -0.5
                    else:
                        rospy.loginfo("turning left, not wall searching")
                        twist.angular.z = 1.0
                        twist.linear.x = 0.1
                        robbo.was_i_turning_left = True
                else:
                    rospy.loginfo("turning right, not wall searching")
                    twist.angular.z = -1.0
                    twist.linear.x = 0.1
                    robbo.was_i_turning_left = True

        #WALL FOLLOWING ALGORITHM
        elif robbo.is_wall_searching:
            if not robbo.hasRightObstacle():
                if robbo.is_spinning < 5 or robbo.hasFrontObstacle():
                    if robbo.is_stuck >= 5:
                        robbo.moving_forward = 0
                        rospy.loginfo("adjust self, back up and turn right")
                        twist.linear.x = -0.5
                        twist.angular.z = -0.5
                    else:
                        robbo.moving_forward = 0
                        rospy.loginfo("turning right, WALL SEARCHING")
                        twist.angular.z = -1.0
                        robbo.is_spinning += 1
                else:
                    if robbo.is_stuck >= 5:
                        robbo.moving_forward = 0
                        rospy.loginfo("adjust self, back up and turn right")
                        twist.linear.x = -0.5
                        twist.angular.z = -0.5
                    else:
                        robbo.moving_forward += 1
                        rospy.loginfo("moving forward, WALL SEARCHING, no right")
                        twist.linear.x = 0.5
            elif robbo.hasRightObstacleManual(0.25):
                robbo.get_off_the_wall = True
                robbo.getting_off_the_wall = 0
            elif robbo.hasRightObstacle():
                robbo.is_spinning = 0
                if not robbo.hasFrontObstacle():
                    if robbo.is_stuck >= 5:
                        robbo.moving_forward = 0
                        rospy.loginfo("adjust self, back up and turn left")
                        twist.linear.x = -0.5
                        twist.angular.z = 0.5
                    else:
                        robbo.moving_forward += 1
                        rospy.loginfo("moving forward, WALL SEARCHING, YES RIGHT")
                        twist.linear.x = 0.5
                elif not robbo.hasLeftObstacle():
                    if robbo.is_stuck >= 5:
                        robbo.moving_forward = 0
                        rospy.loginfo("adjust self, back up and turn right")
                        twist.angular.z = -1.0
                        twist.linear.x = -0.5
                    else:
                        robbo.moving_forward = 0
                        rospy.loginfo("turning left I guess!, WALL SEARCHING")
                        twist.angular.z = -1.0
                        twist.linear.x = -0.5
                else:
                    robbo.moving_forward = 0
                    rospy.loginfo("backing up")
                    twist.linear.x = -0.5

            if robbo.moving_forward >= 5:
                robbo.is_wall_searching = False




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
