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
        self.angle_sensitivity = 20
        self.proximity_sensitivity = 0.5
        self.is_wall_searching = False
        self.is_spinning = 0
        self.is_stuck = 0

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
    def hasRightObstacleManual(self, offset, s):
        return self.hasObstacle(0, offset, s)

    def isWallSearching(self):
        return self.is_wall_searching


    def hasObstacle(self, angle, offset, s):
        has_obstacle = False
        i = angle - offset
        while i < angle + offset:
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
    while not rospy.is_shutdown():
        time += 1
        print "time passed: " + str(time)
        twist = Twist() #values default to 0 when new instance initiated
        if robbo.hasFrontObstacle():
            if robbo.is_stuck >= 15 and robbo.is_stuck < 30:
                rospy.loginfo("moving forward")
                print "is_stuck", robbo.is_stuck
                twist.linear.x = 0.5
            elif robbo.is_stuck >= 30:
                rospy.loginfo("backing up?")
                twist.linear.x = -0.5
            else:
                rospy.loginfo("turning left")
                twist.angular.z = 1.0
                robbo.is_wall_searching = True
                robbo.is_spinning = 0

        else:
            if not robbo.hasRightObstacle() and robbo.isWallSearching():
                rospy.loginfo("turning right, is_spinning: %i" %(robbo.is_spinning))
                twist.angular.z = -1.0
                robbo.is_spinning += 1
                if robbo.is_spinning >= 15:
                    robbo.is_wall_searching = False
            else:
                if robbo.hasFrontObstacle() or robbo.is_stuck >= 15:
                    rospy.loginfo("turning left and backing up?")
                    twist.linear.x = -0.5
                    twist.angular.z = 1.0
                else:
                    rospy.loginfo("moving forward")
                    twist.linear.x = 0.5

        #publish the message
        vel_pub.publish(twist)
        rate.sleep()

        dist = robbo.getDistanceTraveled()
        if time == 3000:# or dist >= 10.0:
            if dist >= 10.0:
                print "SUCCESS"
            else: print "FAILURE"
            return

if __name__ == "__main__":
    try:
        navigator()
    except rospy.ROSInterruptException:
        pass
