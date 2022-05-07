#! /usr/bin/env python3

import time
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wallwalking.srv import FindWall, FindWallResponse


class FindWallService():

    move = Twist()
    value_front = int()
    minimum_position = int()

    def __init__(self):

        self.subScan = rospy.Subscriber('/scan', LaserScan, self.callback_scan)
        while self.subScan.get_num_connections() < 1:
            rospy.loginfo("Waiting for subsccription to /scan")
            time.sleep(0.1)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        while self.pub.get_num_connections() < 1:
            rospy.loginfo("Waiting for connection to /cmd_vel")
            time.sleep(0.1)

        self.srv = rospy.Service('/find_wall', FindWall, self.callback_srv)
        self.rate = rospy.Rate(10)

    def callback_scan(self, msg):

        self.lidar_values = msg.ranges
        # print(self.lidar_values[360])

    def callback_srv(self, request):

        current_lidar_values = self.lidar_values
        cos_30_degree = math.cos(math.radians(30))
        rospy.loginfo("cos 30 degree: " + str(cos_30_degree))

        minimal_wall_distance = float('inf')
        wall_position = int()
        at_least_one_wall = False

        for angle in range(0, 720, 2):

            dist_angle = current_lidar_values[angle]
            dist_angle_plus = current_lidar_values[(angle + 60) % 720]
            dist_angle_minus = current_lidar_values[angle - 60]

            if math.isinf(dist_angle) or math.isinf(dist_angle_plus) or math.isinf(dist_angle_minus):
                pass

            else:
                cos_alpha = dist_angle / dist_angle_plus
                cos_beta = dist_angle / dist_angle_minus

                # rospy.loginfo("cos 30 d: " + str(cos_30_degree))
                # rospy.loginfo("cos alpha: " + str(cos_alpha))
                # rospy.loginfo("cos beta: " + str(cos_beta))

                if (abs(cos_alpha - cos_30_degree) < 0.05) and (abs(cos_beta - cos_30_degree) < 0.05):
                    at_least_one_wall = True
                    if dist_angle < minimal_wall_distance:
                        minimal_wall_distance = dist_angle
                        wall_position = angle
                        rospy.loginfo("new wall position: "+str(wall_position))

        if at_least_one_wall:
            wall_position /= 2
            rospy.loginfo("----------------------")
            rospy.loginfo("wall_position: " + str(wall_position))
            rospy.loginfo("wall_distance: " + str(minimal_wall_distance))

            self.rotate_to_wall(wall_position)
            self.move_to_wall()

        else:
            rospy.loginfo("didn't find any straight walls")

        result = FindWallResponse()
        result.wallfound = at_least_one_wall
        rospy.loginfo("[srv] Service Server Finished")
        return result

    def rotate_to_wall(self, wall_position):

        destination = wall_position - 180

        rospy.loginfo("destination: " + str(destination))
        self.move.angular.z = 0.1 * math.copysign(1, destination)
        self.pub.publish(self.move)

        rospy.sleep(abs(destination) * 0.18)
        self.move.angular.z = 0
        self.pub.publish(self.move)
        rospy.loginfo("finish rotation")

    def move_to_wall(self):

        rospy.loginfo("move forward")
        self.move.linear.x = 0.05
        self.move.angular.z = 0
        self.pub.publish(self.move)

        while self.lidar_values[360] > 0.35:
            rospy.sleep(0.5)

        rospy.loginfo("rotate")
        self.move.linear.x = 0
        self.move.angular.z = 0.1
        self.pub.publish(self.move)
        rospy.sleep(90 * 0.18)

        rospy.loginfo("stop")
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.pub.publish(self.move)


if __name__ == '__main__':
    rospy.init_node('find_real_wall_node')
    FindWallService()
    rospy.spin()
