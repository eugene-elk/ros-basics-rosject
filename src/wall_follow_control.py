import time
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollowControl():

    front_distance_reaction = 0.55
    perfect_wall_distance = 0.32
    koeff_p = 1.0
    koeff_d = 0.5

    def __init__(self):

        self.lidar_front = 0.6
        self.lidar_right = 0.32

        self.error_old = 0
        self.move = Twist()

        self.subScan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        while self.subScan.get_num_connections() < 1:
            rospy.loginfo("[main] Waiting for subscription to /scan")
            time.sleep(0.1)

        self.pubCmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        while self.pubCmdVel.get_num_connections() < 1:
            rospy.loginfo("[main] Waiting for connection to /cmd_vel")
            time.sleep(0.1)

        self.move.linear.x = 0
        self.move.angular.z = 0
        self.pubCmdVel.publish(self.move)

    def scan_callback(self, msg):
        self.lidar_right = msg.ranges[270]
        self.lidar_front = msg.ranges[360]
        rospy.loginfo("[scan_callback] Front: " +
                      str(self.lidar_front) + ", Right: " + str(self.lidar_right))

    def rele_controller(self):

        self.move.linear.x = 0.05
        self.move.angular.z = 0
        if self.lidar_right > 0.4:
            self.move.angular.z = -0.09
        elif self.lidar_right > 0.3:
            self.move.angular.z = 0
        else:
            self.move.angular.z = 0.09

        if self.lidar_front < self.front_distance_reaction:
            self.move.angular.z = 0.20

        self.pubCmdVel.publish(self.move)

    def p_controller(self):

        self.move.linear.x = 0.05

        error = self.perfect_wall_distance - self.lidar_right
        turn_value = self.koeff_p * error
        self.move.angular.z = turn_value
        rospy.loginfo("P: " + str(turn_value))

        if self.lidar_front < self.front_distance_reaction:
            self.move.angular.z = 0.20

        self.pubCmdVel.publish(self.move)

    def pd_controller(self):

        self.move.linear.x = 0.05

        error = self.perfect_wall_distance - self.lidar_right
        value_p = self.koeff_p * error
        value_d = self.koeff_d * (error - self.error_old)
        turn_value = value_p + value_d
        self.move.angular.z = turn_value
        rospy.loginfo("P: " + str(value_p) + ", D: " + str(value_d))

        if self.lidar_front < self.front_distance_reaction:
            self.move.angular.z = 0.20

        self.error_old = error

        self.pubCmdVel.publish(self.move)

    def stop(self):
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.pubCmdVel.publish(self.move)
