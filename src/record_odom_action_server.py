#! /usr/bin/env python

import time
import math
import rospy
import actionlib
from geometry_msgs.msg import Point
from wallwalking.msg import OdomRecordFeedback, OdomRecordResult, OdomRecordAction
from nav_msgs.msg import Odometry


class RecordOdomAction():

    _feedback = OdomRecordFeedback()
    _result = OdomRecordResult()
    one_lap_dist = 5.5

    def __init__(self):
        self.subOdom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        while self.subOdom.get_num_connections() < 1:
            rospy.loginfo("[odom_as] Waiting for subsccription to /odom")
            time.sleep(0.1)

        self._as = actionlib.SimpleActionServer(
            "record_odom", OdomRecordAction, self.goal_callback, False)
        self._as.start()
        self.rate = rospy.Rate(1)

    def odom_callback(self, msg):
        self.odom_x = round(msg.pose.pose.position.x, 2)
        self.odom_y = round(msg.pose.pose.position.y, 2)
        self.odom_theta = round(msg.pose.pose.orientation.z, 2)
        # rospy.loginfo("[odom_as] Odom_x: " + str(self.odom_x) + " Odom_y: " + str(self.odom_y) + " Odom_theta: " + str(self.odom_theta))

    def goal_callback(self, goal):

        success = True
        distance_travelled = 0.0

        last_step = 0.0
        iteration = 0

        odom_readings = Point()
        odom_readings.x = []
        odom_readings.y = []
        odom_readings.z = []

        new_odom = Point()
        result_odom = list()

        while distance_travelled < self.one_lap_dist:

            if self._as.is_preempt_requested():
                rospy.loginfo('[odom_as] The goal has been cancelled')
                self._as.set_preempted()
                success = False
                break

            odom_readings.x.append(self.odom_x)
            odom_readings.y.append(self.odom_y)
            odom_readings.z.append(self.odom_theta)

            new_odom.x = self.odom_x
            new_odom.y = self.odom_y
            new_odom.z = self.odom_theta
            result_odom.append(new_odom)

            if iteration > 0:
                last_step = math.sqrt((math.pow(odom_readings.x[iteration] - odom_readings.x[iteration - 1], 2)) + (
                    math.pow(odom_readings.y[iteration] - odom_readings.y[iteration - 1], 2)))

            iteration += 1
            distance_travelled += last_step

            self._feedback.current_total = round(distance_travelled, 2)
            self._as.publish_feedback(self._feedback)
            self.rate.sleep()

        if success:
            self._result.list_of_odoms = result_odom
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('record_odom_as')
    RecordOdomAction()
    rospy.spin()
