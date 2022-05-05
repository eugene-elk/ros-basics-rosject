#! /usr/bin/env python3

import os
import time
import rospy
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wallwalking.srv import FindWall, FindWallRequest
from wallwalking.msg import OdomRecordAction, OdomRecordGoal, OdomRecordResult, OdomRecordFeedback

# states of action server
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

lidar_right = float()
lidar_front = float()

front_distance_reaction = 0.55
perfect_wall_distance = 0.32


def rele_controller():
    move.linear.x = 0.05
    move.angular.z = 0
    if lidar_right > 0.4:
        move.angular.z = -0.09
    elif lidar_right > 0.3:
        move.angular.z = 0
    else:
        move.angular.z = 0.09

    if lidar_front < front_distance_reaction:
        move.angular.z = 0.20


def p_controller():
    move.linear.x = 0.05

    error = perfect_wall_distance - lidar_right
    koeff_p = 1.0
    move.angular.z = koeff_p * error

    if lidar_front < front_distance_reaction:
        move.angular.z = 0.20




def scan_callback(msg):
    global lidar_right, lidar_front
    lidar_right = msg.ranges[270]
    lidar_front = msg.ranges[360]
    #rospy.loginfo("[scan_callback] Front: " + str(front) + ", Right: " + str(right))


def feedback_callback(feedback):
    distance = feedback.current_total
    rospy.loginfo("Distance: " + str(distance))


rospy.init_node("wall_walking_node")
rospy.wait_for_service('/find_wall')

find_wall_service = rospy.ServiceProxy('/find_wall', FindWall)
find_wall_request = FindWallRequest()
find_wall_result = find_wall_service(find_wall_request)
print(find_wall_result)

os.system("rosnode kill find_wall_node")

client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)
rospy.loginfo('[main] Waiting for Action Server')
client.wait_for_server()
rospy.loginfo('[main] Action Server Found')

subScan = rospy.Subscriber('/scan', LaserScan, scan_callback)
while subScan.get_num_connections() < 1:
    rospy.loginfo("[main] Waiting for subsccription to /scan")
    time.sleep(0.1)

pubCmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
while pubCmdVel.get_num_connections() < 1:
    rospy.loginfo("[main] Waiting for connection to /cmd_vel")
    time.sleep(0.1)

rate = rospy.Rate(1)
move = Twist()
move.linear.x = 0
move.angular.z = 0
pubCmdVel.publish(move)

goal = OdomRecordGoal()
client.send_goal(goal, feedback_cb=feedback_callback)
state_result = client.get_state()

while state_result < DONE:
    # rospy.loginfo("[main] Move along the wall")

    p_controller()

    pubCmdVel.publish(move)
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo("[main] state_result: " + str(state_result))

move.linear.x = 0
move.angular.z = 0
pubCmdVel.publish(move)

rospy.loginfo("[main] State: " + str(state_result))
if state_result == ERROR:
    rospy.logerr("[main] Something went wrong in the Server Side")
if state_result == WARN:
    rospy.logwarn("[main] There is a warning in the Server Side")

# os.system("rosnode kill record_odom_as")
