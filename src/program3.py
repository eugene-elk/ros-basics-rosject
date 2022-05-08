#! /usr/bin/env python3

import os
import rospy
import actionlib
from wallwalking.srv import FindWall, FindWallRequest
from wallwalking.msg import OdomRecordAction, OdomRecordGoal, OdomRecordResult, OdomRecordFeedback
from wall_follow_control import WallFollowControl

# states of action server
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4


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

wf_control = WallFollowControl()

rate = rospy.Rate(5)

goal = OdomRecordGoal()
client.send_goal(goal, feedback_cb=feedback_callback)

state_result = client.get_state()
while (state_result < DONE) and (not rospy.is_shutdown()):
    wf_control.pd_controller()
    rate.sleep()
    state_result = client.get_state()

wf_control.stop()

rospy.loginfo("[main] State: " + str(state_result))
if state_result == ERROR:
    rospy.logerr("[main] Something went wrong in the Server Side")
if state_result == WARN:
    rospy.logwarn("[main] There is a warning in the Server Side")

# os.system("rosnode kill record_odom_as")
