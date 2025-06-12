#!/usr/bin/env python

import rospy
import actionlib
from fc_msgs.msg import GoToPoseAction, GoToPoseGoal
from geometry_msgs.msg import Pose 

goal = Pose()
goal.position.x = 0.6
goal.position.y = 0.0
goal.position.z = 1.0 
goal.orientation.w = 0.0
goal.orientation.x = 0.707
goal.orientation.y = 0.0 
goal.orientation.z = 0.707

goalSend = GoToPoseGoal()
goalSend.pose = goal
goalSend.base_frame = "base_link"
goalSend.max_velocity_scaling_factor = 0.5
goalSend.max_acceleration_scaling_factor = 0.5
goalSend.traj_type = "PTP"

if __name__ == '__main__':
    rospy.init_node('test_fc_go_to_pose')

    client = actionlib.SimpleActionClient('/sim1/fc_go_to_pose', GoToPoseAction)

    rospy.loginfo("Waiting for action server /sim1/fc_go_to_pose...")
    client.wait_for_server()
    rospy.loginfo("Server is up. Sending goal...")

    client.send_goal(goalSend)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo("Action result:\n%s", result)
