import rospy
import actionlib
from fc_msgs.msg import ExecuteCartesianTrajectoryAction, ExecuteCartesianTrajectoryGoal
from geometry_msgs.msg import Pose 

traj = []
goal = Pose()
goal.position.x = 0.6
goal.position.y = 0.0
goal.position.z = 1.0 
goal.orientation.w = 0.0
goal.orientation.x = 0.707
goal.orientation.y = 0.0 
goal.orientation.z = 0.707
traj.append(goal)
goal.position.x = 0.5
goal.position.y = 0.0
goal.position.z = 1.0 
goal.orientation.w = 0.0
goal.orientation.x = 0.707
goal.orientation.y = 0.0 
goal.orientation.z = 0.707
traj.append(goal)

goalSend = ExecuteCartesianTrajectoryGoal()
goalSend.waypoints = traj
goalSend.eef_step = 0.01
goalSend.jump_threshold = 0.0
goalSend.max_velocity_scaling_factor = 0.3
goalSend.max_acceleration_scaling_factor = 0.3
goalSend.blend_radius = 0.0

if __name__ == '__main__':
    rospy.init_node('test_fc_execute_cartesian_trajectory')

    client = actionlib.SimpleActionClient('/sim1/fc_execute_cartesian_trajectory', ExecuteCartesianTrajectoryAction)

    rospy.loginfo("Waiting for action server /sim1/fc_execute_cartesian_trajectory...")
    client.wait_for_server()
    rospy.loginfo("Server is up. Sending goal...")

    client.send_goal(goalSend)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo("Action result:\n%s", result)
