import rospy 
from fc_msgs.srv import ExecuteCartesianTrajectory
from geometry_msgs.msg import Pose 

traj = []
targetPose = Pose()
targetPose.position.x = 0.5
targetPose.position.y = 0.0
targetPose.position.z = 1.0 
targetPose.orientation.w = 0.0
targetPose.orientation.x = 0.707
targetPose.orientation.y = 0.0 
targetPose.orientation.z = 0.707
traj.append(targetPose)

targetPose.position.x = 0.6
targetPose.position.y = 0.0
targetPose.position.z = 1.0
targetPose.orientation.w = 0.0
targetPose.orientation.x = 0.707
targetPose.orientation.y = 0.0 
targetPose.orientation.z = 0.707
traj.append(targetPose)


if __name__ == '__main__':
    rospy.init_node('test_fc_cart_traj_test')

    rospy.wait_for_service('/sim1/fc_execute_cartesian_trajectory')
    try:
        set_pose = rospy.ServiceProxy('/sim1/fc_execute_cartesian_trajectory', ExecuteCartesianTrajectory)
        response = set_pose(traj, 0.01, 0.0, 0.3, 0.3, 0.0)
        rospy.loginfo("Response Pose:\n%s", response.pose)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))