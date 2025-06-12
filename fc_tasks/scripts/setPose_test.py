import rospy 
from fc_msgs.srv import SetPose
from geometry_msgs.msg import Pose 

targetPose = Pose()
targetPose.position.x = 0.5
targetPose.position.y = 0.0
targetPose.position.z = 1.0 
targetPose.orientation.w = 0.0
targetPose.orientation.x = 0.707
targetPose.orientation.y = 0.0 
targetPose.orientation.z = 0.707

if __name__ == '__main__':
    rospy.init_node('test_fc_set_pose')

    rospy.wait_for_service('/sim1/fc_set_pose')
    try:
        set_pose = rospy.ServiceProxy('/sim1/fc_set_pose', SetPose)
        response = set_pose(targetPose, 'sim1/base_link', 0.1, 0.1, 'LIN')
        rospy.loginfo("Response Pose:\n%s", response.pose)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))