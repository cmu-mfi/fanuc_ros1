import rospy 
from fc_msgs.srv import SetPose
from geometry_msgs.msg import Pose 

targetPose = Pose()
targetPose.position.x = 0.55
targetPose.position.y = 0.0
targetPose.position.z = 0.805
targetPose.orientation.w = 0.0
targetPose.orientation.x = 0.707
targetPose.orientation.y = 0.0 
targetPose.orientation.z = 0.707

if __name__ == '__main__':
    rospy.init_node('test_fc_set_pose')

    rospy.wait_for_service('/real/fc_set_pose')
    try:
        set_pose = rospy.ServiceProxy('/real/fc_set_pose', SetPose)
        response = set_pose(targetPose, '/real/base_link', 0.1, 0.1, 'PTP')
        rospy.loginfo("Response Pose:\n%s", response.pose)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))