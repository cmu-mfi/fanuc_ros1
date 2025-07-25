import rospy
import ezdxf 
import tf 
import math 
import numpy as np

from geometry_msgs.msg import Pose 
from fc_msgs.srv import ExecuteCartesianTrajectory


DXF_FILE_PATH = "/root/ros1_ws/src/fanuc_ros1/fc_tasks/scripts/dxf_test.dxf"
FIXED_Z = 0.
FIXED_QUAT = (-1., 0., 0., 0.)

def transform_to_centre(center_x, center_y, poses) -> list:
    workspace_centre_x = 0.5
    workspace_centre_y = 0.5

    delta_x = center_x - workspace_centre_x
    delta_y = center_y - workspace_centre_y

    transform_matrix = np.array([1, 0, 0, -delta_x], [0, 1, 0, -delta_y], [0, 0, 1, 0], [0, 0, 0, 1])
    
    for pose in poses: 
        point = [pose.position.x, pose.position.y, pose.position.z, 1]
        new_point = np.dot(transform_matrix, point)
        pose.position.x = new_point[0]
        pose.position.y = new_point[1]
        pose.position.z = new_point[2]

    return poses

def parse_dxf_to_poses(dxf_file) -> list:
    doc = ezdxf.readfile(dxf_file)
    msp = doc.modelspace()

    poses = []

    for line in msp.query("LINE"):
        for pt in [line.dxf.start, line.dxf.end]:
            pose = Pose()
            pose.position.x = pt[0]
            pose.position.y = pt[1]
            pose.position.z = FIXED_Z
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = FIXED_QUAT
            poses.append(pose)
    
    for polyline in msp.query("POLYLINE"):
        for vertex in polyline.vertices:
            x, y, z = vertex.dxf.location.x, vertex.dxf.location.y, FIXED_Z
            pose = Pose()                                                                       
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = FIXED_QUAT
            poses.append(pose)
    
    bbox = msp.get_extents()
    center_x = (bbox.min.x + bbox.max.x) / 2
    center_y = (bbox.min.y + bbox.max.y) / 2

    poses = transform_to_centre(center_x, center_y, poses)    

    return poses 

if __name__ == '__main__':
    poses = parse_dxf_to_poses(DXF_FILE_PATH)
    for pose in poses: 
        print(pose, "\n")

    rospy.init_node('dxf_trajectory')
    rospy.wait_for_service('/sim1/fc_execute_cartesian_trajectory')
    try:
        set_pose = rospy.ServiceProxy('/sim1/fc_execute_cartesian_trajectory', ExecuteCartesianTrajectory)
        response = set_pose(poses, 0.01, 0.0, 0.1, 0.1, 0.0)
        rospy.loginfo("Response Pose:\n%s", response.pose)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
