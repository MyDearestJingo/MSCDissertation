#!/home/tuos/miniconda3/bin/python
'''
Description: 
Author: Yi Lan (ylan12@sheffield.ac.uk)
Date: 2022-08-24 07:33:13
LastEditTime: 2022-08-24 08:24:49
LastEditors: Yi Lan (ylan12@sheffield.ac.uk)
'''
import numpy as np
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import MoveGroupActionResult
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from panda_robot import PandaArm
from executor_moveit import Executor
from planner import Planner
from tf import transformations
from moveit_commander.conversions import pose_to_list, list_to_pose

if __name__ == "__main__":
    node_name = "capture_core"
    obj_name = "cracker"
    scene = PlanningSceneInterface()
    
    rospy.init_node(node_name, log_level=rospy.DEBUG)

    # for debug
    puber_camera_pose = rospy.Publisher("/capture_core/debug_camera_pose", PoseStamped, queue_size=5)


    try:

        camera_pose = np.fromstring("1.5 0 0.05 1.57 -0 -1.57", sep=' ')
        camera_pose = np.concatenate(
            (camera_pose[:3],
            transformations.quaternion_from_euler(camera_pose[3], camera_pose[4], camera_pose[5], "rzyx")))

        planner = Planner(node_name, obj_name, camera_pose)
        dist_palm_finger = 0.08
        capt_palm_pose = np.array([0, -(planner.obj_dim[1]/2 + dist_palm_finger), 0, 0, 0, -np.pi/2]) # grasp from top
        capt_palm_pose = np.concatenate(
            (capt_palm_pose[:3],
            transformations.quaternion_from_euler(capt_palm_pose[3], capt_palm_pose[4], capt_palm_pose[5], "rzyx")))
        rospy.logdebug("capture palm pose: {}".format(capt_palm_pose))


        # executor = Executor()

        ## cracker box pose (in camrea frame) and dimension pre-definition
        # cracker_dim = np.fromstring("0.158 0.21 0.06", sep=' ')
        # cracker_pose = [0.01, -0.06, 0.91, 0.0, 1.0, 0.0, 0.0]

        # palm pose (in object frame) for capture 


        while True:
            # --- DEBUG: check camera pose in RViz --- # 
            cam_pose_msg = PoseStamped()
            cam_pose_msg.header.frame_id = "panda_link0"
            cam_pose_msg.pose = list_to_pose(camera_pose)
            puber_camera_pose.publish(cam_pose_msg)

            # pose = planner.calc_capture_pose_tftree(
            #     camera_pose, capt_palm_pose, eul_order="ZYX")
            pose = planner.calc_capture_pose_moveit(capt_palm_pose)
            rospy.logdebug("capture pose in moveit ee frame: {}".format(pose))
            rospy.sleep(0.5)
        pass
    finally:
        scene.clear()
        rospy.loginfo("MoveIt scene clear")
        pass