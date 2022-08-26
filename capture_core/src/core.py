#!/home/tuos/miniconda3/bin/python
'''
Description: 
Author: Yi Lan (ylan12@sheffield.ac.uk)
Date: 2022-08-24 07:33:13
LastEditTime: 2022-08-24 18:20:20
LastEditors: Yi Lan (ylan12@sheffield.ac.uk)
'''
import queue
import numpy as np
import rospy
import sys
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import moveit_commander
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

    # --- camera definition --- #
    camera_pose = np.fromstring("1.5 0 0.05 1.57 -0 -1.57", sep=' ')
    camera_pose = np.concatenate(
        (camera_pose[:3],
        transformations.quaternion_from_euler(camera_pose[3], camera_pose[4], camera_pose[5], "rzyx")))


    # --- task definition --- #    
    obj_name = "cracker"
    obj_dim = np.array([16.403600692749023,21.343700408935547,7.179999828338623])/100

    # pick up pose w.r.t object frame
    dist_palm_finger = 0.08
    # capt_palm_pose = np.array([0, -(obj_dim[1]/2 + dist_palm_finger), 0, 0, 0, -np.pi/2]) # grasp from top
    capt_palm_pose = np.array([-obj_dim[0]/2-dist_palm_finger, -0.05, 0, -np.pi/2, 0, -np.pi/2]) # grasp from right
    capt_palm_pose = np.concatenate(
        (capt_palm_pose[:3],
        transformations.quaternion_from_euler(capt_palm_pose[3], capt_palm_pose[4], capt_palm_pose[5], "rzyx")))
    rospy.logdebug("capture palm pose: {}".format(capt_palm_pose))
    
    # place pose w.r.t world frame
    place_pose = np.array([0, 0.5, obj_dim[1]/2+0.01, 0, -np.pi/2, 0]) # rxyz
    rot_b_o = R.from_euler('ZYX', [-np.pi/2, 0, -np.pi/2])
    ori_b = (rot_b_o * R.from_euler('XYZ', place_pose[3:])).as_quat()
    place_pose = np.concatenate((place_pose[:3], ori_b))

    place_pose_msg = PoseStamped()
    place_pose_msg.header.frame_id = "panda_link0"
    place_pose_msg.pose = list_to_pose(place_pose)


    # --- ROS related initialization --- #
    rospy.init_node(node_name, anonymous=True, log_level=rospy.DEBUG)
    puber_camera_pose = rospy.Publisher(
        "/{_node_name}/debug_camera_pose".format(_node_name=node_name), 
        PoseStamped, queue_size=5)

    puber_place_pose = rospy.Publisher(
        "/{_node_name}/{_obj_name}_place_pose".format(_node_name=node_name, _obj_name=obj_name),
        PoseStamped, queue_size=5)
    puber_place_pose.publish(place_pose_msg)

    moveit_commander.roscpp_initialize(sys.argv)
    scene = PlanningSceneInterface()
    

    ground_pose = PoseStamped()
    ground_pose.header.frame_id = "panda_link0"
    ground_pose.pose = list_to_pose([0, 0, -0.1, 0, 0, 0, 1])
    scene.add_box("ground", ground_pose, size=(10,10,0.2))
    
    # ground_pose = PoseStamped()
    # ground_pose.header.frame_id = "panda_link0"
    # ground_pose.pose = list_to_pose([0, 0, 0, 0, 0, 0, 1])
    # scene.add_plane("ground", ground_pose)


    try:
        planner = Planner(node_name, obj_name, camera_pose, _obj_dim=obj_dim)
        executor = Executor()


        ''' Grasp a static box'''
        # goal = planner.calc_capture_pose_moveit(capt_palm_pose)
        # rospy.logdebug("capture pose in moveit ee frame: {}".format(goal))
        # rospy.logdebug("Objects in planning scene now: {}".\
        #     format(executor.scene.get_known_object_names()))
        # rospy.loginfo("Start pick-up task")
        # executor.pick(goal, obj_name, obj_dim[2]-0.03)
        
        # rospy.loginfo("Start place task")
        # executor.place(place_pose, obj_name)

        ''' Wait a moving box'''
        while not rospy.is_shutdown():
            pose_pred = planner.predict_trajectory(_update_scene=True)
            goal = planner.calc_capture_pose_moveit(capt_palm_pose, _obj_pose=pose_pred)
            rospy.sleep(0.1)
            # executor.move_ee_to_cartesian_pose(goal)

        input("Press Enter to continue...")
        pass

    finally:
        scene.clear()
        rospy.loginfo("MoveIt scene clear")
        pass