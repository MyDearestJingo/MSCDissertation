#!/home/tuos/miniconda3/bin/python
import numpy as np
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from moveit_msgs.msg import MoveGroupActionResult
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from panda_robot import PandaArm
from executor_moveit import Executor
from planner import Planner

if __name__ == "__main__":
    try:
        node_name = "capture_core"
        obj_name = "cracker"
        tf_rot_base_obj = R.from_euler('ZYX', [-np.pi/2, 0, -np.pi/2]).as_quat()
        
        rospy.init_node(node_name, log_level=rospy.DEBUG)

        planner = Planner(node_name, obj_name, tf_rot_base_obj)
        # executor = Executor()

        ## cracker box pose (in camrea frame) and dimension pre-definition
        # cracker_dim = np.fromstring("0.158 0.21 0.06", sep=' ')
        # cracker_pose = [0.01, -0.06, 0.91, 0.0, 1.0, 0.0, 0.0]

        # palm pose (in object frame) for capture 
        dist_palm_finger = 0.08
        capt_palm_pose = np.array([0, -(planner.obj_dim[1]/2 + dist_palm_finger), 0, 0, 0, -np.pi/2]) # grasp from top
        
        camera_pose = np.fromstring("1.5 0 0.05 1.57 -0 -1.57", sep=' ')

        while True:
            # pose = planner.calc_capture_pose_tftree(
            #     camera_pose, capt_palm_pose, eul_order="ZYX")
            pose = planner.calc_capture_pose_moveit(
                camera_pose, capt_palm_pose, eul_order="ZYX")
            rospy.logdebug("capture pose in moveit ee frame: {}".format(pose))
            rospy.sleep(0.5)
        pass
    finally:
        pass