#!/home/tuos/miniconda3/bin/python

from pdb import post_mortem
import sys
import copy
import rospy
import numpy as np
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, Grasp
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list, list_to_pose

from scipy.spatial.transform import Rotation as R
from math import pi, tau, dist, fabs, cos


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class Executor:
    def __init__(self):
        super(Executor, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        pass
    
    def move_ee_to_cartesian_pose(self, _goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        goal_msg = Pose()
        goal_msg.position.x     = _goal[0]
        goal_msg.position.y     = _goal[1]
        goal_msg.position.z     = _goal[2]
        goal_msg.orientation.x  = _goal[3]
        goal_msg.orientation.y  = _goal[4]
        goal_msg.orientation.z  = _goal[5]
        goal_msg.orientation.w  = _goal[6]

        self.move_group.set_pose_target(goal_msg)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(goal_msg, current_pose, 0.01)

    
    def pick(self, _capt_pose:list):
        '''
        Build a moveit_msgs::Grasp msg for capturing object at _capt_pose
        @param _capt_pose: float list, includes capture position and orientation (in quaternion)
        '''
        # define grasp pose
        grasp_msg = Grasp()
        grasp_msg.grasp_pose.header.frame_id = "panda_link0"
        grasp_msg.grasp_pose.pose = list_to_pose(_capt_pose)

        # define approach before performing grasp
        grasp_msg.pre_grasp_approach.direction.header.frame_id = "panda_link0"
        grasp_msg.pre_grasp_approach.direction.vector.y = -1.0
        grasp_msg.pre_grasp_approach.min_distance = 0.095
        grasp_msg.pre_grasp_approach.desired_distance = 0.115

        # define retreat after performing grasp
        grasp_msg.post_grasp_retreat.direction.header.frame_id = "panda_link0"
        grasp_msg.post_grasp_retreat.direction.vector.z = 1.0
        grasp_msg.post_grasp_retreat.min_distance = 0.1
        grasp_msg.post_grasp_retreat.desired_distance = 0.25

        # open gripper
        grasp_msg.pre_grasp_posture = self.open_gripper()

        # close gripper
        grasp_msg.grasp_posture = self.close_gripper()

        self.move_group.pick("", grasp_msg)
        pass


    def open_gripper(self):
        posture = JointTrajectory()
        posture.joint_names = ["", ""]
        posture.joint_names[0] = "panda_finger_joint1"
        posture.joint_names[1] = "panda_finger_joint2"

        posture.points = [JointTrajectoryPoint()]
        posture.points[0].positions = [0.05, 0.05]
        posture.points[0].time_from_start = rospy.Duration(0.5)

        return posture
        

    def close_gripper(self):
        posture = JointTrajectory()
        posture.joint_names = ["", ""]
        posture.joint_names[0] = "panda_finger_joint1"
        posture.joint_names[1] = "panda_finger_joint2"

        posture.points = [JointTrajectoryPoint()]
        posture.points[0].positions = [0.0, 0.0]
        posture.points[0].time_from_start = rospy.Duration(0.5)

        return posture


if __name__  == "__main__":
    executor = Executor()
    goal = np.fromstring(
        # "0.59332084  0.09961357  0.11013612 -0.53382679  0.47165199 -0.46815066 -0.52288461",
        "0.59332084  0.09961357  0.31013612 -0.53382679  0.47165199 -0.46815066 -0.52288461",
        # "0.59332084  0.09961357  0.31013612 0 0 0 1",
        sep=' ')
    # goal[3:] = (R.from_euler("ZYX", [np.pi/2,0,0])*R.from_quat(goal[3:])).as_quat()
    # goal[3:] = (R.from_euler("ZYX", [0, np.pi/2,0])*R.from_quat(goal[3:])).as_quat()
    # goal[3:] = (R.from_euler("ZYX", [np.pi/4,0,0])*R.from_quat(goal[3:])).as_quat()
    goal[3:] = (R.from_quat(goal[3:]) * R.from_euler("ZYX", [np.pi/4,0,0])).as_quat()
    # executor.move_ee_to_cartesian_pose(goal)
    executor.pick(goal)
    pass