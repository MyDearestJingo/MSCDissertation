#!/home/tuos/miniconda3/bin/python

import sys
import rospy
import numpy as np
from panda_robot import PandaArm
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, Grasp, PlaceLocation
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
        

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = PandaArm()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)

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
        # group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.obj_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        # self.group_names = group_names
        pass
    
    def move_ee_to_cartesian_pose(self, _goal):
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

    
    def pick(self, _capt_pose:list, obj_name:str, finger_dist:float):
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
        grasp_msg.pre_grasp_approach.direction.vector.z = -1.0
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
        grasp_msg.grasp_posture = self.close_gripper(finger_dist)

        # self.move_group.setSupportSurfaceName("ground");

        self.move_group.pick(obj_name, grasp_msg)
        # self.move_group.pick("", grasp_msg)
        pass

    def move_to_grasp_pose(self, _capt_pose:list):
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
        # grasp_msg.post_grasp_retreat.direction.header.frame_id = "panda_link0"
        # grasp_msg.post_grasp_retreat.direction.vector.z = 1.0
        # grasp_msg.post_grasp_retreat.min_distance = 0
        # grasp_msg.post_grasp_retreat.desired_distance = 0

        # open gripper
        grasp_msg.pre_grasp_posture = self.open_gripper()

        # close gripper
        grasp_msg.grasp_posture = self.open_gripper()

        # self.move_group.setSupportSurfaceName("ground");

        # return self.move_group.pick(obj_name, grasp_msg)
        return self.move_group.pick("", grasp_msg)

    def grasp_and_retreat(self, _obj_name:str, _capt_pose:float, _finger_dist:float):
        # define grasp pose
        grasp_msg = Grasp()
        grasp_msg.grasp_pose.header.frame_id = "panda_link0"
        grasp_msg.grasp_pose.pose = list_to_pose(_capt_pose)

        # define approach before performing grasp
        # grasp_msg.pre_grasp_approach.direction.header.frame_id = "panda_link0"
        # grasp_msg.pre_grasp_approach.direction.vector.y = -1.0
        # grasp_msg.pre_grasp_approach.min_distance = 0.095
        # grasp_msg.pre_grasp_approach.desired_distance = 0.115

        # define retreat after performing grasp
        grasp_msg.post_grasp_retreat.direction.header.frame_id = "panda_link0"
        grasp_msg.post_grasp_retreat.direction.vector.z = 1.0
        grasp_msg.post_grasp_retreat.min_distance = 0
        grasp_msg.post_grasp_retreat.desired_distance = 0

        # open gripper
        # grasp_msg.pre_grasp_posture = self.open_gripper()

        # close gripper
        # grasp_msg.grasp_posture = self.open_gripper()
        grasp_msg.grasp_posture = self.close_gripper(_finger_dist)

        # self.move_group.setSupportSurfaceName("ground");

        # return self.move_group.pick(_obj_name, grasp_msg)
        # return self.move_group.pick("", grasp_msg)
        return self.robot.get_gripper().close()


    def place(self, _place_pose:list, obj_name:str):
        place_msg = PlaceLocation()

        place_msg.place_pose.header.frame_id = "panda_link0"
        place_msg.place_pose.pose = list_to_pose(_place_pose)

        place_msg.pre_place_approach.direction.header.frame_id = "panda_link0"
        place_msg.pre_place_approach.direction.vector.z = -1
        place_msg.pre_place_approach.min_distance = 0.1
        place_msg.pre_place_approach.desired_distance = 0.15

        place_msg.post_place_retreat.direction.header.frame_id = "panda_link0"
        place_msg.post_place_retreat.direction.vector.z = 1
        place_msg.post_place_retreat.min_distance = 0.1
        place_msg.post_place_retreat.desired_distance = 0.2

        place_msg.post_place_posture = self.open_gripper()
        # self.move_group.setSupportSurfaceName("ground");
        
        self.move_group.place(obj_name, place_msg)

        pass


    def open_gripper(self):
        posture = JointTrajectory()
        posture.joint_names = ["", ""]
        posture.joint_names[0] = "panda_finger_joint1"
        posture.joint_names[1] = "panda_finger_joint2"

        posture.points = [JointTrajectoryPoint()]
        posture.points[0].positions = [0.04, 0.04]
        posture.points[0].velocities = [0.3, 0.3]
        posture.points[0].effort = [10,10]
        posture.points[0].time_from_start = rospy.Duration(0.5)

        return posture
        

    def close_gripper(self, dist):
        posture = JointTrajectory()
        posture.joint_names = ["", ""]
        posture.joint_names[0] = "panda_finger_joint1"
        posture.joint_names[1] = "panda_finger_joint2"

        posture.points = [JointTrajectoryPoint()]
        posture.points[0].positions = [dist/2, dist/2]
        # posture.points[0].positions = [0.0, 0.0]
        posture.points[0].velocities = [0.1, 0.1]
        posture.points[0].effort = [10,10]


        # posture.points[0].effort = [10,10]

        posture.points[0].time_from_start = rospy.Duration(0.5)

        return posture


def wait_for_state_update(
    scene:moveit_commander.PlanningSceneInterface,
    obj_name:str, box_is_known=False, box_is_attached=False, timeout=4
):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Received
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
    ## or dies before actually publishing the scene update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed.
    ## To avoid waiting for scene updates like this at all, initialize the
    ## planning scene interface with  ``synchronous = True``.
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([obj_name])
        scene.get_
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = obj_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL


def init_scene(obj_name:str, scene:moveit_commander.PlanningSceneInterface, timeout=5):
    co_list = []
    # scene.clear()

    box_size = (0.06,0.158,0.21)
    box_pose = PoseStamped()
    box_pose.header.frame_id = "panda_link0"
    box_pose.pose = list_to_pose([0.6, 0.0, box_size[2]/2 ,0, -0, 0, 1])
    scene.add_box(obj_name, box_pose, size=box_size)
    co_list.append(obj_name)

    ground_pose = PoseStamped()
    ground_pose.header.frame_id = "panda_link0"
    ground_pose.pose = list_to_pose([0, 0, 0, 0, 0, 0, 1])
    scene.add_plane("ground", ground_pose)
    co_list.append("ground")

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        # attached_objects = scene.get_attached_objects([obj_name])
        extended_objects = scene.get_objects(co_list)
        is_added = len(extended_objects.keys()) >= len(co_list)

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        # is_known = obj_name in scene.get_known_object_names()

        # Test if we are in the expected state
        # if (box_is_attached == is_attached) and (box_is_known == is_known):

        if is_added:
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    return False


if __name__  == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

    goal = np.fromstring(
        # "0.59332084 0.07 0.11013612 -0.5 0.5 -0.5 -0.5",
        # "0.59332084  0.09961357  0.31013612 -0.53382679  0.47165199 -0.46815066 -0.52288461",
        # "0.59332084  0.09961357  0.31013612 0 0 0 1",
        "0.6  0  0.29 -7.06825125e-01 7.07388213e-01  2.81656109e-04 -2.81431908e-04", # capture from top
        sep=' ')
    # goal[3:] = (R.from_euler("ZYX", [np.pi/2,0,0])*R.from_quat(goal[3:])).as_quat()
    # goal[3:] = (R.from_euler("ZYX", [0, np.pi/2,0])*R.from_quat(goal[3:])).as_quat()
    # goal[3:] = (R.from_euler("ZYX", [np.pi/4,0,0])*R.from_quat(goal[3:])).as_quat()
    goal[3:] = (R.from_quat(goal[3:]) * R.from_euler("ZYX", [np.pi/4,0,0])).as_quat()
    finger_dist = 0.0717 - 0.02
    # executor.move_ee_to_cartesian_pose(goal)

    # place_pose = np.array([0, 0.5, 0.15, 0, 0, 0,1])
    place_pose = np.array([0, 0.5, 0.12, 0, 0, np.pi/2])

    try:
        executor = Executor()

        obj_name = "cracker_box"
        # executor.scene.removeCollisionObject(obj_name)
        if init_scene(obj_name, executor.scene):
            rospy.loginfo("Scene initialization completed")
            rospy.logdebug("Objects in planning scene now: {}".\
                format(executor.scene.get_known_object_names()))
        else: 
            rospy.logerr("Scene initialization failed")
            exit()

        rospy.loginfo("Start pick-up task")
        executor.pick(goal, obj_name, finger_dist)
        rospy.logdebug("Objects in planning scene now: {}".\
            format(executor.scene.get_known_object_names()))
        rospy.loginfo("Start place task")
        executor.place(place_pose, obj_name)

    finally:
        rospy.sleep(2)
        executor.scene.clear()
    pass