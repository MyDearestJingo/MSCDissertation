#!/home/tuos/miniconda3/bin/python

import numpy as np
import quaternion as quat
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from panda_robot import PandaArm
from scipy.spatial.transform import Rotation as R

class Executor:
    def __init__(self):
        # --- params initialization --- #

        # Reserve for reading config file
        # ...

        node_name = "capture_executor"
        subtopic_goal = "/capture_core/goal_gripper_pose"

        self.THRES_ARRIVAL = 0.01

        # --- ROS node init --- #
        rospy.init_node(node_name,log_level=rospy.DEBUG)
        suber_goal = rospy.Subscriber(
                        subtopic_goal, 
                        PoseStamped, 
                        self.callback_goal, 
                        queue_size=5)

        # --- Panda robot init --- #
        self.robot = PandaArm() # create PandaArm instance
        self.reset_pose()

        # --- State variables --- #
        self.buff_suber_goal = None
        self.curr_pose = self.update_pose()
        self.is_arrive = False
        self.curr_task = None


    def callback_goal(self, data):
        self.buff_suber_goal = np.array([
            # data.header.stamp.secs,
            # data.header.stamp.nsecs,
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        ])
        # rospy.logdebug("Callback called")
    
    def check_arrival(self):
        self.update_pose()
        if self.curr_pose is not None \
            and self.curr_task is not None:
            if np.linalg.norm(self.curr_task - self.curr_pose) < self.THRES_ARRIVAL:
                self.is_arrive = True 
            else:
                 False
        else:
            self.is_arrive = False

        if self.is_arrive:
            self.curr_task = None

        return self.is_arrive


    def set_new_goal(self, goal_pose=None):
        if goal_pose is not None:
            self.curr_task = goal_pose
        elif self.curr_pose is not None \
            and self.buff_suber_goal is not None \
            and not self.is_arrive:
            
            self.curr_task = self.buff_suber_goal
            rospy.loginfo("New task has been set to {}".format(self.curr_task))
        

    def exec(self):
        if self.curr_task is None:
            rospy.loginfo_once("No task set")
            return

        try:
            while not self.check_arrival():
                # move the robot end-effector to pose specified by 'pos','ori'
                self.robot.move_to_cartesian_pose(
                                self.curr_task[:3],
                                self.curr_task[3:]) 
            rospy.loginfo("Arrive to {}".format(self.curr_pose))
        finally:
            rospy.loginfo("func exec quits")

    
    def reset_pose(self):
        self.robot.move_to_neutral() # moves robot to neutral pose; uses moveit if available, else JointTrajectory action client
        self.robot.get_gripper().home_joints() # homes gripper joints
        self.robot.get_gripper().open() # open gripper
        rospy.loginfo("Reset to home pose")
    

    def update_pose(self):
        p,o = self.robot.ee_pose()
        curr_pose = np.concatenate((p, quat.as_float_array(o)))
        self.curr_pose = curr_pose.copy()
        return curr_pose

if __name__ == "__main__":
    executor = Executor()
    try:
        while True:
            executor.set_new_goal()
            executor.exec()
            rospy.sleep(0.1)
            # pass
    finally:
        rospy.loginfo("executor exit")