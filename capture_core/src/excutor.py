#!/home/tuos/miniconda3/bin/python

import numpy as np
import quaternion as quat
import rospy
from std_msgs.msg import String, Bool
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from panda_robot import PandaArm
from scipy.spatial.transform import Rotation as R

class Executor:
    def __init__(self):
        # --- params initialization --- #

        # Reserve for reading config file
        # ...

        subtopic_goal = "/capture_core/goal_gripper_pose"
        subtopic_resest = "/{}/reset".format(node_name)
        subtopic_moveitstat = "/move_group/status"

        self.THRES_ARRIVAL = 0.01

        # --- Suber/puber init --- #
        suber_goal = rospy.Subscriber(
            subtopic_goal, 
            PoseStamped, 
            self.callback_goal, 
            queue_size=1
        )
        suber_reset = rospy.Subscriber(
            subtopic_resest,
            Bool,
            self.callback_reset
        )
        suber_moveitstat = rospy.Subscriber(
            subtopic_moveitstat,
            GoalStatusArray,
            self.callback_moveitstat
        )


        # --- Panda robot init --- #
        self.robot = PandaArm() # create PandaArm instance
        self.reset_pose()


        # --- Status variables --- #
        self.moveitstat_code = 3
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

    
    def callback_reset(self, data):
        if data.data:
            self.reset_pose()


    def callback_moveitstat(self, data):
        self.moveitstat_code = data.status_list[0].status
        pass
    
    def check_arrival(self,_pose=None):
        self.update_pose()

        if self.curr_task is None:
            self.is_arrive = False

        elif self.curr_pose is not None:
            _pose = self.curr_task if _pose is None else _pose
            diff = np.linalg.norm(_pose[:3] - self.curr_pose[:3])
            rospy.logdebug("meth check_arrival: diff = {:f}".format(diff))

            if diff < self.THRES_ARRIVAL:
                self.is_arrive = True 
                self.curr_task = None
                rospy.loginfo("Arrive to {}".format(self.curr_pose))

            else:
                self.is_arrive = False

        else:
            self.is_arrive = False

        return self.is_arrive


    def set_new_goal(self, goal_pose=None):
        if goal_pose is not None:
            self.curr_task = goal_pose
        elif self.buff_suber_goal is not None:
            self.curr_task = self.buff_suber_goal
            self.buff_suber_goal = None
            rospy.loginfo("Task is set to {}".format(self.curr_task))

    def exec(self):
        if self.curr_task is None:
            # rospy.loginfo_once("No task set")
            return

        try:
            # while not self.check_arrival():
                # move the robot end-effector to pose specified by 'pos','ori'
                # self.curr_task[3:] = (R.from_euler('ZYX', [np.pi, 0, 0]) * R.from_quat(self.curr_task[3:])).as_quat()
            goal_pose = self.curr_task.copy()
            temp = R.from_quat(self.curr_task[3:]).as_euler('ZYX')
            goal_pose[3:] = R.from_euler('ZYX', np.array([temp[2], -temp[1], -temp[0]+np.pi])).as_quat()
            rospy.loginfo("Goal pose {} is transmited".format(goal_pose))
            self.robot.move_to_cartesian_pose(
                            pos=goal_pose[:3],
                            ori=quat.from_float_array(goal_pose[3:]))

            if self.moveitstat_code == 3: # success
                pass
            elif self.moveitstat_code == 4: # aborted
                pass
            rospy.sleep(0.5)
            self.robot.exec_gripper_cmd(0, 1000)
            self.curr_task=None
        finally:
            rospy.logdebug("exec quits")

    
    def reset_pose(self):
        rospy.loginfo("Recieve reset command")
        self.robot.move_to_neutral() # moves robot to neutral pose; uses moveit if available, else JointTrajectory action client
        self.robot.get_gripper().home_joints() # homes gripper joints
        self.robot.get_gripper().open() # open gripper
        rospy.loginfo("Reset to home pose")
    

    def update_pose(self):
        p,o = self.robot.ee_pose()
        curr_pose = np.concatenate((p, quat.as_float_array(o)))
        self.curr_pose = curr_pose.copy()
        orien_euler = R.from_quat(self.curr_pose[3:]).as_euler('ZYX')
        rospy.logdebug("EE current orientation is {}".format(orien_euler))

        return curr_pose

if __name__ == "__main__":
    node_name = "capture_executor"
    rospy.init_node(node_name,log_level=rospy.DEBUG)
    executor = Executor()
    try:
        while True:
            executor.set_new_goal()
            executor.exec()
            rospy.sleep(0.1)
            # pass
    finally:
        rospy.loginfo("executor exit")