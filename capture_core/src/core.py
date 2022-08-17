#!/home/tuos/miniconda3/bin/python
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R


class Capturer:
    def __init__(self):
        # --- params initialization --- #

        # Reserve for reading config file
        # ...

        obj_name = "cracker"

        node_name = "capture_core"
        subtopic_dim = "/dope/dimension_{name}".format(name=obj_name)
        subtopic_pose = "/dope/pose_{name}".format(name=obj_name)
        pubtopic_goal = "/{}/goal_gripper_pose".format(node_name)

        self.obj_stampose = None
        self.obj_dim = None

        self.gripper_thickness = 0.065

        # state var
        self.dim_ready  = False
        self.pose_ready = False

        # camera pose
        # @ TODO:
        # @   [ ]. Init camera pose from config file
        self.cam_pose = np.fromstring("1.5 0 0.2 1.57 -0 -1.57", sep=' ') # orientation in yaw-pitch-roll
        self.rot_w_c = R.from_euler('ZYX', self.cam_pose[-3:])   # rotation from world frame to camera frame

        # ROS node init
        rospy.init_node(node_name,log_level=rospy.DEBUG)
        self.suber_dim = rospy.Subscriber(subtopic_dim, String, self.callback_dim, queue_size=1)
        self.suber_pose = rospy.Subscriber(subtopic_pose, PoseStamped, self.callback_pose, queue_size=1)
        self.puber_goal = rospy.Publisher(pubtopic_goal, PoseStamped)

        # --- goal gripper pose calculation --- #
        while not (self.pose_ready and self.dim_ready):
            if not self.pose_ready:
                rospy.loginfo_once("Waiting for topic {} ready".format(subtopic_pose))
            if not self.dim_ready:
                rospy.loginfo_once("Waiting for topic {} ready".format(subtopic_dim))
            rospy.sleep(0.1)
        rospy.loginfo("Subscribed topics ready")

        # initial goal pose of gripper within object frame
        # @ TODO
        # @   [ ]. Init from config file
        rospy.logdebug("Init goal pose")
        self.goal_pose_o = np.zeros(7)
        self.goal_pose_o[:3] = [-(self.dim[0]/2 + self.gripper_thickness),0,0]
        # self.rot_o_g = R.from_euler('zyx', [-np.pi/2, 0, -np.pi/2]) # rotation from object frame to goal pose
        self.rot_o_g = R.from_quat([-0.5, 0.5, -0.5, 0.5]) # rotation from object frame to goal pose
        self.goal_pose_o[3:] = self.rot_o_g.as_quat()
        # rospy.logdebug(self.goal_pose_o)


    # unpackage the dimension msg
    def callback_dim(self, data):
        str = data.data
        self.dim = np.fromstring(str[1:-1],sep=',',dtype=np.float64)/100
        
        if not self.dim_ready:
            self.dim_ready = True
        pass

    # unpackage the object pose msg
    def callback_pose(self, data):
        self.obj_stampose = np.array([
            data.header.stamp.secs,
            data.header.stamp.nsecs,
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        ])
        if not self.pose_ready:
            self.pose_ready = True

    def calc_capture_point(self):
        obj_pose = self.obj_stampose[2:].copy()    # fix the current pose
        goal_pose = self.goal_pose_o.copy()
        # print(self.goal_pose_o[:3])


        # --- gripper goal position calculation ---
        rot_c_o = R.from_quat(obj_pose[-4:])    # rotation from camera frame to object frame
        # goal_pose[:3] = rot_c_o.inv().apply(goal_pose[:3]) 
        goal_pose[:3] = rot_c_o.apply(goal_pose[:3]) 
        goal_pose[:3] += obj_pose[:3]           # within camera frame now

        # goal_pose[:3] = self.rot_w_c.inv().apply(goal_pose[:3]) # <- wrong output
        goal_pose[:3] = self.rot_w_c.apply(goal_pose[:3])       # <- correct output
        # rospy.logdebug(self.rot_w_c.as_euler('ZYX'))
        rospy.logdebug((self.rot_w_c * self.rot_w_c.inv()).as_euler("ZYX"))
        goal_pose[:3] += self.cam_pose[:3]      # within world frame now
        rospy.logdebug(goal_pose[:3])


        # --- gripper goal orientation calculation --- #

        ## --- in camerea frame --- ##
        # rot_c_g = rot_c_o * self.rot_o_g # checked
        # goal_pose[3:] = rot_c_g.as_quat()

        ## --- in world frame --- ##
        rot_w_g = self.rot_w_c * rot_c_o * self.rot_o_g # checked
        offset = R.from_euler('ZYX', [0, np.pi, 0])     # offset between the hand frame in transformation tree and MoveIt
        goal_pose[3:] = (offset*rot_w_g).as_quat()

        # goal_pose[:3] = [0,0.5,0]                 # debug in world frame
        # goal_pose[:3] = obj_pose[:3] + [0.3,0,0]  # debug in camera frame

        # goal_pose[3:] = self.rot_w_c.as_quat()    # camera orientation checked
        # goal_pose[3:] = self.rot_o_g.as_quat()    # goal gripper orientation checked
        # goal_pose[3:] = rot_c_o.as_quat()         # object orientation checked

        # temp_r = R.from_quat(goal_pose[3:])
        # rospy.logdebug(temp_r.as_euler('ZYX'))

        # camera pose checked
        # goal_pose[:3] = self.cam_pose[:3] + [0,0.2,0]
        # goal_pose[3:] = self.rot_w_c.as_quat()

        # rospy.logdebug(goal_pose[3:])
        
        return goal_pose
    
    def pub_goal(self, _goal_pose):
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        # msg.header.frame_id = 'camera_link_optical'
        msg.pose.position.x    = _goal_pose[0]
        msg.pose.position.y    = _goal_pose[1]
        msg.pose.position.z    = _goal_pose[2]
        msg.pose.orientation.x = _goal_pose[3]
        msg.pose.orientation.y = _goal_pose[4]
        msg.pose.orientation.z = _goal_pose[5]
        msg.pose.orientation.w = _goal_pose[6]
        self.puber_goal.publish(msg)

if __name__ == "__main__":
    core = Capturer()
    
    while True:
        goal_pose = core.calc_capture_point()
        core.pub_goal(goal_pose)
        rospy.loginfo("Goal pose is published as {}".format(goal_pose))
        rospy.sleep(0.1)
    pass
