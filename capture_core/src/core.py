#!/home/tuos/miniconda3/bin/python
import numpy as np
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from moveit_msgs.msg import MoveGroupActionResult
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from panda_robot import PandaArm


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
        subtopic_moveitstat = "/move_group/status"
        subtopic_moveitresult = "/move_group/result"


        self.obj_stampose = None
        self.obj_dim = None

        self.gripper_thickness = 0.065
        self.offset = R.from_euler('ZYX', [0, np.pi, 0])     # offset between the hand frame in transformation tree and MoveIt


        # status var
        self.dim_ready  = False
        self.pose_ready = False
        self.dim_ready  = True
        self.pose_ready = True
        self.moveitstat_code = None
        self.moveitresultstat_code = None

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
        self.suber_moveitstat = rospy.Subscriber(
            subtopic_moveitstat,
            GoalStatusArray,
            self.callback_moveitstat,
            queue_size=5
        )
        self.suber_moveitresult = rospy.Subscriber(
            subtopic_moveitresult,
            MoveGroupActionResult,
            self.callback_moveitresult,
            queue_size=5
        )

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

        # Capture from right
        # self.goal_pose_o[:3] = [-(self.dim[0]/2 + self.gripper_thickness+0.1),-0.03, 0]
        # self.rot_o_g = R.from_quat([-0.5, 0.5, -0.5, 0.5]) # rotation from object frame to goal pose
        # self.rot_o_g = R.from_euler('ZYX', [0, 0, np.pi/2]) * self.rot_o_g

        # Capture from top
        # self.goal_pose_o[:3] = [0, -(self.dim[1]/2 + self.gripper_thickness+0.03), 0]
        # self.rot_o_g = R.from_euler('ZYX', [0, 0, -np.pi/2]) # rotation from object frame to goal pose
        # self.goal_pose_o[3:] = self.rot_o_g.as_quat()
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


    def callback_moveitstat(self, data):
        self.moveitstat_code = data.status_list[0].status
        # rospy.logdebug("MoveIt status code is {}".format(self.moveitstat_code))
        pass


    def callback_moveitresult(self, data):
        self.moveitresultstat_code = data.status.status
        rospy.logdebug("MoveIt result status code is {}".format(self.moveitresultstat_code))


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
        goal_pose[3:] = (self.offset*rot_w_g).as_quat()
        # goal_pose[3:] = rot_w_g.as_quat()

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

    # --- motion for capturing a cracker box at pose [0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0] --- #
    goal_pose = np.array([0.6, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0])
    # goal_pose = np.array([0.6, 0.0, -0.3, 0.0, 0.0, 0.0, 1.0])
    
    goal_pose[3:] = np.fromstring("0.73183567 -0.68028538 -0.02525618 0.03147193", sep=' ')

    goal_list = [goal_pose.copy()]
    goal_pose[:3] = [0.6, 0.0, 0.32]
    goal_list.append(goal_pose.copy())
    goal_pose[:3] = [0.6, 0.0, 0.5]
    goal_list.append(goal_pose.copy())
    
    print(goal_list)
    for i in range(0,2):
        print(i)
        goal_pose = goal_list[i%2]
        # while core.moveitstat_code != GoalStatus.PENDING \
        #     and core.moveitstat_code != GoalStatus.ACTIVE :
        core.moveitresultstat_code = GoalStatus.PENDING
        while core.moveitresultstat_code != GoalStatus.SUCCEEDED:
            # and core.moveitstat_code == GoalStatus.SUCCEEDED:
            
            core.pub_goal(goal_pose)
            rospy.loginfo("Goal pose is published as {}".format(goal_pose))
            # if core.moveitstat_code == GoalStatus.SUCCEEDED:
            #     break
            rospy.sleep(1)
            if core.moveitresultstat_code == GoalStatus.ABORTED:
                break
        rospy.sleep(2)
    rospy.loginfo("FINISH")
    rospy.spin()
    
    # while True:
    # for i in range(1,3):
    #     # goal_pose = core.calc_capture_point()
    #     # goal_pose = np.fromstring("0.3 -0.2 0.5 0 0 0 1 ", sep=' ')
    #     # goal_pose = np.fromstring("0.35718674 -0.34046052  0.250838767  0.73183567 -0.68028538 -0.02525618 0.03147193", sep=' ')
    #     # goal_pose[3:] = R.from_euler('ZYX', [np.pi, 0, 0]).as_quat()
    #     # goal_pose[3:] = R.from_euler('ZYX', [0, 1.57, 0]).as_quat()
    #     # goal_pose[3:] = R.from_euler('ZYX', [0, 0, 1.57]).as_quat()
    #     # goal_pose[3:] = (R.from_euler('ZYX', [0, 0, np.pi/2])* R.from_quat(goal_pose[3:])).as_quat()
    #     core.pub_goal(goal_pose)
    #     rospy.loginfo("Goal pose is published as {}".format(goal_pose))
    #     rospy.sleep(1)
    #     # break
    # pass
