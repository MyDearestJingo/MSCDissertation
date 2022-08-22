#!/home/tuos/miniconda3/bin/python
'''
Description: 
Author: Yi Lan (ylan12@sheffield.ac.uk)
Date: 2022-08-22 10:43:49
LastEditTime: 2022-08-22 12:28:20
LastEditors: Yi Lan (ylan12@sheffield.ac.uk)
'''
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

class Planner:
    def __init__(self, node_name:String, obj_name:String, _obj_dim=None, _obj_pose=None):

        # ROS subscribers & publishers
        subtopic_dim = "/dope/dimension_{name}".format(name=obj_name)
        subtopic_pose = "/dope/pose_{name}".format(name=obj_name)
        pubtopic_captpose_tftree = "/{name}/capture_pose_tftree".format(name=node_name)
        pubtopic_captpose_moveit = "/{name}/capture_pose_moveit".format(name=node_name)

        self.suber_dim = rospy.Subscriber(
            subtopic_dim, String, self.callback_dim, queue_size=1)
        self.suber_pose = rospy.Subscriber(
            subtopic_pose, PoseStamped, self.callback_objpose, queue_size=1)
        self.puber_captpose_tftree = rospy.Publisher(
            pubtopic_captpose_tftree, PoseStamped, queue_size=5
        )
        self.puber_captpose_moveit = rospy.Publisher(
            pubtopic_captpose_moveit, PoseStamped, queue_size=5
        )

        # member vars
        self.obj_dim = _obj_dim
        self.obj_pose =_obj_pose

        # rotation from palm frame (TF tree) to the base frame
        self.rot_p_b = R.from_quat([0, 1, 0, 0])

        # waiting for subers
        while self.obj_dim is None or self.obj_pose is None:
            rospy.loginfo_once("Waiting for topics of object dimension and pose ready")
            rospy.sleep(0.5)

        pass
    
    
    def callback_objpose(self, data):
        self.obj_pose = np.array([
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

    def callback_dim(self, data):
        str = data.data
        self.obj_dim = np.fromstring(str[1:-1],sep=',',dtype=np.float64)/100


    def calc_capture_pose_tftree(
            self, _camera_pose:list, _capt_pose:list, eul_order="", pub=True):
        '''
        Convert the capture pose (for EE frame in MoveIt) from object frame to 
        robot base frame.
        @param camera_pose: float list, pose of camera w.r.t the base frame 
                            of the robot.
        @param capt_pose:   float list, goal pose of ee in TF tree when 
                            performing capture w.r.t object frame.
        @param eul_order:   string, if as None, treat the orientation of two 
                            above params as quaternion orientation represented, 
                            otherwise, euler angle represented with eul_order 
                            specified
        @param pub:         boolean, if true, publish the capture pose to ROS
        '''
        if eul_order != "":
            _camera_pose = np.concatenate((
                _camera_pose[:3],
                R.from_euler(eul_order, _camera_pose[3:6]).as_quat()))
            _capt_pose = np.concatenate((
                _capt_pose[:3],
                R.from_euler(eul_order, _capt_pose[3:6]).as_quat()))

        obj_pose = self.obj_pose.copy()
        obj_dim = self.obj_dim.copy()

        # rotation from object frame to EE frame (TF tree)
        rot_o_g = R.from_quat(_capt_pose[3:]) 
        #TODO [ ]: transform EE goal pose from TF tree frame to MoveIt frame 
        #          and update rot_o_g

        # rotation from camera frame to object frame
        rot_c_o = R.from_quat(obj_pose[3:])

        # rotation from robot base frame to camera frame
        rot_b_c = R.from_quat(_camera_pose[3:])

        # transform capture pose from object frame to base frame
        ## position
        _capt_pose[:3] = rot_c_o.apply(_capt_pose[:3])
        _capt_pose[:3] += obj_pose[:3]
        _capt_pose[:3] = rot_b_c.apply(_capt_pose[:3])
        _capt_pose[:3] += _camera_pose[:3]
        ## orientation
        rot_b_g = rot_b_c * rot_c_o * rot_o_g
        _capt_pose[3:] = rot_b_g.as_quat()

        # rot_p_g = self.rot_p_b * rot_b_c * rot_c_o * rot_o_g
        # _capt_pose[3:] = rot_p_g.as_quat()
        
        if pub:
            self.pub_capture_pose(_capt_pose, self.puber_captpose_tftree)

        return _capt_pose


    def calc_capture_pose_moveit(
        self, _camera_pose:list, _capt_pose:list, eul_order="", pub=True):
        pose_tftree = self.calc_capture_pose_tftree(_camera_pose, _capt_pose, eul_order, pub)
        pose_moveit = (R.from_quat(pose_tftree[3:]) * R.from_euler("ZYX", [np.pi/4,0,0])).as_quat()
        if pub:
            self.pub_capture_pose(pose_moveit, self.puber_captpose_moveit)
        return 

    
    def pub_capture_pose(self, _capt_pose:list, puber:rospy.Publisher):
        msg = PoseStamped()
        msg.header.frame_id    = 'world'
        msg.pose.position.x    = _capt_pose[0]
        msg.pose.position.y    = _capt_pose[1]
        msg.pose.position.z    = _capt_pose[2]
        msg.pose.orientation.x = _capt_pose[3]
        msg.pose.orientation.y = _capt_pose[4]
        msg.pose.orientation.z = _capt_pose[5]
        msg.pose.orientation.w = _capt_pose[6]
        puber.publish(msg)


    def set_obj_pose(self, _obj_pose:list):
        l = len(_obj_pose)
        if l != 7:
            rospy.logerr("the length of object pose vector should be 7, \
                but {} is got".format(l))
            return False
        self.obj_pose = _obj_pose


    def set_obj_dim(self, _obj_dim:list):
        l = len(_obj_dim)
        if l != 7:
            rospy.logerr("the length of object dimension vector should be 7, \
                but {} is got".format(l))
        self.obj_dim = _obj_dim


if __name__ == "__main__":
    node_name = "capture_core_planner"
    rospy.init_node(node_name, log_level=rospy.DEBUG)

    # cracker box pose (in camrea frame) and dimension pre-definition
    obj_dim = np.fromstring("0.158 0.21 0.06", sep=' ')
    # obj_pose = np.concatenate(
    #     (np.fromstring("0 0 0.9 0 3.14 0", sep=' '), np.zeros(1)))
    # obj_pose[3:] = R.from_euler("ZYX", obj_pose[3:6]).as_quat()
    obj_pose = [0.01, -0.06, 0.91, 0.01, 1.0, 0.06, 0.01]

    planner = Planner(node_name, "cracker", obj_dim, obj_pose)

    camera_pose = np.fromstring("1.5 0 0.05 1.57 -0 -1.57", sep=' ')
    # palm pose (in object frame) for capture 
    # capt_palm_pose = np.array([0, -0.13, 0, 0, 0, -np.pi/2]) # grasp from top
    capt_palm_pose = np.array([-obj_dim[0]/2, 0, 0, -np.pi/2, 0, -np.pi/2]) # grasp from right

    while True:
        pose = planner.calc_capture_pose_tftree(
            camera_pose, capt_palm_pose, eul_order="ZYX")
        rospy.logdebug("capture pose in tf tree: {}".format(pose))
        rospy.sleep(0.5)
