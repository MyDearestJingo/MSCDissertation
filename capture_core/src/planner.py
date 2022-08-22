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
    def __init__(self, node_name, obj_name, _obj_dim=None, _obj_pose=None):

        # ROS subscribers & publishers
        subtopic_dim = "/dope/dimension_{name}".format(name=obj_name)
        subtopic_pose = "/dope/pose_{name}".format(name=obj_name)
        pubtopic_captpose = "/{name}/capture_pose".format(name=node_name)

        self.suber_dim = rospy.Subscriber(
            subtopic_dim, String, self.callback_dim, queue_size=1)
        self.suber_pose = rospy.Subscriber(
            subtopic_pose, PoseStamped, self.callback_pose, queue_size=1)
        self.puber_captpose = rospy.Publisher(
            pubtopic_captpose, PoseStamped, queue_size=5
        )

        # member vars
        self.obj_dim = _obj_dim
        self.obj_pose =_obj_pose

        # waiting for subers
        while self.obj_dim is None or self.obj_pose is None:
            rospy.sleep(0.5)

        pass
    
    
    def callback_objpose(self, data):
        self.obj_pose = np.array([
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
        pass


    def callback_dim(self, data):
        str = data.data
        self.obj_dim = np.fromstring(str[1:-1],sep=',',dtype=np.float64)/100
        
        if not self.dim_ready:
            self.dim_ready = True
        pass


    def calc_capture_pose_moveitf(
            self, _camera_pose:list, _capt_pose:list, eul_order=None, pub=True):
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
        if eul_order is not None:
            _camera_pose[3:] = R.from_euler(
                eul_order, _camera_pose[3:]).as_quat()
            _capt_pose[3:] = R.from_euler(
                eul_order, _capt_pose[3:]).as_quat()

        obj_pose = self.obj_pose.copy()
        obj_dim = self.obj_dim.copy()

        # rotation from object frame to EE frame (TF tree)
        rot_o_g = R.from_quat(_capt_pose[3:]) 
        #TODO [ ]: transform EE goal pose from TF tree frame to MoveIt frame 
        #TODO      and update rot_o_g

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
        
        if pub:
            self.pub_capture_pose(_capt_pose)

        return _capt_pose

    
    def pub_capture_pose(self, _capt_pose):
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        # msg.header.frame_id = 'camera_link_optical'
        msg.pose.position.x    = _capt_pose[0]
        msg.pose.position.y    = _capt_pose[1]
        msg.pose.position.z    = _capt_pose[2]
        msg.pose.orientation.x = _capt_pose[3]
        msg.pose.orientation.y = _capt_pose[4]
        msg.pose.orientation.z = _capt_pose[5]
        msg.pose.orientation.w = _capt_pose[6]
        self.puber_goal.publish(msg)

if __name__ == "__main__":
    node_name = "capture_core_planner"
    rospy.init_node(node_name, log_level=rospy.DEBUG)
    planner = Planner(node_name, "cracker")

    camera_pose = np.fromstring("1.5 0 0.25 1.57 -0 -1.57", sep=' ')
    capt_palm_pose = np.array([0, -0.13, 0, 0, 0, np.pi/2])

    while True:
        pose = planner.calc_capture_pose_moveitf(
            camera_pose, capt_palm_pose, eul_order="ZYX")
        rospy.logdebug("capture pose: {}".format(pose))
        rospy.sleep(0.5)
