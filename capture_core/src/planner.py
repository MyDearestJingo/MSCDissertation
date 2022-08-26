#!/home/tuos/miniconda3/bin/python
'''
Description: 
Author: Yi Lan (ylan12@sheffield.ac.uk)
Date: 2022-08-22 10:43:49
LastEditTime: 2022-08-26 12:56:34
LastEditors: Yi Lan (ylan12@sheffield.ac.uk)
'''
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
from moveit_commander import PlanningSceneInterface
from moveit_commander.conversions import pose_to_list, list_to_pose
from tf import transformations
import quaternion as quat

class Planner:
    def __init__(
        self, node_name:str, _obj_name:str, 
        _cam_pose:list, _obj_dim=None, _obj_pose=None):

        # ROS subscribers & publishers
        subtopic_dim = "/dope/dimension_{name}".format(name=_obj_name)
        subtopic_pose = "/dope/pose_{name}".format(name=_obj_name)
        pubtopic_captpose_tftree = "/{name}/{obj}_capture_pose_tftree".format(name=node_name, obj=_obj_name)
        pubtopic_captpose_moveit = "/{name}/{obj}_capture_pose_moveit".format(name=node_name, obj=_obj_name)

        self.suber_pose = rospy.Subscriber(
            subtopic_pose, PoseStamped, self.callback_objpose, queue_size=1)
        self.puber_captpose_tftree = rospy.Publisher(
            pubtopic_captpose_tftree, PoseStamped, queue_size=5
        )
        self.puber_captpose_moveit = rospy.Publisher(
            pubtopic_captpose_moveit, PoseStamped, queue_size=5
        )
        self.suber_dim = None
        if _obj_dim is None:
            self.suber_dim = rospy.Subscriber(
                subtopic_dim, String, self.callback_dim, queue_size=1)

        self.moveit_scene = PlanningSceneInterface()

        # member vars
        self.obj_name = _obj_name
        self.obj_dim = _obj_dim
        self.obj_pose =_obj_pose
        self.obj_pose_ts = None # float time stamp of self.obj_pose
        self.cam_pose = _cam_pose

        # rotation from palm frame (TF tree) to the base frame
        self.rot_p_b = R.from_quat([0, 1, 0, 0])

        # waiting for subers
        while self.obj_dim is None or self.obj_pose is None:
            rospy.loginfo_once("Waiting for topics of object dimension and pose ready")
            rospy.sleep(0.5)

        self.add_obj_to_moveit_scene()

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
        self.obj_pose_ts = data.header.stamp.secs + data.header.stamp.nsecs*1e-9

    def callback_dim(self, data):
        str = data.data
        self.obj_dim = np.fromstring(str[1:-1],sep=',',dtype=np.float64)/100


    def calc_capture_pose_tftree(
            self, _capt_pose:list, pub=True):
        '''
        Convert the capture pose (for EE frame in MoveIt) from object frame to 
        robot base frame.
        @param camera_pose: float list, pose of camera w.r.t the base frame 
                            of the robot.
        @param capt_pose:   float list, goal pose of ee in TF tree when 
                            performing capture w.r.t object frame.
        # @param eul_order:   string, if as None, treat the orientation of two 
        #                     above params as quaternion orientation represented, 
        #                     otherwise, euler angle represented with eul_order 
        #                     specified
        @param pub:         boolean, if true, publish the capture pose to ROS
        '''
        # if eul_order != "":
        #     _capt_pose = np.concatenate((
        #         _capt_pose[:3],
        #         R.from_euler(eul_order, _capt_pose[3:6]).as_quat()))

        obj_pose = self.obj_pose.copy()
        obj_dim = self.obj_dim.copy()
        capt_pose = _capt_pose.copy()

        # rotation from object frame to EE frame (TF tree)
        rot_o_g = R.from_quat(capt_pose[3:]) 

        # rotation from camera frame to object frame
        rot_c_o = R.from_quat(obj_pose[3:])

        # rotation from robot base frame to camera frame
        rot_b_c = R.from_quat(self.cam_pose[3:])

        # transform capture pose from object frame to base frame
        ## position
        capt_pose[:3] = rot_c_o.apply(capt_pose[:3]) + obj_pose[:3]
        capt_pose[:3] = rot_b_c.apply(capt_pose[:3]) + self.cam_pose[:3]
        ## orientation
        rot_b_g = rot_b_c * rot_c_o * rot_o_g
        capt_pose[3:] = rot_b_g.as_quat()

        # rot_p_g = self.rot_p_b * rot_b_c * rot_c_o * rot_o_g
        # _capt_pose[3:] = rot_p_g.as_quat()
        
        if pub:
            self.pub_capture_pose(capt_pose, self.puber_captpose_tftree)

        return capt_pose


    def calc_capture_pose_moveit(
        self, _capt_pose:list, pub=True):
        pose_tftree = self.calc_capture_pose_tftree(_capt_pose, pub)
        rospy.logdebug("capture pose in tf frame: {}".format(pose_tftree))
        pose_moveit = np.concatenate(
            (pose_tftree[:3], \
            (R.from_quat(pose_tftree[3:]) * R.from_euler("ZYX", [np.pi/4,0,0])).as_quat()))
        
        if pub:
            self.pub_capture_pose(pose_moveit, self.puber_captpose_moveit)
        
        return pose_moveit

    
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


    def predict_trajectory(self, _pred_horizon=5):
        '''
        # Predict the object trajectory for a set time horizon and create an 
        # occupancy zone in MoveIt scene.

        # @Param _pred_horizon: a float, specify the prediction horizon in seconds
        '''

        lin_vel = np.zeros(3)   # velocity vector in 3D
        ang_vel = np.zeros(3)   # angular velocities for xyz axes

        ## --- reserve space --- ##
        # sampling from DOPE observation for pose and linear velocities prediction
        # ...
        pose_pre = self.obj_pose.copy()
        pose_ts = self.obj_pose_ts.copy()

        ## --- pose prediction --- ##
        curr_t = rospy.Time.now()
        curr_t = curr_t.secs + curr_t.nsces * 1e-9
        t_diff = curr_t - pose_ts

        pose_pred = np.zeros(7)
        pose_pred[:3] = (t_diff + _pred_horizon) * lin_vel + pose_pre[:3]

        delta_ang = np.flip((t_diff + _pred_horizon) * ang_vel) # convert to zyx order
        delta_ori = quat.from_euler_angles(delta_ang) # convert to quat

        ori_pre = np.concatenate(pose_pre[6], pose_pre[3:5]) # convert to wxyz
        ori_pre = quat.from_float_array(ori_pre)
        ori_pred = quat.as_float_array(ori_pre * delta_ori)
        pose_pred[3:] = np.concatenate(ori_pred[4:6], ori_pred[3]) # convert to xyzw

        ## --- reserve space --- ##
        # create occupancy zone in MoveIt planning scene
        # ...






    def add_obj_to_moveit_scene(self,timeout=5):
        co_list = []
        obj_pose = PoseStamped()
        obj_pose.header.frame_id = "panda_link0"

        rot_b_c = R.from_quat(self.cam_pose[3:])
        
        # transformation from camera frame to base frame
        ori_b = (rot_b_c * R.from_quat(self.obj_pose[3:])).as_quat()
        pos_b = rot_b_c.apply(self.obj_pose[:3]) + self.cam_pose[:3]
        obj_pose.pose = list_to_pose(np.concatenate((pos_b, ori_b)))

        self.moveit_scene.add_box(self.obj_name, obj_pose, size=self.obj_dim)
        co_list.append(self.obj_name)

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            # attached_objects = scene.get_attached_objects([self.obj_name])
            extended_objects = self.moveit_scene.get_objects(co_list)
            is_added = len(extended_objects.keys()) >= len(co_list)

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            # is_known = self.obj_name in scene.get_known_object_names()

            # Test if we are in the expected state
            # if (box_is_attached == is_attached) and (box_is_known == is_known):

            if is_added:
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False
        pass


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


    def set_cam_pose(self, _cam_pose:list):
        self.cam_pose = _cam_pose

if __name__ == "__main__":
    node_name = "capture_core_planner"
    rospy.init_node(node_name, log_level=rospy.DEBUG)

    # cracker box pose (in camrea frame) and dimension pre-definition
    obj_dim = np.fromstring("0.158 0.21 0.06", sep=' ')
    # obj_pose = np.concatenate(
    #     (np.fromstring("0 0 0.9 0 3.14 0", sep=' '), np.zeros(1)))
    # obj_pose[3:] = R.from_euler("ZYX", obj_pose[3:6]).as_quat()
    obj_pose = [0.01, -0.06, 0.91, 0.0, 1.0, 0.0, 0.0]


    camera_pose = np.fromstring("1.5 0 0.05 1.57 -0 -1.57", sep=' ')
    camera_pose = np.concatenate(
        (camera_pose[:3],
        transformations.quaternion_from_euler(camera_pose[3], camera_pose[4], camera_pose[5], "rzyx")))
        
    # palm pose (in object frame) for capture 
    capt_palm_pose = np.array([0, -0.18, 0, 0, 0, -np.pi/2]) # grasp from top
    capt_palm_pose = np.concatenate(
        (capt_palm_pose[:3],
        transformations.quaternion_from_euler(capt_palm_pose[3], capt_palm_pose[4], capt_palm_pose[5], "rzyx")))
    # capt_palm_pose = np.array([-obj_dim[0]/2-0.1, 0, 0, -np.pi/2, 0, -np.pi/2]) # grasp from right

    planner = Planner(node_name, "cracker", camera_pose ,obj_dim, obj_pose)

    while True:
        pose = planner.calc_capture_pose_tftree(
            camera_pose, capt_palm_pose)
        rospy.logdebug("capture pose in tf tree: {}".format(pose))
        rospy.sleep(0.5)
