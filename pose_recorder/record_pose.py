#!/home/tuos/miniconda3/bin/python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
import numpy as np

buff = []

class Recorder:
    def __init__(self, savepath, delay, duration):
        self.savepath = savepath
        self.delay = delay
        self.duration = duration
        self.enable = False

        rospy.init_node('pose_recorder', log_level=rospy.INFO)
        object_odom_suber = rospy.Subscriber(
            '/odom/soup_can_pose',
            Odometry,
            self.object_odom_cb,
            queue_size=10
        )
        self.object_odom_buff = []

        dope_estm_suber = rospy.Subscriber(
            '/dope/pose_soup',
            PoseStamped,
            self.dope_estm_suber,
            queue_size=10
        )
        self.dope_estm_buff = []

    def object_odom_cb(self, data):
        newmsg = [
            data.header.stamp.secs,
            data.header.stamp.nsecs,
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z,
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
            data.twist.twist.linear.x,
            data.twist.twist.linear.y,
            data.twist.twist.linear.z,
            data.twist.twist.angular.x,
            data.twist.twist.angular.y,
            data.twist.twist.angular.z
        ]
        if self.enable:
            self.object_odom_buff.append(newmsg)
        rospy.logdebug("object_odom_buff contains {0} records".format(len(self.object_odom_buff)))

    def dope_estm_suber(self, data):
        newmsg = [
            data.header.stamp.secs,
            data.header.stamp.nsecs,
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        ]
        if self.enable:
            self.dope_estm_buff.append(newmsg)
        rospy.logdebug("dope_estm_buff contains {0} records".format(len(self.dope_estm_buff)))

    def exec(self):
        now = rospy.get_time()  # time (sec) in float
        while now == 0:
            now = rospy.get_time()
            # print("now is {0}".format(now))

        startat = now
        while now < startat + self.delay:
            now = rospy.get_time()
            print("Recording will start in {:3d}s".format(int(startat+self.delay-now)),end='\r')
        print('')
        
        while now < startat + self.duration + self.delay:
            now = rospy.get_time()
            self.enable = True
            print("Recording runs ETA: {:3d}s".format(int(startat+self.duration+self.delay-now)),end='\r')
        self.enable = False
        print('')

        np.savetxt(
            self.savepath["object_odom_savepath"], 
            self.object_odom_buff, delimiter=','
        )
        rospy.loginfo(
            "{:d} records have been saved to {}".format(
                len(self.object_odom_buff), 
                self.savepath["object_odom_savepath"]
            )
        )
        np.savetxt(
            self.savepath["dope_estm_savepath"], 
            self.dope_estm_buff, delimiter=','
        )
        rospy.loginfo(
            "{:d} records have been saved to {}".format(
                len(self.dope_estm_buff), 
                self.savepath["dope_estm_savepath"]
            )
        )


if __name__ == "__main__":
    rec = Recorder(
        {'object_odom_savepath': '/home/tuos/catkin_ws/src/pose_recorder/data/object_odom_rec.csv',
        'dope_estm_savepath': '/home/tuos/catkin_ws/src/pose_recorder/data/dope_estm_rec.csv'}, 
        5, 10
    )
    rec.exec()
    # rospy.spin()