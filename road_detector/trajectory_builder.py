import rospy 
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, PoseArray, Point
import time, os 
from utils import LineTrajectory
from visualization_msgs.msg import Marker, MarkerArray
import rospkg 

class TrajectoryBuilder(object):
    """
    Listens to points published by road detector and uses them to build a trajectory. Publishes this trajectory to /road_detector/trajectory 
    """
    def __init__(self):
        self.trajectory = LineTrajectory('/trajectory')
        self.data_points = []
        self.point_sub = rospy.Subscriber('/road_detector/next_point', PointStamped, self.point_callback, queue_size=10)
        self.trajectory_pub = rospy.Publisher('/road_detector/trajectory', PoseArray, queue_size=10)
        self.trajectory_pub_marker = rospy.Publisher('/road_detector/trajectory_marker', MarkerArray, queue_size=10)
        self.trajectory.publish_viz()

    def publish_trajectory(self):
        self.traj_pub.publish(self.trajectory.toPoseArray())

    def point_callback(self, msg):
        self.data_points.append(msg.point)
        self.trajectory.addPoint(msg.point)
        
    def mark_pt(self, subscriber)
    