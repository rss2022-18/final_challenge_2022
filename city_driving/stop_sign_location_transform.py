#!/usr/bin/env python

import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from final_challenge_2022.msg import StopSignLocationPixel, StopSignLocation

# The following collection of pixel locations and corresponding relative
# ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

######################################################
# DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_IMAGE_PLANE = [[253, 203],
                   [471, 201],
                   [281, 178],
                   [441, 245]]  # dummy points
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
# DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_GROUND_PLANE = [[-12.75, 39.3701],
                    [12.75, 39.3701],
                    [-12.75, 50.3701],
                    [4.25, 23.0]]  # dummy points
######################################################

METERS_PER_INCH = 0.0254

class StopSignLocationTransform():
    """
    A controller for stopping when encountering a stop sign.
    Listens for a relative stop sign location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        self.stop_sign_px_sub = rospy.Subscriber("/stop_sign_bbox", Float32MultiArray, self.stop_sign_detection_callback)
        self.stop_sign_pub = rospy.Publisher("/stop_sign_location", StopSignLocation)

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rospy.logerr(
                "ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        # Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    def stop_sign_detection_callback(self, msg):
        # Extract information from message
        if msg is not None:
            contents = msg.data
            u = (contents[0] + contents[2])/2
            v = (contents[1] + contents[3])/2

            # Call to main function
            x, y = self.transformUvToXy(u, v)

            # Publish relative xy position of object in real world
            relative_xy_msg = StopSignLocationPixel()
            relative_xy_msg.x_pos = y
            relative_xy_msg.y_pos = -1*x
            self.stop_sign_pub.publish(relative_xy_msg)

    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.
        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.
        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
