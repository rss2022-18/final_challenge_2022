#!/usr/bin/env python

import rospy
import numpy as np
from cmath import cos, sin, sqrt
from math import atan2
from turtle import distance
import datetime

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from final_challenge_2022.msg import StopSignLocation

# NOTE: the car just has to reach zero momentum at a stop sign, there is no particular amount of time it must be stopped
class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        self.cone_detector_subscriber= rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        self.stop_sign_subscriber = rospy.Subscriber("/stop_sign_location", StopSignLocation, self.stop_sign_callback)
        

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)
        # 1 m/s is the absolute max speed for the city. for full credit, staff expect ~0.5 m/s to be sufficient
        self.desired_velocity = 1  #[m/s]
        self.L = 0.35 #[m]
        # In order to line follow, lookahead > parking_distance must be true
        # these two variables can be adjusted if needed. requires testing.
        self.lookahead = 0.90 # [m], variable
        self.parking_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.slow_down = False
        self.distance_from_stop_sign = None
        self.stopped = False
        self.dist_req = 1  # [m], we should stop about 0.75-1meter away from the sign
        self.timer_following = None
        self.timer_stopping = None
        self.stop_buffer = 3  # [s], amount of time to ignore a stop sign

    def stop_sign_callback(self, msg):
        if msg is not None:
            self.relative_x = msg.x_pos
            self.relative_y = msg.y_pos
            self.distance_from_stop_sign = sqrt(self.relative_x**2 + self.relative_y**2).real


        # The stop distance is about 0.75 - 1 meters
        elapsed_time = (self.timer_following - rospy.Time.now()).to_sec()
        # adding 0.25 meters gives us about 0.25 meters to slow down, this value can be changed / tuned
        if self.distance_from_stop_sign <= self.dist_req+0.25 and not self.slow_down and elapsed_time > self.stop_buffer:
            self.slow_down = True


    def relative_cone_callback(self, msg):
        if self.stopped:
            if self.timer_stopping is None:
                self.timer_stopping = rospy.Time.now()
            drive_cmd = AckermannDriveStamped()
            drive_cmd.drive.speed          = 0
            # TODO: do we also need to publish an angle?
            self.drive_pub.publish(drive_cmd)
            # we need to be stopped for at least two seconds, this time can be changed
            # another potential metric is, if we're too close to the sign, ignore it and continue?
            if (self.timer_stopping - rospy.Time.now()).to_sec() > 2:
                self.stopped = False
                self.slow_down = False
                self.timer_following = rospy.Time.now()
                self.timer_stopping = None
        else:
            if self.timer_following is None:
                self.timer_following = rospy.Time.now()
            self.relative_x = msg.x_pos
            self.relative_y = msg.y_pos
            drive_cmd = AckermannDriveStamped()

            distance_from_cone = sqrt(self.relative_x**2 + self.relative_y**2).real
            angle_to_cone      = atan2(self.relative_y, self.relative_x)
            numerator = 2*self.L*sin(angle_to_cone).real
            denominator = self.lookahead.real

            # pure pursuit nominal case
            delta = (atan2(numerator, denominator)).real # wow, this one worked really well. 
            
            # velocity scaling, according to angle. 
            vel = abs(self.desired_velocity*cos(angle_to_cone).real)

            # this value could be tuned, not sure what velocity threshold to use
            if abs(vel) < 0.25: #prevent sticking and not moving. 
                vel = 0.25
            
            # Note: Not sure if this will ever occur for line following
            if distance_from_cone < self.parking_distance*2 : #start slowing down
                slowdown_normalized = (distance_from_cone-self.parking_distance)/self.parking_distance
                vel = vel*(slowdown_normalized)

                if distance_from_cone >= self.parking_distance-0.05 and distance_from_cone <= self.parking_distance +0.05:
                    vel = 0
                elif distance_from_cone < self.parking_distance:
                    vel = -vel

            # want to face the cone, not just back into it. 
            if abs(angle_to_cone) > np.pi*0.5:
                #just turn in place
                if angle_to_cone > np.pi*0.5:
                    delta = np.pi*0.25
                    #print('spinning right')

                elif angle_to_cone < -np.pi*0.5:
                    delta = -np.pi*0.25
                    #print('spinning left ')
            
            #maybe baselink and everything else is already implemented?

            if self.slow_down:
                slowdown_normalized = (self.distance_from_stop_sign-self.dist_req)/self.parking_distance
                vel = vel*(slowdown_normalized)

                if self.distance_from_stop_sign >= self.dist_req-0.05 and self.distance_from_stop_sign <= self.dist_req +0.05:
                    vel = 0
                    self.stopped = True 
            
            drive_cmd.drive.steering_angle = delta
            drive_cmd.drive.speed          = vel

            self.drive_pub.publish(drive_cmd)
            self.error_publisher()

    def error_publisher(self):
        """ 
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        error_msg.x_error = float(self.relative_x)
        error_msg.y_error = float(self.relative_y)
        error_msg.theta_error = atan2(self.relative_y, self.relative_x)
        error_msg.distance_error = float((sqrt((self.relative_x)**2 + (self.relative_y)**2)).real)
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass