#!/usr/bin/env python

import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from final_challenge_2022.msg import StopSignLocation


class StopSignController():
    """
    A controller for stopping when encountering a stop sign.
    Listens for a relative stop sign location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/stop_sign_location", StopSignLocation, self.stop_sign_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)

    def stop_sign_callback(self, msg):
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

        if abs(vel) < 0.25: #prevent sticking and not moving. 
            vel = 0.25
        

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
#
        drive_cmd.drive.steering_angle = delta
        drive_cmd.drive.speed = vel

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
