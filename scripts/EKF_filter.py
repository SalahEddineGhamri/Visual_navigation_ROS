#! /usr/bin/env python

# title           : EKF_filter.py
# description     : This module reads /vo, /odom, /imu and uses the extended kalman
#                   filtering for the fusion.
# author          : Salah Eddine Ghamri
# date            : 17-03-2018
# version         : 0.5
# usage           : in Roslaunch file add:
#                   <node name="EKF_filter" pkg="package_name" type="EKF_filter.py" output="screen">
# notes           : Rate affects response
# python_version  : 2.6.7
# ==================================================================================

import rospy
import KALMAN_lib

# initialisation------------------------------------
# node initialisation
rospy.init_node('EKF_filter')
frequency = 200.00  # it depends
Rate = rospy.Rate(frequency)  # very dangerous pay attention

# x is of the form [x, y, theta, v, omega]'---------
x = [[0.0], [0.0], [0.0], [0.0], [0.0]]
# P is covariance size(x) x size(x)
P = [0.0, 0.0, 0.0, 0.0, 0.0]
# variance of process noise
sigma_v = 0.1
sigma_omega = 0.1
# variance of measurement noise must be declared in
# "R" in KALMAN_lib.estimate() if needed.
# --------------------------------------------------

kalman = KALMAN_lib.kalman_class(x, P)
caller = KALMAN_lib.caller()

if __name__ == "__main__":
    try:
        old_time = rospy.Time().now().to_sec()
        # loop continues while ros is not shutdown
        while not rospy.is_shutdown():

            # time step for prediction
            new_time = rospy.Time().now().to_sec()
            T = new_time - old_time

            # read sensors now
            caller.read_sensors()

            # Estimation, estimates and return estimation error.
            # but we don't estimate the first time
            error = kalman.estimate(caller)

            # predict the next robot position
            kalman.predict(T, sigma_v, sigma_omega)

            # Publish on /odom_combined
            kalman.publish_message(caller)
            old_time = new_time

            # rospy.sleep(10)
            Rate.sleep()

    except rospy.ROSInterruptException:
        pass
