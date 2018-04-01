# module for EKF_filter.py classes

# title           : KALMAN_lib.py
# description     : This module contains all classes of EKF_filter.py
# author          : Salah Eddine Ghamri
# date            : 17-03-2018
# version         : 0.4
# usage           : must be in the same folder of EKF_filter.py, no need
#                   to chmod +x this script
# notes           : //
# python_version  : 2.6.7
# Platforme       : Ubuntu
# =====================================================================

"""
Algorithm :
------Prediction-----------
predicted_x(k+1|k) = A(previous_x) previous_x(k|k) + B(previous_x) u(previous_x)
predicted_P(k+1|k) = A(previous_x) previous_P(k|k) A(previous_x)' + G(previous_x)QG(previous_x)'
------Kalman---------------
error(k) = z(k) - (C* predicted_x(k|k-1))
S(k) = C * predicted_P(k|k-1) * C' + R(k)
K(k) = predicted_P(k|k-1) * C' * S^-1
------Estimation------------
estimated_x(k|k) = predicted_x(k|k-1) + K(k) * error(k)
estimated_P(k|k) = (I - K(k) * C) predicted_P(k|k-1)
"""

import numpy as np
import rospy
from scipy.linalg import block_diag
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Point, Quaternion


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    y = angle
    y = y % (2 * np.pi)    # force in range [0, 2 pi)
    if y > np.pi:             # move to [-pi, pi)
        y -= 2 * np.pi
    return y


class kalman_class():

    def __init__(self, x, P):

        self.not_first_time = False
        self.previous_x = np.matrix(x)
        self.previous_P = np.diag(P)
        self.time = 0
        self.estimated_x = self.previous_x
        self.estimated_P = self.previous_P
        self.predicted_x = self.previous_x
        self.predicted_P = self.previous_P

        self.C_full = np.matrix([[1, 0, 0, 0, 0],
                                 [0, 1, 0, 0, 0],
                                 [0, 0, 1, 0, 0],
                                 [0, 0, 0, 1, 0],
                                 [0, 0, 1, 0, 0],
                                 [0, 0, 0, 0, 1],
                                 [1, 0, 0, 0, 0],
                                 [0, 1, 0, 0, 0],
                                 [0, 0, 1, 0, 0]])

        self.C_redu = np.matrix([[1, 0, 0, 0, 0],
                                 [0, 1, 0, 0, 0],
                                 [0, 0, 1, 0, 0],
                                 [0, 0, 0, 1, 0],
                                 [0, 0, 1, 0, 0],
                                 [0, 0, 0, 0, 1]])

    def predict(self, T, sigma_v, sigma_omega):
        """
        f = Matrix([[x + v * T * cos(theta + omega * T)],
                    [y + v * T * sin(theta + omega * T)],
                    [theta + omega * T],
                    [v],
                    [omega]])
        """

        self.predicted_x[2, 0] = self.estimated_x[2, 0] + self.estimated_x[4, 0] * T
        # normalize angle [-pi pi]------------------------------------------------
        self.predicted_x[2, 0] = normalize_angle(self.predicted_x[2, 0])
        # ------------------------------------------------------------------------
        self.predicted_x[0, 0] = self.estimated_x[0, 0] + self.estimated_x[3, 0] * T * np.cos(self.predicted_x[2, 0])
        self.predicted_x[1, 0] = self.estimated_x[1, 0] + self.estimated_x[3, 0] * T * np.sin(self.predicted_x[2, 0])
        self.predicted_x[3, 0] = self.estimated_x[3, 0]
        self.predicted_x[4, 0] = self.estimated_x[4, 0]

        # d_f is the jacobian of f for u = [v, omega]'
        ang = T*self.estimated_x[4, 0] + self.estimated_x[2, 0]
        # normalize angle [-pi pi]-----------------------------------------------
        ang = normalize_angle(ang)
        # ------------------------------------------------------------------------

        d_f = np.matrix([[T*np.cos(ang), (-(T**2)*self.estimated_x[3, 0]*np.sin(ang))],
                        [T*np.sin(ang), (T**2)*self.estimated_x[3, 0]*np.cos(ang)],
                        [0, T],
                        [1, 0],
                        [0, 1]])

        # d_f_prime is the jacobian of f for x
        d_f_prime = np.matrix([[1, 0, -T*self.estimated_x[3, 0]*np.sin(ang), T*np.cos(ang), -(T**2)*self.estimated_x[3, 0]*np.sin(ang)],
                              [0, 1, T*self.estimated_x[3, 0]*np.cos(ang), T*np.sin(ang), (T**2)*self.estimated_x[3, 0]*np.cos(ang)],
                              [0, 0, 1, 0, T],
                              [0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 1]])

        var_Q = np.matrix([[sigma_v**2, 0], [0, sigma_omega**2]])
        Q = d_f * var_Q * d_f.T
        self.predicted_P = d_f_prime * self.estimated_P * d_f_prime.T + Q

        # old state and covariance update
        self.previous_x = self.predicted_x
        self.previous_P = self.predicted_P

    def estimate(self, measure):
        # measurement in form z = Cx + v , v is white noise
        # in case we see the ar-tags (related to the publishing in /vo)
        # it calculates the error between measurement and prediction
        # update x and P

        # if we see we use full measurement matrix size
        # else we use the reduced size, so /vo is neglected.
        if measure.I_see_something:
            z = np.matrix([[measure.odom_x], [measure.odom_y],
                           [measure.odom_theta], [measure.odom_v],
                           [measure.imu_theta], [measure.imu_omega],
                           [measure.vo_x], [measure.vo_y], [measure.vo_theta]])
            C = self.C_full
            R = np.matrix(block_diag(measure.odom_covariance, measure.imu_covariance, measure.vo_covariance))
        else:
            z = np.matrix([[measure.odom_x], [measure.odom_y],
                           [measure.odom_theta], [measure.odom_v],
                           [measure.imu_theta], [measure.imu_omega]])
            C = self.C_redu
            R = np.matrix(block_diag(measure.odom_covariance, measure.imu_covariance))

        # Kalman gain
        S = C * self.previous_P * C.T + R
        self.K = self.previous_P * C.T * np.linalg.inv(S)

        # error calcualtion
        error = z - (C * self.previous_x)
        # normalize angle ------------------------------------------------
        error[2, 0] = normalize_angle(error[2, 0])
        # -----------------------------------------------------------------

        # we don't estimate first time
        if self.not_first_time:
            self.estimated_x = self.previous_x + self.K * error
            # normalize angle [-pi pi]------------------------------------
            self.estimated_x[2, 0] = normalize_angle(self.estimated_x[2, 0])
            # ------------------------------------------------------------
            mat = np.eye(5, dtype=int) - self.K * C
            self.estimated_P = mat * self.previous_P
        else:
            self.not_first_time = True

        # other ways to calculate innovation covarience
        # self.P = np.dot(mat, np.dot(self.predicted_P, mat.T)) + np.dot(self.K, np.dot(self.R, self.K.T))
        # self.P = self.predicted_P - np.dot( self.K, np.dot(self.C, self.predicted_P))
        self.update_velocity()
        return error

    def callback_velocity(self, msg):
        # calculates velocity and updates x
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        v = np.sqrt(vx**2 + vy**2)
        self.estimated_x[3, 0] = v
        self.estimated_x[4, 0] = omega

    def update_velocity(self):
        # subscriber in control commands topic
        rospy.Subscriber('/mobile_base/commands/velocity', Twist, self.callback_velocity)

    def publish_message(self, caller_obj):
        # publisher
        Pub = rospy.Publisher('/odom_combined', Odometry, queue_size=10)
        msg_odom = Odometry()
        # Publish kalma.x and kalman.P
        x = self.estimated_x[0, 0]
        y = self.estimated_x[1, 0]
        # we don't care for z <-- odometry
        z = 0
        # extracting Quaternion from the homogeneous
        quatern = quaternion_from_euler(0, 0, self.estimated_x[2, 0])
        # Constructing the message
        msg_odom.header.stamp = caller_obj.time_stamp
        msg_odom.header.frame_id = 'base_footprint'
        # msg.header.child_frame_id = child_frame_id
        msg_odom.pose.pose.position = Point(x, y, z)
        msg_odom.pose.pose.orientation = Quaternion(*quatern)
        p = np.diag([self.estimated_P[0, 0], self.estimated_P[1, 1], 10000000, 10000000, 1000000, self.estimated_P[3, 3]])
        msg_odom.pose.covariance = tuple(p.ravel().tolist())
        # publishing the message
        Pub.publish(msg_odom)


class caller():

    def __init__(self):

        self.n = 0
        self.odom_covariance = np.empty((4, 4), dtype=int)
        self.imu_covariance = np.empty((2, 2), dtype=int)
        self.vo_covariance = None
        self.odom_x = None
        self.odom_y = None
        self.odom_theta = None
        self.odom_v = None
        self.imu_theta = None
        self.imu_omega = None
        self.vo_x = None
        self.vo_y = None
        self.vo_theta = None
        self.I_see_something = False
        self.time_stamp = None

    def read_sensors(self):
        self.call_odom()
        self.call_imu()
        self.call_vo()
        pass

    def call_odom(self):
        try:
            msg = rospy.wait_for_message('/odom', Odometry, timeout=0.1)
            self.time_stamp = msg.header.stamp
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            # theta is the yaw we need to use tf.transforms quater -> euler
            # the euler_from_quaternion gives angle in [-pi, pi]
            quat = msg.pose.pose.orientation
            (roll, pitch, yaw) = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
            self.odom_theta = yaw
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            self.odom_v = np.sqrt(vx**2 + vy**2)
            pose_covariance = msg.pose.covariance
            twist_covariance = msg.twist.covariance
            a = pose_covariance[35]
            b = twist_covariance[0] + twist_covariance[7]
            self.odom_covariance = np.diag([pose_covariance[0],
                                            pose_covariance[7],
                                            a,
                                            b])
        except:
            pass

    def call_imu(self):
        try:
            msg = rospy.wait_for_message('/mobile_base/sensors/imu_data', Imu, timeout=0.1)
            # message de type Imu
            quat = msg.orientation
            # angle in [-pi, pi]
            self.imu_theta = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2]
            self.imu_omega = msg.angular_velocity.z
            self.imu_covariance = np.diag((msg.orientation_covariance[8],
                                           msg.angular_velocity_covariance[8]))
        except:
            pass

    def call_vo(self):
        try:
            msg = rospy.wait_for_message('/vo', Odometry, timeout=0.1)
            # same as odometry message
            self.vo_x = msg.pose.pose.position.x
            self.vo_y = msg.pose.pose.position.y
            quat = msg.pose.pose.orientation
            # angle in [-pi, pi]
            self.vo_theta = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2]
            pose_covariance = msg.pose.covariance
            self.vo_covariance = np.diag([pose_covariance[0],
                                         pose_covariance[7],
                                         pose_covariance[35]])
            self.I_see_something = True
        except:
            self.I_see_something = False
            pass
