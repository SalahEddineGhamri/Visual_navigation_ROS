#! /usr/bin/env python

# title           : vo_publisher.py
# description     : This will create the visual odometry based on Ar-alvar
#                   readings,  after conversion to odometry typed messages
#                   they will be published on a topic named /vo.
# author          : Salah Eddine Ghamri
# date            : 15-03-2018
# version         : 0.4
# usage           : in Roslaunch file add
#                   <node name="vo_publisher" pkg="package_name" type="vo_publisher.py" output="screen">
# notes           : The position of landmarks are stored here (change if needed).
#                   If Ar-alvar is not recognizing a tag "vo_publisher.py" will not
#                   publish.
# python_version  : 2.6.7
# System          : Ubuntu
# ==============================================================================
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,  Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers

# msg format is Odometry
msg = Odometry()

# a handler for a publisher
pub = rospy.Publisher('/vo',  Odometry,  queue_size=100)

# frame definition
frame_id = 'base_footprint'

# transformation de tf
transform = tf.TransformListener()

# covariance matrix chosen variance "sqrt(0.01)",  this must be well determined.
# "0.1" as variance means that ar_alvar is very precise +- 10 cm.
P = np.mat(np.diag([0.01]*6))
P = np.array(P).reshape(6,  6)
P_twist = np.mat(np.diag([0.01]*6))
P_twist = np.array(P_twist).reshape(6, 6)

# dictionary for landmarks position on the map
alvar_matrices = {}

# landmark coordinates are stored as 4x4 matrices
# in ldmark dictionary ldmark['marker_...'][2]
# if a new landmark is added just add here.
ldmark = {'marker_0': ((-4.309,  -1.589,  0.31), (0.707,  0.0,  0.707,  0.0)),
          'marker_1': ((-4.157,  -5.707,  0.31), (0.707,  0.0,  0.707,  0.0)),
          'marker_2': ((-0.949,  -0.891,  0.31), (0.0,  0.0,  -0.792,  0.611)),
          'marker_3': ((-1.808,  -4.031,  0.31), (0.0,  0.0,  0.608,  0.793)),
          'marker_4': ((-5.503,   0.251,  0.31), (0.0,  0.0,  -0.086,  0.996)),
          'marker_5': ((-6.086,  -1.588,  0.31), (0.0,  0.0,  -0.105,  0.994)),
          'marker_6': ((-6.343,  -2.841,  0.31), (0.0,  0.0,  -0.144,  0.989)),
          'marker_7': ((-10.311,  0.256,  0.31), (0.0,  0.0,  -0.117,  0.993))}

for key in ldmark:
    ldmark[key] = (ldmark[key][0], ldmark[key][1],
                   transform.fromTranslationRotation(ldmark[key][0],  ldmark[key][1]))


def callback_alvar_message(message):
    for tag in message.markers:
        Trans = (tag.pose.pose.position.x,
                 tag.pose.pose.position.y,
                 tag.pose.pose.position.z)
        Rot = (tag.pose.pose.orientation.x,
               tag.pose.pose.orientation.y,
               tag.pose.pose.orientation.z,
               tag.pose.pose.orientation.w)
        matrix = transform.fromTranslationRotation(Trans,  Rot)
        alvar_matrices["marker_"+str(tag.id)] = matrix


if __name__ == "__main__":
    rospy.init_node('vo_publisher')
    listener = tf.TransformListener()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            now = rospy.Time(0)
            now_stamp = rospy.Time.now()

            # read alvar messages and store tags position in alvar_matrices if existed
            rospy.Subscriber("/ar_pose_marker",  AlvarMarkers,  callback_alvar_message)

            # if alvar_matrices contains a landmark, so it is detected.
            if len(alvar_matrices) >= 1:
                # here more regorous method need to be taken
                # tags are put in the map of ascending order, the closer -> min
                dtctd = min(alvar_matrices.keys())
                tran_Rmark1 = alvar_matrices[dtctd]
            else:
                continue

            # rotation arround "Y" with (Pi/2).
            # tran_Rmark1 robot position in the tags reference frame.
            rot_mat = np.array(((0, 0, 1, 0), (0, 1, 0, 0), (-1, 0, 0, 0), (0, 0, 0, 1)))
            tran_Rmark1 = np.dot(rot_mat, tran_Rmark1)

            # -----------------------
            detected_marker = '/'+dtctd
            listener.waitForTransform('/map', detected_marker, now, rospy.Duration(10))
            (tran,  rott) = listener.lookupTransform('/map', detected_marker, now)
            tran_markmap = listener.fromTranslationRotation(tran, rott)

            # Here we call the position of the detected landmark
            # tran_markmap = ldmark[dtctd][2]
            # Here we calculate the robot postion based on the alvar and rviz.
            robot_pos = np.dot(tran_markmap,  tran_Rmark1)
            # extracting postion from the homogeneous 4x4 matrix

            x = robot_pos[0, 3]
            y = robot_pos[1, 3]
            z = robot_pos[2, 3]

            # extracting Quaternion from the homogeneous 4x4 matrix.
            quatern = tf.transformations.quaternion_from_matrix(robot_pos)

            # Constructing the message
            msg.header.stamp = now_stamp
            msg.header.frame_id = frame_id
            # msg.header.child_frame_id = child_frame_id
            msg.pose.pose.position = Point(x, y, z)
            msg.pose.pose.orientation = Quaternion(*quatern)
            msg.pose.covariance = tuple(P.ravel().tolist())
            msg.twist.covariance = tuple(P_twist.ravel().tolist())

            # publishing the message, only when distance (z) is under 3 meters.
            # not the best solution, we need reflection on this.
            if np.abs((alvar_matrices[dtctd])[2, 3]) <= 3:
                pub.publish(msg)
            else:
                pass
            rate.sleep()

        except(tf.LookupException,  tf.ConnectivityException, rospy.ROSInterruptException):
            continue
