#!/usr/bin/env python

# ROS Dependencies
import math
from math import sin, cos, pi
import re

import rospy
import tf


from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# Core Dependencies
import serial
import time

import sys
import traceback
import json
import uartlite


# Use relay for automatic error reset
relay_support = True
#config_file = open('/home/xilinx/ROS-ArloRobot-master/src/dhb10-controller/scripts/ip_addr.config', 'r')



global currentLeftWheelSpeed
global currentRightWheelSpeed
global previousLeftWheelSpeed
global previousRightWheelSpeed
global lastCallback
global robot_in_error


def velocity_callback(data):
  # asdf

  global currentLeftWheelSpeed
  global currentRightWheelSpeed
  global previousLeftWheelSpeed
  global previousRightWheelSpeed
  global lastCallback
  global robot_in_error
  
  currentLeftWheelSpeed = 0
  currentRightWheelSpeed = 0

  if data.linear.x:
    currentLeftWheelSpeed = encoder_postions_per_meter * data.linear.x
    currentRightWheelSpeed = encoder_postions_per_meter * data.linear.x

  if data.angular.z > 0:
    currentRightWheelSpeed += encoder_postions_per_meter * data.angular.z * wheelbase_radius_meters
    currentLeftWheelSpeed -= encoder_postions_per_meter * data.angular.z * wheelbase_radius_meters
  elif data.angular.z < 0:
    currentRightWheelSpeed += encoder_postions_per_meter * data.angular.z * wheelbase_radius_meters
    currentLeftWheelSpeed -= encoder_postions_per_meter * data.angular.z * wheelbase_radius_meters


  previousLeftWheelSpeed = currentLeftWheelSpeed
  previousRightWheelSpeed = currentRightWheelSpeed
  lastCallback = rospy.get_rostime()


try:

 # Init ROS node
 exc_info = sys.exc_info()
 
 ros_ns = rospy.get_namespace()
 rospy.init_node('motor_controller', anonymous=False)
 
 lastCallback = rospy.get_rostime()
 robot_in_error = True
 
 if ros_ns not in ['', '/', None]:
  rospy.Subscriber('/' + ros_ns + '/cmd_vel', Twist, velocity_callback)
 else: 
  rospy.Subscriber('/cmd_vel', Twist, velocity_callback)

 robot_saved_state = open('/home/wang/Desktop/saved-robot-state','w+')

 odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
 clone_pub = rospy.Publisher("clone_odom", Odometry, queue_size=50)
 tf_broadcaster = tf.TransformBroadcaster()

 # Statics
 wheelbase_radius_meters = 0.19685*0.1
 wheelbase_diameter_meters = wheelbase_radius_meters * 2
 encoder_postions_per_meter =300.7518

 # States
 # TODO - Use industrial messages/robot status here! http://docs.ros.org/kinetic/api/industrial_msgs/html/msg/RobotStatus.html
 robot_state = None
 robot_in_error = False



 # Tracked odometry
 x = 0.0
 y = 0.0
 th = 0.0

 vx = 0.0
 vy = 0.0
 vth = 0.0

 # Euler integrated pose defined in odometry frame
 x_timestep_plus_one = 0.0
 y_timestep_plus_one = 0.0
 th_timestep_plus_one = 0.0

 # Postional/rotational offset from a saved pose state for when the motor controller is reset during a run
 x_pos_saved_state_offset = 0.0
 y_pos_saved_state_offset = 0.0
 y_pos_saved_state_offset = 0.0
 x_orient_saved_state_offset = 0.0
 y_orient_saved_state_offset = 0.0
 z_orient_saved_state_offset = 0.0
 w_orient_saved_state_offset = 0.0
 covar_saved_state_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

 # Encoder readings
 left_wheel_encoder_pos_per_sec = 0.0
 right_wheel_encoder_pos_per_sec = 0.0


 # Velocity estimations
 left_wheel_velocity = 0.0
 right_wheel_velocity = 0.0

 vx_meters_per_sec = 0.0
 vth_meters_per_sec = 0.0
 vth_radians_per_sec = 0.0

 # Position estimations
 left_wheel_pos_meters = 0.0
 right_wheel_pos_meters = 0.0

 # Odometry frame
 odom_vx_meters_per_sec = 0.0
 odom_vy_meters_per_sec = 0.0
 odom_vth_radians_per_sec = 0.0

 currentLeftWheelSpeed = 0
 currentRightWheelSpeed = 0
 previousLeftWheelSpeed = 0
 previousRightWheelSpeed = 0

 current_time = rospy.Time.now()
 last_time = rospy.Time.now()
 last_degub_time = rospy.Time.now()


 while not rospy.is_shutdown():
   left_wheel_encoder_pos_per_sec = currentLeftWheelSpeed
   right_wheel_encoder_pos_per_sec = currentRightWheelSpeed
   current_time = rospy.Time.now()
   
   duration =(current_time -lastCallback).to_sec()
   if duration >= 2:    
    left_wheel_encoder_pos_per_sec = 0
    right_wheel_encoder_pos_per_sec = 0

   dt = (current_time - last_time).to_sec()
   left_wheel_velocity = left_wheel_encoder_pos_per_sec / encoder_postions_per_meter
   right_wheel_velocity = right_wheel_encoder_pos_per_sec / encoder_postions_per_meter

   vx_meters_per_sec = (left_wheel_velocity + right_wheel_velocity) / 2 
   vth_radians_per_sec = (right_wheel_velocity - left_wheel_velocity) / wheelbase_diameter_meters

   odom_vx_meters_per_sec = vx_meters_per_sec * math.cos(th) # - vy * sin(theta), vy = 0 for diff drive
   odom_vy_meters_per_sec = vx_meters_per_sec * math.sin(th) # + vy * cos(theta)
   odom_vth_radians_per_sec = vth_radians_per_sec

   x_timestep_plus_one = x + (odom_vx_meters_per_sec * dt)
   y_timestep_plus_one = y + (odom_vy_meters_per_sec * dt)
   th_timestep_plus_one = th + (odom_vth_radians_per_sec * dt)



   # Add any saved state offset, if any, to the odometry from motor controller readings
   x_timestep_plus_one += x_pos_saved_state_offset
   y_timestep_plus_one += y_pos_saved_state_offset
   th_timestep_plus_one += z_orient_saved_state_offset
   
   time_diff = rospy.Time.now() - last_degub_time

   
   last_degub_time = rospy.Time.now()

   # since all odometry is 6DOF we'll need a quaternion created from yaw
   odom_quat = tf.transformations.quaternion_from_euler(0, 0, th_timestep_plus_one)

   # first, we'll publish the transform over tf
   tf_broadcaster.sendTransform(
    (x_timestep_plus_one, y_timestep_plus_one, 0.),
    odom_quat,
    current_time,
    "base_link",
    "odom"
   )

   # next, we'll publish the odometry message over ROS
   odom = Odometry()
   odom.header.stamp = current_time
   odom.header.frame_id = "odom"

   # Set the position
   odom.pose.pose = Pose(Point(x_timestep_plus_one, y_timestep_plus_one, 0.), Quaternion(*odom_quat))

   # Set the velocity
   odom.child_frame_id = "base_link"
   odom.twist.twist = Twist(Vector3(vx_meters_per_sec, 0, 0), Vector3(0, 0, odom_vth_radians_per_sec))

   # Publish the message
   odom_pub.publish(odom)
   robot_saved_state.seek(0)
   robot_saved_state.write('\rSpeed: ' + str(odom.pose))

   last_time = current_time
   x = x_timestep_plus_one
   y = y_timestep_plus_one
   th = th_timestep_plus_one

   if th >= 2*math.pi:
    th = th - 2*math.pi

   if th < 0:
    th = 2*math.pi - th

   rospy.sleep(0.001)

finally:
 print("finilly")
 exit()
