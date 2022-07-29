#!/usr/bin/env python3
from __future__ import print_function
import rospy
from sensor_msgs.msg import JointState
from mujoco_py import load_model_from_path, MjSim, MjViewer
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mujoco_py.generated import const
from scipy.signal import butter, lfilter, freqz
import time
import tf
from geometry_msgs.msg import Twist
import impedance_match
from std_msgs.msg import String, Bool


joint_state_moveit = [0,0,0,0,0,0]
kp = [50, 50, 50, 50, 50, 50]
kd = [1, 1, 1, 1, 1, 1]
step = 0.0005
joint_error_previous = [0,0,0,0,0,0]
pose = Twist()

pub = rospy.Publisher('/impedance_mathcing/goal', Twist, queue_size=10)

def callback_init(data):
    if data.data is True:
        xdes = object_id.idc()
        print(xdes)
        pose.linear.x = xdes[0]
        pose.linear.y = xdes[1]
        pose.linear.z = xdes[2]
        pose.angular.x = xdes[3]
        pose.angular.y = xdes[4]
        pose.angular.z = xdes[5]
        pub.publish(pose)

def joint_state_moveit_callback(data):
    global joint_state_moveit
    joint_state_moveit = data.position

def PID_control(joint_state, joint_target):
    joint_state = joint_state[:6]
    global joint_error_previous 
    joint_torque = [0,0,0,0,0,0]
    

    joint_error = [0,0,0,0,0,0]
    joint_error_dot = [0,0,0,0,0,0]
    

    joint_error = joint_target - joint_state
    joint_error_dot = (joint_error - joint_error_previous)/step

    if joint_error.all() > 0.01 or joint_error.all() < -0.01:
        joint_torque = kp*joint_error + kd*joint_error_dot
        joint_error_previous = joint_error
    return joint_torque

joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
rospy.init_node('mujoco_arm_node', anonymous=True)
sub = rospy.Subscriber("/joint_states", JointState, joint_state_moveit_callback)
rate = rospy.Rate(10)
_xml_path = '/home/pranav/orbit_ws/src/impedance_test/scripts/ur3_mujoco.xml'
model = load_model_from_path(_xml_path)
sim = MjSim(model)
viewer = MjViewer(sim)
sim.step()

x0 = [0,0.2,0.05,0,0,0]
object_id = impedance_match.idcontrol(sim,x0)

while not rospy.is_shutdown():
    sub = rospy.Subscriber("/init_flag", Bool, callback_init)
    joint_target = joint_state_moveit
    joint_state = sim.data.qpos
    torques = PID_control(joint_state, joint_target)
    for i in range(0,5):
        sim.data.ctrl[i] = torques[i]
    # print(sim.data.actuator_force[0],sim.data.actuator_force[1])
    sim.step()
    viewer.render()