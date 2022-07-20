#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from mujoco_py import load_model_from_path, MjSim, MjViewer
import math
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mujoco_py.generated import const
from scipy.signal import butter, lfilter, freqz
import time

joint_state_moveit = [0,0,0,0,0,0]
# kp = [1000000000, 100, 10, 10, 1, 1]
kp = [10, 100, 10, 10, 1, 1]
kd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
step = 0.0005
joint_error_previous = [0,0,0,0,0,0]

def joint_state_moveit_callback(data):
    # print(data.position)
    global joint_state_moveit
    joint_state_moveit = data.position

def PID_control(joint_state, joint_target):
    global joint_error_previous 
    joint_torque = [0,0,0,0,0,0]
    

    joint_error = [0,0,0,0,0,0]
    joint_error_dot = [0,0,0,0,0,0]
    

    joint_error = joint_target - joint_state
    joint_error_dot = (joint_error - joint_error_previous)/step

    while joint_error.all() > 0.01 or joint_error.all() < -0.01:
        joint_torque = kp*joint_error + kd*joint_error_dot
        joint_error_previous = joint_error
        return joint_torque

joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
rospy.init_node('mujoco_arm_node', anonymous=True)
sub = rospy.Subscriber("/joint_states", JointState, joint_state_moveit_callback)
rate = rospy.Rate(10)
_xml_path = 'ur3_mujoco.xml'
model = load_model_from_path(_xml_path)
sim = MjSim(model)
viewer = MjViewer(sim)
sim.step()

val = -2
q=[]
while True:
    # print(joint_state)
    joint_target = joint_state_moveit
    joint_state = sim.data.qpos
    torques = PID_control(joint_state, joint_target)
    # print(torques)

    # sim.data.ctrl[0] = 100000000
    # sim.data.set_joint_qpos('shoulder_pan_joint', -1.57) 
    for i in range(0,6):
        sim.data.ctrl[i] = torques[i]
    # sim.data.ctrl[0] = -3
    # sim.data.ctrl[1] = -30
    # sim.data.ctrl[2] = -3
    # sim.data.ctrl[3] = -3
    # sim.data.ctrl[4] = -3
    # sim.data.ctrl[5] = -3
    # print(sim.data.)
    # sim.data.set_joint_qpos('shoulder_pan_joint', joint_state[0])
    # sim.data.set_joint_qpos('shoulder_lift_joint', joint_state[1])
    # sim.data.set_joint_qpos('elbow_joint', joint_state[2])
    # sim.data.set_joint_qpos('wrist_1_joint', joint_state[3])
    # sim.data.set_joint_qpos('wrist_2_joint', joint_state[4])
    # sim.data.set_joint_qpos('wrist_3_joint',   joint_state[5])
    print(sim.data.actuator_force[0],sim.data.actuator_force[1])
    # for i in range(0,6):
        # sim.data.set_joint_qpos(joint_names[i]+'_joint', val)
    #     sim.data.ctrl[i] = 3.14
        # print(joint_state[i])
    # print(sim.data.get_joint_qpos('elbow_joint'))
    # print(sim.data.sensordata)
    sim.step()
    viewer.render()