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

_xml_path = '/home/pranav/orbit_ws/src/impedance_test/scripts/ur3_mujoco.xml'
model = load_model_from_path(_xml_path)
sim = MjSim(model)
viewer = MjViewer(sim)
for i in range(6):
        sim.data.qpos[i]=1
sim.step()

x0 = [0,0.2,0.05,0,0,0]
        # xdes = object_id.idc()
object_id = impedance_match.idcontrol(sim,x0)

xdes = object_id.idc()

