from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mujoco_py.generated import const

def euler_to_quaternion(x, y, z):
    qx = np.sin(x/2) * np.cos(y/2) * np.cos(z/2) - np.cos(x/2) * np.sin(y/2) * np.sin(z/2)
    qy = np.cos(x/2) * np.sin(y/2) * np.cos(z/2) + np.sin(x/2) * np.cos(y/2) * np.sin(z/2)
    qz = np.cos(x/2) * np.cos(y/2) * np.sin(z/2) - np.sin(x/2) * np.sin(y/2) * np.cos(z/2)
    qw = np.cos(x/2) * np.cos(y/2) * np.cos(z/2) + np.sin(x/2) * np.sin(y/2) * np.sin(z/2)
    return [qx, qy, qz, qw]

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_move_node', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("ur_group3")
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)
rate = rospy.Rate(10)
listener = tf.TransformListener()

# pose_goal.position.x = 0.457
# pose_goal.position.y = 0.112
# pose_goal.position.z = 0.067
pose_goal = geometry_msgs.msg.Pose()
# pose_goal.position.x = 0
# pose_goal.position.y = 0.2
# pose_goal.position.z = 0.2
# quat = euler_to_quaternion(0, 0, np.radians(0))
# pose_goal.orientation.x = quat[0]
# pose_goal.orientation.y = quat[1]
# pose_goal.orientation.z = quat[2]
# pose_goal.orientation.w = quat[3]

# move_group.set_pose_target(pose_goal)
# move_group.go(wait=True)

# move_group.stop()
# move_group.clear_pose_targets()

# print("move 1")

pose_goal.position.x = 0
pose_goal.position.y = 0.2
pose_goal.position.z = 0.05
quat = euler_to_quaternion(0, 0, np.radians(0))
pose_goal.orientation.x = quat[0]
pose_goal.orientation.y = quat[1]
pose_goal.orientation.z = quat[2]
pose_goal.orientation.w = quat[3]
move_group.set_pose_target(pose_goal)
move_group.go(wait=True)

move_group.stop()
move_group.clear_pose_targets()

print("move 2")

waypoints = []
wpose = move_group.get_current_pose().pose
wpose.position.y +=  0.2  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

(plan, fraction) = move_group.compute_cartesian_path(
    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold

move_group.execute(plan, wait=True)

# pose_goal.position.x = 0
# pose_goal.position.y = 0.4
# pose_goal.position.z = 0.04
# quat = euler_to_quaternion(0, 0, np.radians(0))
# pose_goal.orientation.x = quat[0]
# pose_goal.orientation.y = quat[1]
# pose_goal.orientation.z = quat[2]
# pose_goal.orientation.w = quat[3]
# move_group.set_pose_target(pose_goal)
# move_group.go(wait=True)

# move_group.stop()
# move_group.clear_pose_targets()

print("move 3")


while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('base_link', 'wrist_3_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    
    # print(rot)

    # move_group.set_pose_target(pose_goal)
    # move_group.go(wait=True)

    # move_group.stop()
    # move_group.clear_pose_targets()

    # move_group.set_pose_target(pose_goal2)
    # move_group.go(wait=True)

    # move_group.stop()
    # move_group.clear_pose_targets()