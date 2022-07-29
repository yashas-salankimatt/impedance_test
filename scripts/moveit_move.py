from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String, Bool
from moveit_commander.conversions import pose_to_list
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mujoco_py.generated import const

def callback_goal(goal):
    [x,y,z,w] = euler_to_quaternion(goal.angular.x, goal.angular.y, goal.angular.z)
    wpose = move_group.get_current_pose().pose
    wpose.position.x +=  goal.linear.x 
    wpose.position.y +=  goal.linear.y 
    wpose.position.z +=  goal.linear.z 
    wpose.orientation.x +=  x
    wpose.orientation.y +=  y
    wpose.orientation.z +=  z
    wpose.orientation.w +=  w
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    move_group.execute(plan, wait=True)

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
pub = rospy.Publisher("/init_flag", Bool, queue_size=10)
rate = rospy.Rate(10)
listener = tf.TransformListener()

waypoints = []
pose_goal = geometry_msgs.msg.Pose()

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


while not rospy.is_shutdown():
    rospy.Subscriber('/impedance_mathcing/goal', Twist, callback_goal)
    pub.publish(True)
    rate.sleep()
    try:
        (trans, rot) = listener.lookupTransform('base_link', 'wrist_3_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
