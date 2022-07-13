#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool
import tf
import matplotlib.pyplot as plt
import time
import numpy as np
import scipy.io

sensor_ready = False
sensor_data_point = 0

def ready_callback(data):
    if data.data == True:
        global sensor_ready
        sensor_ready = True
        print("Sensor ready")

def sensor_callback(data):
    global sensor_data_point
    sensor_data_point = data.wrench.force
    

def move():
    print("Starting intial move")
    pub = rospy.Publisher('/left/pose_cmd', Twist, queue_size=1)
    sub = rospy.Subscriber('/sensor/ready', Bool, ready_callback)
    rospy.init_node('move_node', anonymous=True)
    listener = tf.TransformListener()

    rate = rospy.Rate(10) # 10hz
    twistInit = Twist()
    twistInit.linear.x = 0.210
    twistInit.linear.y = -.3554
    twistInit.linear.z = 0.054
    twistInit.angular.x = 1.57079625
    twistInit.angular.y = 0
    twistInit.angular.z = 0

    twistGoal = Twist()
    twistGoal.linear.x = 0.210
    twistGoal.linear.y = -.357
    twistGoal.linear.z = 0.054
    twistGoal.angular.x = 1.57079625
    twistGoal.angular.y = 0
    twistGoal.angular.z = 0

    transInit = 0
    reachedInitPos = False

    print("Waiting for sensor to be ready")
    while (not sensor_ready):
        rate.sleep()

    sensorSub = rospy.Subscriber('/sensor/external_ft', WrenchStamped, sensor_callback)
    force_x_array = []
    force_y_array = []
    force_z_array = []
    position_array = []

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('left/base', 'left/tool0', rospy.Time(0))
            if (type(transInit) == int):
                transInit = trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        
        if (abs(trans[1] - twistInit.linear.y) > .01 and not reachedInitPos):
            pub.publish(twistInit)
        elif abs(trans[1] - twistInit.linear.y) <= .01 and not reachedInitPos:
            print("Reached initial position")
            reachedInitPos = True
            time.sleep(3)

        if (reachedInitPos):
            pub.publish(twistGoal)
            print("Reading sensor data")
            if (sensor_data_point.z < -0.1) and abs(trans[1] - twistGoal.linear.y) >= .005:
                print(trans[1])

            force_x_array.append(sensor_data_point.x)
            force_y_array.append(sensor_data_point.y)
            force_z_array.append(sensor_data_point.z)
            position_array.append(abs(trans[1] - twistInit.linear.y))

        if (abs(trans[1] - twistGoal.linear.y) < .005 and reachedInitPos):
            print("Reached goal")
            break
        # else:
            # print(trans)
        rate.sleep()

    print("Done")
    dataz = np.array(force_z_array)
    pos = np.array(position_array)
    
    file_path1 = 'force_z.mat'
    file_path2 = 'pos.mat'

    scipy.io.savemat(file_path1, {'data': dataz})
    scipy.io.savemat(file_path2, {'data': pos})

    plt.plot(position_array, force_x_array, 'r')
    plt.plot(position_array,force_y_array, 'g')
    plt.plot(position_array,force_z_array,'b')
    plt.legend(['X','Y','Z'])
    plt.show()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
