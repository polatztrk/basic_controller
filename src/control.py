#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist



def imu_callback(data):

    orientation_quater=data.orientation
    (roll,pitch,yaw)=euler_from_quaternion([orientation_quater.x,orientation_quater.y,orientation_quater.z,orientation_quater.w])
    our_yaw = math.degrees(yaw)


def callback(data):
    current_x = data.x
    current_y = data.y
    diff = yaw-target_yaw
    diff2 = math.atan2(target_y-current_y,target_x-current_x)
    r = math.sqrt((current_x-target_x)**2+(current_y-target_y)**2)
    vel = k_velocity*math.cos(diff2)*r
    ang = k_angular*diff2
    msg = Twist()
    msg.linear.x = vel
    msg.k_angular.z = ang
    point_pub.publish(msg)
    


def gps_subscriber_node():
    global point_pub, axis_marker_pub, point_marker_pub,k_velocity,k_angular,our_yaw,target_yaw,target_lat,target_lon
    k_velocity = 1
    k_angular = 1
    target_yaw = 30
    target_x = 5
    target_y = 5

    rospy.init_node('control_subscriber', anonymous=True)
    rospy.Subscriber("/imu/data2",Imu, imu_callback,queue_size=10)
    rospy.Subscriber('/coordinate', Point, callback)
    # Create publishers
    point_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
 
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_subscriber_node()
    except rospy.ROSInterruptException:
        pass
