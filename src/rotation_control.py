#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


prev_error = 0.0
prev_integral = 0.0
def imu_callback(data):
    global prev_error
    global prev_integral
    orientation_quater=data.orientation
    (roll,pitch,yaw)=euler_from_quaternion([orientation_quater.x,orientation_quater.y,orientation_quater.z,orientation_quater.w])
    our_yaw = math.degrees(yaw)
    if(our_yaw<0):
        our_yaw = 360+our_yaw
    error = target_yaw-our_yaw
    integral = (k_angular_i/20)*(error+prev_error)+prev_integral
    derivative = 20*k_angular_d*(error-prev_error)
    rospy.loginfo_throttle(1,"Error: %.4f, %.4f,%.4f,%.4f",error,target_yaw,math.degrees(yaw),prev_error)
    ang = k_angular_p*error+integral+integral+derivative
    msg = Twist()
    msg.angular.z = ang
    point_pub.publish(msg)
    prev_error = error
    prev_integral = integral

    
def gps_subscriber_node():
    global point_pub,k_angular_p,k_angular_i,our_yaw,target_yaw,prev_error,prev_integral,k_angular_d

    k_angular_p = 0.01
    k_angular_i = 0.000003
    k_angular_d = 0.004
    target_yaw = 10

    rospy.init_node('rot_control_subscriber', anonymous=True)
    rospy.Subscriber("/imu/data2",Imu, imu_callback,queue_size=10)

    point_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        rate.sleep()
 
if __name__ == '__main__':
    try:
        gps_subscriber_node()
    except rospy.ROSInterruptException:
        pass
