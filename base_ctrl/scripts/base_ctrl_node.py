#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import math



def callback_pose(data):
    global x
    global y
    global theta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    theta = math.atan2(data.pose.pose.orientation.z,
                       data.pose.pose.orientation.w)*2
    print ('x: ' + str(x) + ' y: ' + str(y) + ' theta: ' + str(theta))


def callback_goal(data):
    global xd
    global yd
    xd = data.data[0]
    yd = data.data[1]
    print("goalx: " + str(xd) + " goaly: " + str(yd))


def sat(val, ksat):
    if val > ksat:
        return ksat
    if val < -ksat:
        return -ksat
    return val


def main():
    global xd
    global yd
    xd = 0
    yd = 0
    k = 0.5
    kd = 0.5
    ksat = 0.3
    global x
    global y
    global theta
    x = 0
    y = 0
    theta = 0
    rospy.init_node('base_ctrl')
    rate = rospy.Rate(10)
    pub_speeds = rospy.Publisher('/hardware/mobile_base/speeds',
                                 Float32MultiArray, queue_size=1)
    rospy.Subscriber('/navigation/localization/current_pose',
                     PoseWithCovarianceStamped, callback_pose)
    rospy.Subscriber('/pot_fields/goal_position',
                     Float32MultiArray, callback_goal)

    while not rospy.is_shutdown():
        msg_speeds = Float32MultiArray()
#       print(x)
        ea = math.atan2(yd - y, xd - x) - theta
        if ea <= -math.pi:
            ea = ea + 2*math.pi
        elif ea > math.pi:
            ea = ea - 2*math.pi
        r = math.sqrt((yd - y)**2 + (xd - x)**2)
        Vr = k*ea + kd*r*(1-abs(sat(ea, ksat))/ksat)
        Vl = -k*ea + kd*r*(1-abs(sat(ea, ksat))/ksat)
        print("Vr: " + str(Vr) + " Vl: " + str(Vl))
        msg_speeds.data = [Vl, Vr]
        pub_speeds.publish(msg_speeds)
        rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
