#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math
import sys


def callback_pose(data):
    global x
    global y
    global theta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    theta = math.atan2(data.pose.pose.orientation.z,
                       data.pose.pose.orientation.w)*2
    print ('x: ' + str(x) + ' y: ' + str(y) + ' theta: ' + str(theta))


def callback_scan(data):
    global scan
    scan = data
#    print("scan len: " + str(scan.ranges.__len__()))


def sat(val, ksat):
    if val > ksat:
        return ksat
    if val < -ksat:
        return -ksat
    return val


def attractionForce(goalPos, currentPos):
    ka = 1.0
    Fa = [ka*(goalPos[0] - currentPos[0]), ka*(goalPos[1] - currentPos[1])]
    print("Fa: " + str(Fa))
    return Fa


def rejectionForce(laser, theta):
    kr = 1.0
    d0 = 1.0
    Fr = [0.0, 0.0]
    if laser.ranges.__len__() == 0:
        return [0.0, 0.0]
    for i in range(laser.ranges.__len__()):
        d = laser.ranges[i]
        if d <= d0:
            fri = math.sqrt(1/d - 1/d0)/(d*d*d)
        else:
            fri = 0.0
        angle = laser.angle_min + i*laser.angle_increment + theta
        u = [-math.cos(angle), - math.sin(angle)]
        Fr[0] += fri*u[0]
        Fr[1] += fri*u[1]
    Fr[0] *= kr/laser.ranges.__len__()
    Fr[1] *= kr/laser.ranges.__len__()
    print("Fr: " + str(Fr))
    return Fr


def nextPosition(currPos, Fatt, Frej):
    alpha = 0.3
    suma = [Fatt[0] + Frej[0], Fatt[1] + Frej[1]]
    mag = math.sqrt(suma[0]**2 + suma[1]**2)
    if mag == 0.0:
        return [currPos]
    Np = [currPos[0] + alpha*suma[0]/mag, currPos[1] + alpha*suma[1]/mag]
    print("Np: " + str(Np))
    return Np


def main(argv):
    xd = float(argv[1])
    yd = float(argv[2])
    global x
    global y
    global scan
    global theta
    theta = 0.0
    scan = LaserScan()
    x, y = 0.0, 0.0
    rospy.init_node('pot_fields')
    rate = rospy.Rate(10)
    pub_nextPos = rospy.Publisher('/pot_fields/goal_position',
                                  Float32MultiArray, queue_size=1)
    rospy.Subscriber('/navigation/localization/current_pose',
                     PoseWithCovarianceStamped, callback_pose)
    rospy.Subscriber('/hardware/scan',
                     LaserScan, callback_scan)

    while not rospy.is_shutdown():
        msg_goal = Float32MultiArray()
#       print(x)
        nextPos = nextPosition([x, y], attractionForce([xd, yd], [x, y]),
                               rejectionForce(scan, theta))
        msg_goal.data = nextPos
#        msg_goal.data = [0.0, 0.0]
        pub_nextPos.publish(msg_goal)
        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
