#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

x0 = y0 = yaw = 0.0
u = w = 0.0
distancex = distancey = 0.0
e = a = theta = 0.0
g = 0.2
k = 0.7
h = 0.7


def callback(msg):

 x0 = msg.pose.pose.position.x 
 y0 = msg.pose.pose.position.y
 yaw = msg.pose.pose.orientation.z
 rospy.loginfo("%s %s %s",x0,y0,yaw)

 if ((x0 < 1.99 or y0 < 1.99)):
   distancex = x0 - 2 
   distancey = y0 - 2 
 elif (x0 >= 1.99 and y0 >= 1.99 and int(yaw) == 0 and  x0 < 3.99):
   distancex = x0 - 4 
   distancey = y0 - 4
 elif (x0 >= 3.99 and y0 >= 3.99 and int(yaw) == 0 and x0 < 5.99 ):
   distancex = x0 - 6
   distancey = y0 - 6
 elif (x0 >= 5.99 and y0 >= 5.99 ):
   distancex = x0 - 8
   distancey = y0 - 8
 elif (x0 >= 7.99 and y0 >= 7.99 ):
   rospy.signal_shutdown("Eftase ston proorismo tou")
 
 e = math.sqrt(distancex ** 2 + distancey ** 2)
 theta = math.atan2(-distancey,-distancex)
 a = theta - yaw
 u = (g*math.cos(a))*e
 w = k*a + g*(math.cos(a)*math.sin(a)/a)*(a+(h*theta)) 
 
 pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
 cmd_msg = Twist() 
 cmd_msg.linear.x = u 
 cmd_msg.angular.z = w 
 pub.publish(cmd_msg)


def listener():

 	rospy.init_node("odom_listener",anonymous=True)
        rospy.Subscriber("/odometry/filtered",Odometry, callback)
        rospy.spin()

if __name__ == '__main__':
 listener()

