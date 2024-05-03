#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped

huskyX = []
huskyY = []
quadX = []
quadY = []

def callback1(msg):
 
 Hx0 = msg.pose.pose.position.x 
 Hy0 = msg.pose.pose.position.y
 huskyX.append(Hx0)
 huskyY.append(Hy0)
 #rospy.loginfo("HYSKY %s %s",Hx0,Hy0)

def callback2(msg):
 
 Qx0 = msg.pose.position.x 
 Qy0 = msg.pose.position.y
 quadX.append(2+Qy0)
 quadY.append(-Qx0)
 #rospy.loginfo("QUAD %s %s",Qx0,Qy0) 
 #rospy.loginfo("===================================================================")

def listener():

 	rospy.init_node("odommapr",anonymous=True)
        rospy.Subscriber("/odometry/filtered",Odometry, callback1)
	rospy.Subscriber("/mavros/local_position/pose",PoseStamped, callback2)
        rospy.spin()

if __name__ == '__main__':
 listener()
 plt.scatter(huskyX, huskyY, label='husky')
 plt.scatter(quadX, quadY, label='quad')
 # Set labels and title
 plt.xlabel('X')
 plt.ylabel('Y')
 plt.title('Scatter Plot')
 plt.legend()

 # Display the plot
 plt.show()

