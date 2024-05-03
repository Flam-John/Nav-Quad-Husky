#!/usr/bin/env python

import rospy
import numpy as np
from mavros_msgs.msg import PositionTarget
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion
import math 


class ArucoVisualServoController:
    def __init__(self):
        rospy.init_node('aruco_visual_servo_controller', anonymous=True)

        #Subscribe
        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.aruco_callback)

        #Publish
        self.pub_cmd = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

        #Initialize
        self.target_pose = None

        #Gains cand Constants
        self.l = 0.5
	self.p = -0.01
        self.desired_distance = 2.0

        rospy.loginfo("Aruco Visual Servo")

    def aruco_callback(self, msg):
        if len(msg.transforms) > 0:
            	self.target_pose = np.zeros((3, 1))
		self.target_orientation = np.zeros((4, 1))
            	self.target_pose[0, 0] = msg.transforms[0].transform.translation.x
            	self.target_pose[1, 0] = msg.transforms[0].transform.translation.y
            	self.target_pose[2, 0] = msg.transforms[0].transform.translation.z
            	self.target_orientation[0, 0] = msg.transforms[0].transform.rotation.x
                self.target_orientation[1, 0] = msg.transforms[0].transform.rotation.y
                self.target_orientation[2, 0] = msg.transforms[0].transform.rotation.z
                self.target_orientation[3, 0] = msg.transforms[0].transform.rotation.w
		self.orientation_list = [msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w]
		self.angleslist = euler_from_quaternion(self.orientation_list)
		#print(self.angleslist)
        else:
            	self.target_pose = None


    def run(self):
	    rate = rospy.Rate(20)
	    while not rospy.is_shutdown():
		if self.target_pose is not None:
		    
		    # calculate the error
		    error_pose = np.zeros((3,)) 
		    error_pose[0] = self.target_pose[0, 0] - 0.0 
		    error_pose[1] = self.target_pose[1, 0] - 0.0 
		    error_pose[2] = self.target_pose[2, 0] - self.desired_distance 

		    angle_pose = np.zeros((3,))
		    angle = 2 * math.acos(self.target_orientation[3, 0])
		    angle_pose[0] = self.target_orientation[0, 0] / math.sqrt(1-self.target_orientation[3, 0]*self.target_orientation[3, 0])
		    angle_pose[1] = self.target_orientation[1, 0] / math.sqrt(1-self.target_orientation[3, 0]*self.target_orientation[3, 0])
		    angle_pose[2] = self.target_orientation[2, 0] / math.sqrt(1-self.target_orientation[3, 0]*self.target_orientation[3, 0])
    		    
		    current = np.zeros((3,))
		    current[0] = self.target_pose[0, 0]
		    current[1] = self.target_pose[1, 0]
		    current[2] = self.target_pose[2, 0]

                    cross_mult = np.zeros((3,))
		    cross_mult = np.cross(current ,  angle * angle_pose) 
		    
		    # errors
		    u = np.zeros((3,)) 
		    u[0] = self.l * error_pose[0] + self.p *  cross_mult[0]
		    u[1] = self.l * error_pose[1] + self.p *  cross_mult[1]
		    u[2] = self.l * error_pose[2] + self.p *  cross_mult[2]

		    # send to cmd
		    cmd_msg = PositionTarget()
		    cmd_msg.header.stamp = rospy.Time.now()
		    cmd_msg.coordinate_frame = 8
		    cmd_msg.type_mask=1479 
		    
		     
		    cmd_msg.velocity.x = -u[1]
		    cmd_msg.velocity.y = -u[0]
		    cmd_msg.velocity.z = -u[2]
		    cmd_msg.yaw_rate = -0.4 * self.angleslist[2]
		    self.pub_cmd.publish(cmd_msg)

		rate.sleep()

if __name__ == '__main__':
	aruco_visual_servo_controller = ArucoVisualServoController()
	aruco_visual_servo_controller.run()
