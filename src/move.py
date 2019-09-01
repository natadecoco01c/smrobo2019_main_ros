#!/usr/bin/env python
### coding: UTF-8  
# launch file example
#<node name="move" pkg="myapp" type="move.py" output="screen">
#  	<param name="KP_Vy" value="10"/>
#  	<param name="KI_Vy" value="0"/>
#  	<param name="KD_Vy" value="0"/>
#  	<param name="lim_Vy" value="3110"/>
#  	<param name="KP_omega" value="13"/> 
#  	<param name="KI_omega" value="0"/>
#  	<param name="KD_omega" value="0"/>
#  	<param name="lim_omega" value="5"/>
#  	<param name="epsilon_position" value="2"/>
#  	<param name="epsilon_theta" value="0.003"/>
#</node>

import tf
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
from math import pow, atan2, sqrt
import numpy as np
from time import sleep

def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def rangechange(theta,a):
    return theta+2*np.pi if theta < a else theta #[-pi,pi] to [a,a+2*pi]

class pid:

    def __init__(self,KP,KI,KD,target_val,Rate,lim):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.target_val = target_val
        self.Rate = Rate
        self.lim = lim
        self.diff = [0,0]
        self.integral = 0

    def do(self,sensor_val):
        self.diff[0] = self.diff[1]
        self.diff[1] = self.target_val - sensor_val 
        self.integral += (self.diff[0] + self.diff[1]) / 2.0 * self.Rate
        p = self.KP * self.diff[1]
        i = self.KI * self.integral
        d = self.KD * (self.diff[1]- self.diff[0]) / self.Rate
        e = p+i+d
        return e if np.fabs(e) < self.lim else self.lim * np.sign(e)

class omniwheel:

    def __init__(self):
        rospy.init_node('move', anonymous=True)
        
        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0

        self.epsilon_position = rospy.get_param('~epsilon_position')
        self.epsilon_theta = rospy.get_param('~epsilon_theta')
        self.KP_Vy = rospy.get_param('~KP_Vy')
        self.KI_Vy = rospy.get_param('~KI_Vy')
        self.KD_Vy = rospy.get_param('~KD_Vy')
        self.lim_Vy = rospy.get_param('~lim_Vy')
        self.KP_omega = rospy.get_param('~KP_omega')
        self.KI_omega = rospy.get_param('~KI_omega')
        self.KD_omega = rospy.get_param('~KD_omega')
        self.lim_omega = rospy.get_param('~lim_omega')
        self.x0 = 0
        self.y0 = 0
        self.yaw0 = 0
        self.rate = rospy.Rate(1000)
        self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.point_subscriber = rospy.Subscriber('/odom',PoseStamped, self.update_odom)

        self.point_subscriber = rospy.Subscriber('target_pos',Twist, self.get_goal)
        self.goal_reached_publisher = rospy.Publisher('target_reached',UInt8, queue_size=10)

    def update_odom(self,data):
        self.pose_x = data.pose.position.x
	self.pose_y = data.pose.position.y
	self.pose_theta = quaternion_to_euler(data.pose.orientation).z
    
    def error(self,x,y):
        return (x-self.pose_x)**2+(y-self.pose_y)**2

    def pub(self,Vx,Vy,omega):
        vel_msg = Twist()
        vel_msg.linear.x = Vx
        vel_msg.linear.y = Vy
        vel_msg.angular.z = omega
        self.velocity_publisher.publish(vel_msg)

    def stop(self,Rate=100):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
   
    def rotate(self,angle):
        Rate = 200
        r = rospy.Rate(Rate)
        pidcontroler_omega = pid(self.KP_omega,self.KI_omega,self.KD_omega,rangechange(angle,angle+np.pi),Rate,self.lim_omega)
        while (rangechange(self.pose_theta,angle+np.pi)-rangechange(angle,angle+np.pi))**2 > self.epsilon_theta:#動く前に回転
        	omega = pidcontroler_omega.do(rangechange(self.pose_theta,angle+np.pi))
                self.pub(0,0,omega)
                r.sleep()
        self.stop()
               
    def move(self,x,y,Rate = 200,p=0,eps=1):
            r = rospy.Rate(Rate) #$(Rate)Hz
            pidcontroler_Vx = pid(self.KP_Vy,self.KI_Vy,self.KD_Vy,x+self.x0,Rate,self.lim_Vy)
            pidcontroler_Vy = pid(self.KP_Vy,self.KI_Vy,self.KD_Vy,y+self.y0,Rate,self.lim_Vy)
            if p==0:#向きは無視
                while self.error(x+self.x0,y+self.y0) > self.epsilon_position*eps:
                    Vx = pidcontroler_Vx.do(self.pose_x)
                    Vy = pidcontroler_Vy.do(self.pose_y)
                    self.pub(Vx,Vy,0)
                    r.sleep()
            elif p==1:#y
                pidcontroler_omega = pid(self.KP_omega,self.KI_omega,self.KD_omega,rangechange(np.pi/2,3*np.pi/2),Rate,self.lim_omega)
                while self.error(x+self.x0,y+self.y0) > self.epsilon_position*eps:
                    Vx = pidcontroler_Vx.do(self.pose_x)
                    Vy = pidcontroler_Vy.do(self.pose_y)
                    omega = pidcontroler_omega.do(rangechange(self.pose_theta,3*np.pi / 2))
                    self.pub(Vx,Vy,omega)
                    r.sleep()
            elif p==2:#-x
                pidcontroler_omega = pid(self.KP_omega,self.KI_omega,self.KD_omega,np.pi,Rate,self.lim_omega)
                while self.error(x+self.x0,y+self.y0) > self.epsilon_position*eps:
                    Vx = pidcontroler_Vx.do(self.pose_x)
                    Vy = pidcontroler_Vy.do(self.pose_y)
                    omega = pidcontroler_omega.do(rangechange(self.pose_theta,0))
                    self.pub(Vx,Vy,omega)
                    r.sleep()
            elif p==3:#-y
                pidcontroler_omega = pid(self.KP_omega,self.KI_omega,self.KD_omega,3*np.pi/2,Rate,self.lim_omega)
                while self.error(x+self.x0,y+self.y0) > self.epsilon_position*eps:
                    Vx = pidcontroler_Vx.do(self.pose_x)
                    Vy = pidcontroler_Vy.do(self.pose_y)
                    omega = pidcontroler_omega.do(rangechange(self.pose_theta,np.pi/2))
                    self.pub(Vx,Vy,omega)
                    r.sleep()
	    self.stop()

    def get_goal(self,data):
        self.move(data.linear.x,data.linear.y)
	self.rotate(data.angular.z)
        msg = UInt8()  
        msg.data = 1
	self.goal_reached_publisher.publish(msg)
                       
if __name__ == '__main__':
    try:
        wheel = omniwheel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
