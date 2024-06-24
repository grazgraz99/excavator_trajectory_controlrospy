#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
import pandas as pd
import math

'''
'''
class state_calc:
    def __init__(self):
        
        self.pub=rospy.Publisher("kinematic_update",Float32MultiArray,queue_size=10)#setup publisher
        self.cmd_vel_pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)#setup cmd publisher
        odom_topic=rospy.get_param('odom_topic')
        self.odom_subscriber=rospy.Subscriber(odom_topic,Odometry,self.odom_callback)

        current_state=rospy.get_param('initial_state')
      
        self.x_pos=0
        self.y_pos=0
        self.qx=0
        self.qy=0
        self.qz=0
        self.qw=0

        self.goal_num=0
        self.lookahead_distance=3

        
   
        
    def odom_callback(self, data):
        self.x_pos=data.pose.pose.position.x
        self.y_pos=data.pose.pose.position.y
        print(f'x: {self.x_pos}, y: {self.y_pos}')

        #bot orientation(quaternion form)
        self.qx=data.pose.pose.orientation.x
        self.qy=data.pose.pose.orientation.y
        self.qz=data.pose.pose.orientation.z
        self.qw=data.pose.pose.orientation.w
        self.quaternion= [self.qx, self.qy, self.qz, self.qw]

        #Calculate heading angle(euler angle form)
        euler=euler_from_quaternion(self.quaternion)
        self.yaw=euler[2]

        self.pure_pursuit()
       
    def pure_pursuit(self):
        #self.get_goal#()
        # self.go_to_goal()
        self.choose_target()
        self.go_to_goal1()
        

    def choose_target(self):
        self.current_goal=self.goal_list[self.goal_num] #goal point selected from list
        self.goal_x=self.current_goal[0]#x goal extracted
        self.goal_y=self.current_goal[1]#y goal extracted

        self.distance_error=np.sqrt((self.goal_x-self.x_pos)**2+(self.goal_y-self.y_pos)**2)
        if self.distance_error<.5:
            self.goal_num+=1
    def go_to_goal1(self):
        command=Twist()
        angle_to_goal = np.arctan2(self.goal_y - self.y_pos, self.goal_x - self.x_pos)
        angle_error=angle_to_goal-self.yaw
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))


        command.angular.z=.5*angle_error

        if self.distance_error>1:
            self.distance_error=1

        command.linear.x=.75*self.distance_error        

        self.cmd_vel_pub.publish(command)#make the car move
        

    def get_goal(self): 
        for i, point in enumerate(self.goal_list):
            self.dist=np.sqrt((self.x_pos-point[0])**2+(self.y_pos-point[1])**2)
            # print(f'{point}')
            if (self.lookahead_distance-.5)<=self.dist<=(self.lookahead_distance+.5):#find a waypoint within tolerance of the lookahead distance
                self.waypoint=point#set the in tolerance point to be the next way point
                #self.goal_list=self.goal_list[i+1:]
                print(f'next waypoint: {self.waypoint}')
                break


    # def go_to_goal(self):
    #     command=Twist()
    #     curvature=2*self.y_pos-self.waypoint[1]/self.lookahead_distance**2
    #     angle_to_goal = np.arctan2(self.waypoint[1] - self.y_pos, self.waypoint[0] - self.x_pos)
    #     angle_diff=angle_to_goal-self.yaw
    #     #command.angular.z=.2*curvature
    #     command.angular.z=.25*angle_diff
    #     command.linear.x=.25*self.dist

    #     self.cmd_vel_pub.publish(command)

    def load_pts(self):
        self.goal_list=[]
        df=pd.read_csv('/home/cse4568/catkin_ws/src/excavator_trajectory_controlrospy/maps/task2_centerline.csv', sep=',')

        #df=pd.read_csv('/home/cse4568/catkin_ws/src/pa1-bicycle-kinematics-grazgraz99/maps/task2_centerline copy.csv', sep=',')
        self.x_values=df['x'].tolist()
        self.y_values=df['y'].tolist()
        for x,y in zip(self.x_values,self.y_values):
            self.goal_list.append([x,y])
        #print(f'{self.goal_list}')


    

if __name__ == "__main__":
    rospy.init_node('pa1_task1_node')
    pa1_task1_node=state_calc()
    pa1_task1_node.load_pts()#load the waypoints from the task2_centerline.csv
    
    rate=rospy.Rate(1)
    while not rospy.is_shutdown():
        #pa1_task1_node.publisher()
        rate.sleep()
    rospy.spin()