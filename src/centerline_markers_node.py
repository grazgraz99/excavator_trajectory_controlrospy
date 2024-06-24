#!/usr/bin/env python3
from visualization_msgs.msg import Marker
import rospy, math
import pandas as pd
import rospkg
from geometry_msgs.msg import Point





'''
Publishes markers for RVIZ showing the centerline that the vehicle is following
'''
class centerline_node:
    def __init__(self):
        self.publisher=rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        # self.rospack=rospkg.RosPack
        # centerline_path=rospack.get_path('task2_centerline.csv')
        self.overall()

    def overall(self):
        self.load_pts()
        self.marker_publisher()
        
        
    def load_pts(self):
        df=pd.read_csv('/home/cse4568/catkin_ws/src/excavator_trajectory_controlrospy/maps/task2_centerline.csv', sep=',')
        self.x_values=df['x'].tolist()
        self.y_values=df['y'].tolist()
        

    def marker_publisher(self):
        self.marker_msg=Marker()
        self.marker_msg.header.frame_id="odom"
        self.marker_msg.header.stamp=rospy.Time()
        # marker_msg.ns="my_namespace"
        self.marker_msg.action=0
        self.marker_msg.type=4
        self.marker_msg.scale.x=.05
        # marker_msg.scale.y=2
        # marker_msg.scale.z=2
        self.marker_msg.pose.orientation.w=1
        self.marker_msg.color.g=1
        self.marker_msg.color.a=1
        for x, y in zip(self.x_values, self.y_values):
            p=Point()
            p.x=x
            p.y=y
            self.marker_msg.points.append(p)
  


if __name__ == "__main__":
    rospy.init_node('centerline_visualization_node')
    centerline=centerline_node()
    
    rate=rospy.Rate(60)
    while not rospy.is_shutdown():
        centerline.publisher.publish(centerline.marker_msg)
        rate.sleep()
    rospy.spin()




        