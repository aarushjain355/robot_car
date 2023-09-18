#!usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray
from path_planning.msg import lidar_data

publisher = rospy.Publisher("/scan", LaserScan, queue_size=10)
publisher2 = rospy.Publisher("/position", Float32MultiArray, queue_size=10)

msg = Float32MultiArray()
def map_callback(occupancyGrid):

    pass    

def lidar_callback(data):

    angles = data.angles
    distances = data.distances

    laser_scan_msg = LaserScan()

    laser_scan_msg.header.stamp = rospy.Time.now()
    laser_scan_msg.header.frame_id = "lidar data"

    laser_scan_msg.angle_min = min(angles)
    laser_scan_msg.angle_max = max(angles)
    laser_scan_msg.angle_increment = (laser_scan_msg.angle_max - laser_scan_msg.angle_min) / (len(angles) - 1)
    laser_scan_msg.time_increment = 0.0
    laser_scan_msg.scan_time = 10
    laser_scan_msg.range_min = 0.15
    laser_scan_msg.range_max = 6
    publisher.publish(laser_scan_msg)
    
def amcl_callback(msg):

    position_x =  msg.pose.pose.position.x
    position_y = msg.pose.pose.position.y
    msg.data[0] = position_x
    msg.data[1] = position_y


if __name__ == "__main__":

    rospy.init_node("SLAM General Node", anonymous=True)
    rospy.Subscriber("lidar_control", lidar_data, lidar_callback)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)

    
  
    