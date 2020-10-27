#!/usr/bin/env python
import cv2
import os
import numpy as np 
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2#在rospy中影像傳輸的格式
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge #opencv -> ros

DATA_PATH = '/home/rtioms/auto_machine_with_OD/2011_09_26/2011_09_26_drive_0005_sync/'

if __name__ == '__main__':
	frame = 0
	rospy.init_node('kitti_node',anonymous=True) # anonymous=True 表示有許多名稱都可以叫talker
	cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
	pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
	bridge = CvBridge() #img為opencv的格式 必須透過cvbridge 轉成ROS可用的格式


	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
			img = cv2.imread(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame)) #串接兩個路徑  %010d表示有10個數字 %0表示目前要讀取的值
			point_cloud = np.fromfile(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame), dtype=np.float32).reshape(-1, 4)) #point_cloud 為n*4的矩陣
			cam_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))#opencv -> ros  , usignint 8bit
			header = Header()
			header.stamp = rospy.Time.now()
			header.frame_id = 'map' #current coordinate name
			pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))#此函數讀取一個點於的xyz座標(float32型態),不接受反射率資料,!![全部的點, 每個點的前三行] , 目的為將point轉換成PointCloud2
			rospy.loginfo("published")
			rate.sleep()
			frame += 1
			frame %= 154
