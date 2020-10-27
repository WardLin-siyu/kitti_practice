#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge #opencv -> ros
import tf
FRAME_ID = 'map'
DETECTION_COLOR_MAP = {'Car': (255,255,0), 'Pedestrian': (0, 226, 255), 'Cyclist': (141, 40, 255)} # color for detection, in format bgr
LIFETIME = 1.0 # 1/rate

LINES = [[0, 1], [1, 2], [2, 3], [3, 0]] # lower face
LINES+= [[4, 5], [5, 6], [6, 7], [7, 4]] # upper face
LINES+= [[4, 0], [5, 1], [6, 2], [7, 3]] # connect lower face and upper face
LINES+= [[4, 1], [5, 0]] # front face
EGOCAR = np.array([[2.15, 0.9, -1.73], [2.15, -0.9, -1.73], [-1.95, -0.9, -1.73], [-1.95, 0.9, -1.73], 
                	[2.15, 0.9, -0.23], [2.15, -0.9, -0.23], [-1.95, -0.9, -0.23], [-1.95, 0.9, -0.23]])

def publish_camera(cam_pub, bridge , image, boxes, types):
	for typ, box in zip(types, boxes):
		top_left = int(box[0]), int(box[1])
		bottom_right = int(box[2]), int(box[3])
		cv2.rectangle(image, top_left, bottom_right, DETECTION_COLOR_MAP[typ], 2)
	cam_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))#opencv -> ros  , usignint 8bit

def publish_3dbox(box3d_pub, corners_3d_velos, types, track_ids):
	marker_array = MarkerArray()
	
	for track_id, corners_3d_velo in enumerate(corners_3d_velos):
		marker = Marker()
		marker.header.frame_id = FRAME_ID
		marker.header.stamp = rospy.Time.now()
		marker.id = track_id
		marker.action = Marker.ADD
		marker.lifetime = rospy.Duration(LIFETIME)
		marker.type = Marker.LINE_LIST
		b, g, r = DETECTION_COLOR_MAP[types[track_id]]
		marker.color.r = r/255.0
		marker.color.g = g/255.0
		marker.color.b = b/255.0
		marker.color.a = 1.0
		marker.scale.x = 0.1
		marker.points = []
		for l in LINES:
			p1 = corners_3d_velo[l[0]]
			marker.points.append(Point(p1[0], p1[1], p1[2]))
			p2 = corners_3d_velo[l[1]]
			marker.points.append(Point(p2[0], p2[1], p2[2]))
		marker_array.markers.append(marker)

		text_marker = Marker()
		text_marker.header.frame_id = FRAME_ID
		text_marker.header.stamp = rospy.Time.now()
		text_marker.id = track_id + 1000 #avoid rpeat same id
		text_marker.action = Marker.ADD
		text_marker.lifetime = rospy.Duration(LIFETIME)
		text_marker.type = Marker.TEXT_VIEW_FACING
		p4 = corners_3d_velo[4]# upper front left corner
		text_marker.pose.position.x = p4[0]
		text_marker.pose.position.y = p4[1]
		text_marker.pose.position.z = p4[2] + 0.5
		text_marker.text = str(track_ids[track_id])

		text_marker.scale.x = 1
		text_marker.scale.y = 1
		text_marker.scale.z = 1

		b, g, r = DETECTION_COLOR_MAP[types[track_id]]
		text_marker.color.r = r/255.0
		text_marker.color.g = g/255.0
		text_marker.color.b = b/255.0
		text_marker.color.a = 1.0
		marker_array.markers.append(text_marker)
	
	marker = Marker()
	marker.header.frame_id = FRAME_ID
	marker.header.stamp = rospy.Time.now()
	marker.id = -1
	marker.action = Marker.ADD
	marker.lifetime = rospy.Duration(LIFETIME)
	marker.type = Marker.LINE_LIST

	marker.color.r = 0
	marker.color.g = 1
	marker.color.b = 1
	marker.color.a = 1.0
	marker.scale.x = 0.1
	marker.points = []
	for l in LINES:
		print(l)
		p1 = EGOCAR[l[0]]
		print(p1)
		marker.points.append(Point(p1[0], p1[1], p1[2]))
		p2 = EGOCAR[l[1]]
		print(p2)
		marker.points.append(Point(p2[0], p2[1], p2[2]))

	marker_array.markers.append(marker)
	
	text_marker = Marker()
	text_marker.header.frame_id = FRAME_ID
	text_marker.header.stamp = rospy.Time.now()
	text_marker.id = -2
	text_marker.action = Marker.ADD
	text_marker.lifetime = rospy.Duration(LIFETIME)
	text_marker.type = Marker.TEXT_VIEW_FACING
	p4 = EGOCAR[4]# upper front left corner
	text_marker.pose.position.x = p4[0]
	text_marker.pose.position.y = p4[1]
	text_marker.pose.position.z = p4[2] + 0.5
	text_marker.text = 'EGOCAR'
	text_marker.scale.x = 1
	text_marker.scale.y = 1
	text_marker.scale.z = 1

	b, g, r = DETECTION_COLOR_MAP['Car']
	text_marker.color.r = r/255.0
	text_marker.color.g = g/255.0
	text_marker.color.b = b/255.0
	text_marker.color.a = 1.0
	marker_array.markers.append(text_marker)

	box3d_pub.publish(marker_array)

def publish_point_cloud(pcl_pub, point_cloud):
	header = Header()
	header.stamp = rospy.Time.now()
	header.frame_id = 'map' #current coordinate name
	pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))

def publish_ego_car(ego_car_pub):
	"""
	Publish lef/right 45 degree FOV line
	use Marker  http://wiki.ros.org/rviz/DisplayTypes/Marker
	"""
##camera FOV
	maker_array = MarkerArray()
	marker = Marker()
	marker.header.frame_id = FRAME_ID
	marker.header.stamp = rospy.Time.now()

	marker.id = 0 #must be different 
	marker.action = Marker.ADD
	marker.lifetime = rospy.Duration() #forever show in screen
	marker.type = Marker.LINE_STRIP #you can change line style in here 

	marker.color.r = 0.0 # 010 = green
	marker.color.g = 1.0 #
	marker.color.b = 0.0 #
	marker.color.a = 1.0 # Transparent 0 ~ 1 opaque  
	marker.scale.x = 0.2 # Thickness

	marker.points = []  #Connection point coordinates
	marker.points.append(Point(10, -10, 0))
	marker.points.append(Point(0, 0, 0))
	marker.points.append(Point(10, 10, 0))

	maker_array.markers.append(marker)

##car model
	mesh_marker = Marker()
	mesh_marker.header.frame_id = FRAME_ID
	mesh_marker.header.stamp = rospy.Time.now()

	mesh_marker.id = -1 
	mesh_marker.lifetime = rospy.Duration() #forever show in screen
	mesh_marker.type = Marker.MESH_RESOURCE #you can change line style in here 
	mesh_marker.mesh_resource = "package://agv_object_dection/Land Rover Range Rover Sport HSE 2010.dae"

	mesh_marker.pose.position.x = 0.0  # set position
	mesh_marker.pose.position.y = 0.0
	mesh_marker.pose.position.z = -1.73  #lidar = 0 0 0 car = 0 0 -1.73

	q = tf.transformations.quaternion_from_euler(np.pi/2, 0, np.pi) #ratation value
	mesh_marker.pose.orientation.x = q[0]
	mesh_marker.pose.orientation.y = q[1]
	mesh_marker.pose.orientation.z = q[2]
	mesh_marker.pose.orientation.w = q[3]

	mesh_marker.color.r = 1.0 # 
	mesh_marker.color.g = 1.0 #
	mesh_marker.color.b = 1.0 #
	mesh_marker.color.a = 1.0 # Transparent 0 ~ 1 opaque 

	mesh_marker.scale.x = 0.9 # Thickness
	mesh_marker.scale.y = 0.9 # Thickness
	mesh_marker.scale.z = 0.9 # Thickness

	maker_array.markers.append(mesh_marker)
	ego_car_pub.publish(maker_array)

def publish_imu(imu_pub, imu_data):
	imu = Imu()
	imu.header.frame_id = FRAME_ID
	imu.header.stamp = rospy.Time.now()

	q = tf.transformations.quaternion_from_euler(float(imu_data.roll), 0, np.pi) #ratation value
	imu.orientation.x = q[0]
	imu.orientation.y = q[1]
	imu.orientation.z = q[2]
	imu.orientation.w = q[3]
	imu.linear_acceleration.x = imu_data.af
	imu.linear_acceleration.y = imu_data.al
	imu.linear_acceleration.z = imu_data.au
	imu.angular_velocity.x = imu_data.wf
	imu.angular_velocity.y = imu_data.wl
	imu.angular_velocity.z = imu_data.wu

	imu_pub.publish(imu)

def publish_gps(gps_pub, imu_data):
	gps = NavSatFix()
	gps.header.frame_id = FRAME_ID
	gps.header.stamp = rospy.Time.now()
	gps.latitude = imu_data.lat 
	gps.longitude = imu_data.lon
	gps.altitude = imu_data.alt
	
	gps_pub.publish(gps)

def publish_loc(loc_pub, tracker, centers):
	marker_array = MarkerArray()

	for track_id in centers:
		marker = Marker()
		marker.header.frame_id = FRAME_ID
		marker.header.stamp = rospy.Time.now()


		marker.action = Marker.ADD
		marker.lifetime = rospy.Duration(LIFETIME)
		marker.type = Marker.LINE_STRIP
		marker.id = track_id
		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.color.a = 1.0
		marker.scale.x = 0.2

		marker.points = []
		for p in tracker[track_id].locations:
			marker.points.append(Point(p[0], p[1], 0))

		marker_array.markers.append(marker)
	loc_pub.publish(marker_array)
	rospy.loginfo("locations published")


def publish_dist(dist_pub, minPQDs):
	marker_array = MarkerArray()
	for i, (minP, minQ, minD) in enumerate(minPQDs):
		marker = Marker()
		marker.header.frame_id = FRAME_ID
		marker.header.stamp = rospy.Time.now()

		marker.action = Marker.ADD
		marker.lifetime = rospy.Duration(LIFETIME)
		marker.type = Marker.LINE_STRIP
		marker.id = i
		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 1.0
		marker.color.a = 0.5
		marker.scale.x = 0.1

		marker.points = []
		marker.points.append(Point(minP[0], minP[1], 0))
		marker.points.append(Point(minQ[0], minQ[1], 0))
		marker_array.markers.append(marker)
	
		text_marker = Marker()
		text_marker.header.frame_id = FRAME_ID
		text_marker.header.stamp = rospy.Time.now()
		text_marker.id = i + 1000 #avoid rpeat same id
		text_marker.action = Marker.ADD
		text_marker.lifetime = rospy.Duration(LIFETIME)
		text_marker.type = Marker.TEXT_VIEW_FACING

		p = (minP + minQ) / 2.0
		text_marker.pose.position.x = p[0]
		text_marker.pose.position.y = p[1]
		text_marker.pose.position.z = 0.0

		text_marker.text = '%.2f'%minD

		text_marker.scale.x = 1
		text_marker.scale.y = 1
		text_marker.scale.z = 1

		text_marker.color.r = 1
		text_marker.color.g = 1
		text_marker.color.b = 1
		text_marker.color.a = 0.8
		marker_array.markers.append(text_marker)

	dist_pub.publish(marker_array)
	rospy.loginfo("distance  published")


