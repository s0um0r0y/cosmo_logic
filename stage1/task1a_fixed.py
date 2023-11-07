#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ eYRC#CL#2277 ]
# Author List:		[ Soumo Roy,Joel J Viju,Rohan Raj,Anirudh Singareddy ]
# Filename:		    task1a.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


import rclpy
import sys
import cv2
import math
import tf2_ros
from tf2_ros import TransformBroadcaster
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image

def calculate_rectangle_area(coordinates):
    if len(coordinates)!=4:
        raise ValueError("4 sets of (x, y) coordinates should be present")

    x1,y1=coordinates[0]
    x2,y2=coordinates[1]
    x3,y3=coordinates[2]
    x4,y4=coordinates[3]

    width=np.sqrt((x1-x2)**2+(y1-y2)**2)
    height=np.sqrt((x2-x3)**2+(y2-y3)**2)

    area=width*height

    return area,width

def detect_aruco(image):
     
    aruco_area_threshold = 1500
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])
    size_of_aruco_m = 0.15

    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
    rvec_list = []
    tvec_list = []

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters =  cv2.aruco.DetectorParameters()
    
    corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(f"Marker id is {marker_ids}")

    for i in range(len(marker_ids[0])):
        
        coordinates = corners[i][0]
        area, width = calculate_rectangle_area(coordinates)

        if area >= aruco_area_threshold:
            center_x = np.mean(coordinates[:, 0])
            center_y = np.mean(coordinates[:, 1])
            center_aruco_list.append((center_x, center_y))

            rvec,tvec,_ = cv2.aruco.estimatePoseSingleMarkers(corners[i], size_of_aruco_m, cam_mat, dist_mat)
            rvec_list.append(rvec)
            tvec_list.append(tvec)
            
            tvec = tvec.squeeze(1)
            distance_from_rgb = tvec[0,2]
            
            distance_from_rgb_list.append(distance_from_rgb)
            angle_aruco_list.append(rvec)
            width_aruco_list.append(width)
            ids.append(marker_ids[i][0])

            cv2.aruco.drawDetectedMarkers(image,corners)
            cv2.drawFrameAxes(image,cam_mat,dist_mat,rvec,tvec,size_of_aruco_m)

    return center_aruco_list,distance_from_rgb_list,angle_aruco_list,width_aruco_list,ids,rvec_list,tvec_list

class aruco_tf(Node):

    def __init__(self):
        super().__init__('aruco_tf_publisher')                                       

        self.color_cam_sub = self.create_subscription(Image,'/camera/color/image_raw',self.colorimagecb,10)
        self.depth_cam_sub = self.create_subscription(Image,'/camera/aligned_depth_to_color/image_raw',self.depthimagecb,10)

        image_processing_rate = 0.4                                                    
        self.bridge = CvBridge()                                                        
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    
        self.timer = self.create_timer(image_processing_rate, self.process_image)     
        
        self.cv_image = None                                                           
        self.depth_image = None                                                         


    def depthimagecb(self, data):
        try:
            self.depth_image=self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error("failed to convert depth image: %s" % str(e))
            return


    def colorimagecb(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error("failed to convert color image: %s" % str(e))
            return


    def process_image(self):
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids, rvec_list, tvec_list = detect_aruco(self.cv_image)

        for i in range(len(ids)):
            marker_id = ids[i]
            distance_from_rgb=distance_from_rgb_list[i]       
            angle_aruco=angle_aruco_list[i][0][0]
        
            from_opencv=R.from_rotvec(angle_aruco).as_euler('xyz',degrees=True)
            to_ros=np.array([-from_opencv[0]-90,-from_opencv[1],from_opencv[2]])
            p=R.from_euler('yzx',to_ros, degrees=True)
            q=R.from_euler('z',90,degrees=True)
            r=p*q
            quat = r.as_quat()
            
            distance_from_rgb = distance_from_rgb/1000

            cX, cY = center_aruco_list[i][:2]
            #depth = math.sqrt(pow(cX,2)+pow(cY,2))
            x = distance_from_rgb*(sizeCamX-cX-centerCamX)/focalX
            y = distance_from_rgb*(sizeCamY-cY-centerCamY)/focalY
            z = distance_from_rgb

            marker_frame_vec = np.array([x,y,z])
            print(f"shape of{marker_frame_vec.shape}")

            angle_y_rad = (math.pi)/2
            angle_z_rad = -(math.pi)/2

            rotation_matrix_y = np.array([[np.cos(angle_y_rad), 0, np.sin(angle_y_rad)],
                              [0, 1, 0],
                              [-np.sin(angle_y_rad), 0, np.cos(angle_y_rad)]])
            
            rotation_matrix_z = np.array([[np.cos(angle_z_rad), -np.sin(angle_z_rad), 0],
                              [np.sin(angle_z_rad), np.cos(angle_z_rad), 0],
                              [0, 0, 1]])
            

            rvec = rvec_list[i]
            tvec = tvec_list[i]

            rotation_matrix_from_rvec = R.from_rotvec(rvec.reshape(3,))
            rotation_matrix_from_rvec = rotation_matrix_from_rvec.as_matrix()
           
            final_vector=np.dot(rotation_matrix_from_rvec,marker_frame_vec)
            cam_frame_vector=tvec+final_vector
            cam_frame_vector=cam_frame_vector.reshape(-1, 1)

            rotated_cam_frame_vector=np.dot(rotation_matrix_y,np.dot(rotation_matrix_z,cam_frame_vector))
            cv2.circle(self.cv_image,(int(cX),int(cY)),5,(0,0,255),-1)

            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = 'camera_link'
            transform_stamped.child_frame_id = 'cam_{}'.format(marker_id)
            transform_stamped.transform.translation.x = float(rotated_cam_frame_vector[0])
            transform_stamped.transform.translation.y = float(rotated_cam_frame_vector[1])
            transform_stamped.transform.translation.z = float(rotated_cam_frame_vector[2])
            transform_stamped.transform.rotation.w =float(quat[3])
            transform_stamped.transform.rotation.x =float(quat[0])
            transform_stamped.transform.rotation.y = float(quat[1])
            transform_stamped.transform.rotation.z = float(quat[2])
            
            self.br.sendTransform(transform_stamped)
            # print(distance_from_rgb)

            try:
                base_to_camera = self.tf_buffer.lookup_transform('base_link', 'cam_{}'.format(marker_id), rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
           

            # print(transform_stamped.transform.rotation)
            print(base_to_camera.transform.rotation)

            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = 'base_link'
            transform_stamped.child_frame_id = 'obj_{}'.format(marker_id)
            transform_stamped.transform.translation.x = base_to_camera.transform.translation.x
            transform_stamped.transform.translation.y = base_to_camera.transform.translation.y
            transform_stamped.transform.translation.z = base_to_camera.transform.translation.z
            transform_stamped.transform.rotation.w = base_to_camera.transform.rotation.w
            transform_stamped.transform.rotation.x = base_to_camera.transform.rotation.x
            transform_stamped.transform.rotation.y = base_to_camera.transform.rotation.y
            transform_stamped.transform.rotation.z = base_to_camera.transform.rotation.z
            self.br.sendTransform(transform_stamped)

        cv2.imshow('Aruco Markers', self.cv_image)
        cv2.waitKey(1) 

def main():

    rclpy.init(args=sys.argv)                                      

    node = rclpy.create_node('aruco_tf_process')                   

    node.get_logger().info('Node created: Aruco tf process')        

    aruco_tf_class = aruco_tf()                                     

    rclpy.spin(aruco_tf_class)                                      

    aruco_tf_class.destroy_node()                                   

    rclpy.shutdown()                                                


if __name__ == '__main__':
    main()