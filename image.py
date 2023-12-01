#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import numpy as np
import cv2
import qrcode
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
class ImageSubscriber(Node) :
   def __init__(self) :
     super().__init__('image_subscriber')
     self.bridge = CvBridge() 
     self.marker_publisher = self.create_publisher(Marker, 'survivor_marker', 10)
     self.image_sub = self.create_subscription(
        Image, '/oakd/rgb/preview/image_raw', self.image_cb, qos_profile_sensor_data)
     self.image = []      

   def qr_detect(self, frame):
     decoded_obj = decode(frame)
     for obj in decoded_obj:
       print("qr code data : "+ obj.data.decode('utf-8'))
       print("survivor detected")
       #survivor_position = Point()
       #survivor_position.x = 1.0
       #survivor_position.y = 2.0
       #survivor_position.z = 0.0
       #self.publish_marker(survivor_position)

       points = obj.polygon
       if len(points) > 4:
         hull = cv2.convexHull(np.array(points, dtype=np.float32))
         cv2.polylines(frame,[hull],True,(0,255,0),2)
       else:
         cv2.polylines(frame, [np.array(points, dtype=np.int32)],True,(0,255,0),2)
     return frame 
     
   def image_cb(self, msg) :
     self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
     cv2.imshow('img', self.image)
     cv2.waitKey(1)
     qr_frame = self.qr_detect(self.image)
     cv2.imshow('qr img', qr_frame)
     cv2.waitKey(1)

   def publish_marker(self, position):
        #mark on a survivor's position
        marker = Marker()
        marker.header.frame_id = "map"  # Assuming your map frame_id is "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "survivor"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Marker size in the x direction
        marker.scale.y = 0.2  # Marker size in the y direction
        marker.scale.z = 0.2  # Marker size in the z direction
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0
        self.marker_publisher.publish(marker)

def main(args=None) :
  rclpy.init(args=args)
  node = ImageSubscriber()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__' :
  main()
