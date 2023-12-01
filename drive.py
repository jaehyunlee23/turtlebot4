import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from pyzbar.pyzbar import decode
from geometry_msgs.msg import Point
import threading
from datetime import datetime
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import geometry_msgs.msg
from scipy.spatial.transform import Rotation
class Node(Node):
    def __init__(self):
        super().__init__('pub_sub')
        self.cmd_pub_ = self.create_publisher(Twist,"/cmd_vel",10)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)
        self.collision_sub = self.create_subscription(DiagnosticArray,'/diagnostics',self.collison_cb,qos_profile_sensor_data)         
        self.lidar_sub = self.create_subscription(
        LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(
        Image, '/oakd/rgb/preview/image_raw', self.image_cb, qos_profile_sensor_data)
        self.bridge = CvBridge()
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_to_pose_cb_group = ReentrantCallbackGroup()
        self.image = []   
        self.Erange = 0.0
        self.Wrange = 0.0
        self.forward_scan = 0.0
        self.Fcount = 0
        self.initial_pose = PoseStamped()
        self.current_pose = PoseStamped()
        self.initial_pose_set = False  
        self.qr1 = False
        self.qr2 = False
        self.qr3 = False
        self.qr4 = False
        self.marker_id = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.rotation_time = self.get_clock().now()
        self.initial_time = self.get_clock().now().nanoseconds    
        self.timer = self.create_timer(40.0, self.rotate_callback)
        self.flag360 = False 
        self.start_time = self.get_clock().now().nanoseconds
    def rotate_callback(self):
      if(self.initial_pose_set == True): 
       self.flag360 = True
       self.start_time=self.get_clock().now().nanoseconds
    def lidar_cb(self, msg) :
     if(self.flag360 == True):
       mmsg = Twist()
       if(int(self.get_clock().now().nanoseconds)-int(self.start_time)<18000000000):
         print("360searching")
         mmsg.linear.x = 0.0
         mmsg.angular.z = 0.35
         self.cmd_pub_.publish(mmsg)
       else :
         mmsg.linear.x = 0.0
         mmsg.angular.z = 0.0
         self.cmd_pub_.publish(mmsg)
         self.rotation_time = self.get_clock().now()
         self.flag360 = False
     if(self.flag360 == False and self.initial_pose_set == True): 
      angle = 0.0
      mmsg = Twist()
      Lrange = 0.0
      Rrange = 0.0
      Lcount = 0
      Rcount = 0
      self.Fcount = 0
      self.forward_scan = 0.0
      self.Erange = 0.0
      self.Wrange = 0.0   
      if((int(self.get_clock().now().nanoseconds)-int(self.initial_time)) > 380000000000):
        print("timesup")
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"  
        goal_pose.pose = self.initial_pose.pose
        goal_pose.pose.position.x = self.initial_pose.pose.position.x
        goal_pose.pose.position.y = self.initial_pose.pose.position.y
        goal_pose.pose.position.z = self.initial_pose.pose.position.z
        self.send_navigation_goal(goal_pose)
      for value in msg.ranges:
       if 50<angle<130 :
         if(value < 2.0):
          self.forward_scan = self.forward_scan + value
          self.Fcount = self.Fcount + 1
         if value < 0.38:
              if angle > 90: 
               Lcount = Lcount + 1
              else:
               Rcount = Rcount + 1
       if angle<50 or angle>340 :
          if(value < 47.0):
           self.Erange = self.Erange + value
       if 130<angle<200 :
           if(value < 47.0):
            self.Wrange = self.Wrange + value
       angle = angle + 0.5

      print("Mission time(nanosec) : " + str((int(self.get_clock().now().nanoseconds)-int(self.initial_time)))) 
      if(int(self.get_clock().now().nanoseconds) - int(self.rotation_time.nanoseconds)) > 5000000000:
        if(int(self.get_clock().now().nanoseconds) - int(self.rotation_time.nanoseconds)) > 9000000000 :
          self.rotation_time = self.get_clock().now()
        if(self.Erange < self.Wrange):
          if(int(self.get_clock().now().nanoseconds) - int(self.rotation_time.nanoseconds)) < 9000000000 :
            mmsg.linear.x = 0.0
            mmsg.angular.z = 0.25
            self.cmd_pub_.publish(mmsg)
        else :
          if(int(self.get_clock().now().nanoseconds) - int(self.rotation_time.nanoseconds)) < 9000000000 :
            mmsg.linear.x = 0.0
            mmsg.angular.z = -0.25
            self.cmd_pub_.publish(mmsg)
      elif Lcount>4 or Rcount>4 :                                       
          if(Lcount > Rcount):
           mmsg.linear.x = 0.0
           mmsg.angular.z = -0.04
           self.cmd_pub_.publish(mmsg)
          else:
           mmsg.linear.x = 0.0
           mmsg.angular.z = 0.04
           self.cmd_pub_.publish(mmsg)
               
      else :
         self.rotation_time = self.get_clock().now()
         mmsg.linear.x = 0.12
         mmsg.angular.z = 0.0
         self.cmd_pub_.publish(mmsg)      
    def odom_cb(self,msg):
      if not self.initial_pose_set:
        self.initial_pose = msg.pose
        self.initial_pose_set = True
      self.current_pose = msg.pose
      #print("Initial position(x,y) : " + str(self.initial_pose.pose.position.x) +"  ,  "+ str(self.initial_pose.pose.position.y))
      #print("Current position(x,y) : " + str(self.current_pose.pose.position.x) +"  ,  "+ str(self.current_pose.pose.position.y))
      try:
              trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
      except (LookupException, ConnectivityException, ExtrapolationException):
              return
      marker = Marker()
      marker.header.frame_id = "map"
      marker.id = self.marker_id 
      self.marker_id += 1 
      marker.type = marker.SPHERE
      marker.action = marker.ADD
      position = geometry_msgs.msg.Point()
      position.x = trans.transform.translation.x
      position.y = trans.transform.translation.y
      position.z = trans.transform.translation.z
      marker.pose.position = position
      marker.pose.orientation = trans.transform.rotation
      marker.scale.x = 0.1
      marker.scale.y = 0.1
      marker.scale.z = 0.1
      marker.color.a = 1.0
      marker.color.r = 0.0
      marker.color.g = 1.0
      marker.color.b = 0.0
      self.marker_publisher.publish(marker)

    def image_cb(self,msg):
      if(self.initial_pose_set == True):
       self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
       cv2.imshow('img', self.image)
       cv2.waitKey(1)
       qr_frame = self.qr_detect(self.image)

    def send_navigation_goal(self, pose):
      goal_msg = NavigateToPose.Goal()
      goal_msg.pose = pose
      self.nav_to_pose_client.wait_for_server()
      self.nav_to_pose_client.send_goal_async(goal_msg)

    def qr_detect(self, frame): 
      decoded_obj = decode(frame)
      if(self.initial_pose_set==True): 
       if(self.qr1==True and self.qr2==True and self.qr3==True and self.qr4==True):
        print("returnhome")
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"  
        goal_pose.pose = self.initial_pose.pose
        goal_pose.pose.position.x = self.initial_pose.pose.position.x
        goal_pose.pose.position.y = self.initial_pose.pose.position.y
        goal_pose.pose.position.z = self.initial_pose.pose.position.z
        self.send_navigation_goal(goal_pose)
       else:
        for obj in decoded_obj:
         if(obj.data.decode('utf-8')=="1" and self.qr1==False) and self.Fcount != 0:
          print("qr code data : "+ obj.data.decode('utf-8'))
          try:
              trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
              rotation = trans.transform.rotation
              quat = [rotation.x, rotation.y, rotation.z, rotation.w]
              rotation = Rotation.from_quat(quat)
              euler_angles = rotation.as_euler('xyz')
          except (LookupException, ConnectivityException, ExtrapolationException):
              return
          marker = Marker()
          marker.header.frame_id = "map"
          marker.id = self.marker_id  
          self.marker_id += 1  
          marker.type = marker.SPHERE
          marker.action = marker.ADD
          position = geometry_msgs.msg.Point()
          forward_range = self.forward_scan/self.Fcount
          position.x = trans.transform.translation.x + forward_range*np.cos(euler_angles[2])
          position.y = trans.transform.translation.y + forward_range*np.sin(euler_angles[2])
          position.z = trans.transform.translation.z 
          marker.pose.position = position
          marker.pose.orientation = trans.transform.rotation
          marker.scale.x = 0.5
          marker.scale.y = 0.5
          marker.scale.z = 0.5
          marker.color.a = 1.0
          marker.color.r = 0.0
          marker.color.g = 0.0
          marker.color.b = 1.0
          self.marker_publisher.publish(marker)
          self.qr1 = True
          time.sleep(1)
          return frame
         if(obj.data.decode('utf-8')=="2" and self.qr2==False and self.Fcount != 0):
          print("qr code data : "+ obj.data.decode('utf-8'))
          print("qr code position(x,y) : "+str(self.current_pose.pose.position.x) +"  ,  " +str(self.current_pose.pose.position.y))
          try:
              trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
              rotation = trans.transform.rotation
              quat = [rotation.x, rotation.y, rotation.z, rotation.w]
              rotation = Rotation.from_quat(quat)
              euler_angles = rotation.as_euler('xyz')
          except (LookupException, ConnectivityException, ExtrapolationException):
              return

          marker = Marker()
          marker.header.frame_id = "map"
          marker.id = self.marker_id  
          self.marker_id += 1  
          marker.type = marker.SPHERE
          marker.action = marker.ADD
          position = geometry_msgs.msg.Point()
          forward_range = self.forward_scan/self.Fcount
          position.x = trans.transform.translation.x + forward_range*np.cos(euler_angles[2])
          position.y = trans.transform.translation.y + forward_range*np.sin(euler_angles[2])
          position.z = trans.transform.translation.z
          marker.pose.position = position
          marker.pose.orientation = trans.transform.rotation
          marker.scale.x = 0.5
          marker.scale.y = 0.5
          marker.scale.z = 0.5
          marker.color.a = 1.0
          marker.color.r = 0.0
          marker.color.g = 0.0
          marker.color.b = 1.0
          self.marker_publisher.publish(marker)
          self.qr2 = True
          time.sleep(1)
          return frame
         if(obj.data.decode('utf-8')=="3" and self.qr3==False and self.Fcount != 0):
          print("qr code data : "+ obj.data.decode('utf-8'))
          print("qr code position(x,y) : "+str(self.current_pose.pose.position.x) +"  ,  " +str(self.current_pose.pose.position.y))  
          try:
              trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
              rotation = trans.transform.rotation
              quat = [rotation.x, rotation.y, rotation.z, rotation.w]
              rotation = Rotation.from_quat(quat)
              euler_angles = rotation.as_euler('xyz')
          except (LookupException, ConnectivityException, ExtrapolationException):
              return
          marker = Marker()
          marker.header.frame_id = "map"
          marker.id = self.marker_id 
          self.marker_id += 1  
          marker.type = marker.SPHERE
          marker.action = marker.ADD
          position = geometry_msgs.msg.Point()
          forward_range = self.forward_scan/self.Fcount
          position.x = trans.transform.translation.x + forward_range*np.cos(euler_angles[2])
          position.y = trans.transform.translation.y + forward_range*np.sin(euler_angles[2])
          position.z = trans.transform.translation.z
          marker.pose.position = position
          marker.pose.orientation = trans.transform.rotation
          marker.scale.x = 0.5
          marker.scale.y = 0.5
          marker.scale.z = 0.5
          marker.color.a = 1.0
          marker.color.r = 0.0
          marker.color.g = 0.0
          marker.color.b = 1.0
          self.marker_publisher.publish(marker)
          self.qr3 = True
          time.sleep(1)
          return frame
         if(obj.data.decode('utf-8')=="4" and self.qr4==False and self.Fcount != 0):
          print("qr code data : "+ obj.data.decode('utf-8'))  
          print("qr code position(x,y) : "+str(self.current_pose.pose.position.x) +"  ,  " +str(self.current_pose.pose.position.y))
          try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rotation = trans.transform.rotation
            quat = [rotation.x, rotation.y, rotation.z, rotation.w]
            rotation = Rotation.from_quat(quat)
            euler_angles = rotation.as_euler('xyz')
          except (LookupException, ConnectivityException, ExtrapolationException):
              return

          marker = Marker()
          marker.header.frame_id = "map"
          marker.id = self.marker_id 
          self.marker_id += 1 
          marker.type = marker.SPHERE
          marker.action = marker.ADD
          position = geometry_msgs.msg.Point()
          forward_range = self.forward_scan/self.Fcount
          position.x = trans.transform.translation.x + forward_range*np.cos(euler_angles[2])
          position.y = trans.transform.translation.y + forward_range*np.sin(euler_angles[2])
          position.z = trans.transform.translation.z
          marker.pose.position = position
          marker.pose.orientation = trans.transform.rotation
          marker.scale.x = 0.5
          marker.scale.y = 0.5
          marker.scale.z = 0.5
          marker.color.a = 1.0
          marker.color.r = 0.0
          marker.color.g = 0.0
          marker.color.b = 1.0
          self.marker_publisher.publish(marker)
          self.qr4 = True
          time.sleep(1)
          return frame   
      return frame 
    def collison_cb(self,msg):
      for status in msg.status:
        if status.name == 'turtlebot4_diagnostics: Hazard Detections':
            hazard_values = eval(status.values[0].value)
            if 'BUMP' in hazard_values:
                print("Front bump hit something. Take action!")
                time.sleep(3)
                mmsg = Twist()
                if(self.Erange<self.Wrange):
                  mmsg.linear.x = 0.0
                  mmsg.angular.z = 0.5
                  self.cmd_pub_.publish(mmsg)
                else :
                  mmsg.linear.x = 0.0
                  mmsg.angular.z = -0.5
                  self.cmd_pub_.publish(mmsg)
def main(args=None):
    rclpy.init(args=args)
    node = Node()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()