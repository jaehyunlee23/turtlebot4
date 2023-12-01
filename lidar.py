#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2


class LiDARSubscriber(Node) :
   def __init__(self) :
     super().__init__('lidar_subscriber')
     self.lidar_sub = self.create_subscription(
        LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data)

   def lidar_cb(self, msg) :
     #self.lidar = np.array(self.lidar)
     #self.lidar = np.resize(self.lidar, (20, 720))
     #cv2.imshow('img', self.lidar)
     #cv2.waitKey(1)
      angle = 0
      #mmsg = Twist()
      Lrange = 0
      Rrange = 0
      Lcount = 0
      Rcount = 0
      Fcount = 0
      Erange = 0.0
      Wrange = 0.0
      forward_scan = 0.0
      for value in msg.ranges:
       if 70<angle<190 :
         if(value < 47.0):
          forward_scan = forward_scan + value
          Fcount = Fcount + 1
         print(str(angle)+'  :  '+str(value))
         if value < 0.38:
              if angle > 130: 
               Lcount = Lcount + 1
              else:
               Rcount = Rcount + 1
       if 0<angle<70 :
          if(value < 47.0):
           self.Erange = self.Erange + value
       if 190<angle<260 :
           if(value < 47.0):
            self.Wrange = self.Wrange + value
       angle = angle + 0.5   
      
      print("LCount : " + str(Lcount))
      print("RCount : " + str(Rcount))
      print("forward_scan_averagerange(m) : " + str(forward_scan/Fcount))
      print("Rrange : " + str(Rrange))
      print("Lrange : " + str(Lrange))
      print("Erange : " + str(Erange))
      print("Wrange : " + str(Wrange))
      if (Lcount + Rcount) > 10 :                                       
        if(Lcount > Rcount):
          print("rotate2right")
          # mmsg.angular.z = -0.1
          # self.cmd_pub_.publish(mmsg)
        else:
           print("rotate2left")
           #mmsg.angular.z = 0.1
           #self.cmd_pub_.publish(mmsg)
        #if stucked at corner where BOTH L,R have obstacle close to robot => compare the distance           
      else :
       print("go str8ahed")
       #mmsg.linear.x = 0.1
       #self.cmd_pub_.publish(mmsg)
       
        
     
def main(args=None) :
  rclpy.init(args=args)
  node = LiDARSubscriber()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__' :
  main()
