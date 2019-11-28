#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""sensor_msgs.msg in our RTK"""
import rospy
import rosunit
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
import utm
import serial
import pynmea2
import rospy
import csv
from std_msgs.msg import String
from pyproj import Proj 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import tf


s= serial.Serial('/dev/rtk',115200)
#s = open('/home/tealligence/Desktop/rtk_guo/3-2.txt', "r")	



global angle
line2 =str(str(s.readline())[0:])



def txt():
	while True:
		for line2 in s:
			if line2.startswith('$GNTXT'):
		  		b = line2.split(',')
		  		angle=float(b[4])
		  		#print("ok_heading")
		  		#print(angle)
		  		return (angle) 

def talker():
     pub = rospy.Publisher('/RTK',Point, queue_size=10)
     rospy.init_node('RUN', anonymous=True)
     rate = rospy.Rate(100) # 10hz
     count=0
     #print("ok")
     global sequence_number
     global z

     while not rospy.is_shutdown():
        while True:
            line =str(str(s.readline())[0:])

 #              print("angle is ok")

            if line.startswith('$GNGGA'):
              #print(line)
              a= line.split(',')
              RTK =(a[6])
              #print(RTK)	
              record_item = []
              sequence_number += 1
              msg =pynmea2.parse(line)
              lat=msg.latitude
              lgi=msg.longitude
              navsat =NavSatFix()
              navsat.status = 0
              navsat.header.seq = 1
              navsat.header.stamp = rospy.Time.now()
              navsat.position_covariance_type = 2
#              navsat.STATUS = 1
              navsat.header.frame_id ='base_link'
              navsat.latitude =lat              
              navsat.longitude =lgi
              navsat.altitude =txt()
              print (navsat)


              utm=Proj(proj='utm',zone=48,ellps='WGS84')
              x,y=utm(lgi, lat)

              utm_point = Point()
              #msg = geom_msg.Pose()
              utm_point.x =x               
              utm_point.y =y
              #utm_point.angular.z = txt()
            
              # br = tf.TransformBroadcaster()
              # br.sendTransform((x, y, 0),
              # tf.transformations.quaternion_from_euler(0, 0, txt()),rospy.Time.now(),"base_link","odom")

              #rospy.loginfo(utm_point)
              pub.publish(utm_point)
              ##print(utm)


       




if __name__ == '__main__':

  csv_writer = csv.writer(open('./GPS_test.csv', 'w+'))
  sequence_number=1
  try:
      talker()
  except rospy.ROSInterruptException:
      pass
