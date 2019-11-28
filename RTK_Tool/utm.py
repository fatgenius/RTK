#!/usr/bin/env python
import rospy
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


s= serial.Serial('/dev/ttyUSB0',115200)
#s = open('/home/tealligence/Desktop/rtk_guo/3-2.txt', "r")	



global angle
global sequence_number
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
     pub = rospy.Publisher('utm', Twist, queue_size=10)
     rospy.init_node('talker', anonymous=True)
     rate = rospy.Rate(100) # 10hz
     count=0
     #print("ok")
     #global sequence_number

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
              
              msg =pynmea2.parse(line)
              lat=msg.latitude
              lgi=msg.longitude
              record_item.append(sequence_number)
              record_item.append(lgi)
              record_item.append(lat)
              


              utm=Proj(proj='utm',zone=48,ellps='WGS84')
              x,y=utm(lgi, lat)
             
              print(x)
              utm_point = Point()
              #msg = geom_msg.Pose()
              utm_point.linear.x =x               
              utm_point.linear.y =y
#              utm_point.angular.z = txt()
              utm_point.angular.z =z
              print (utm_point)

              #print sequence_number

#              br = tf.TransformBroadcaster()
#              br.sendTransform((x, y, 0),
#              tf.transformations.quaternion_from_euler(0, 0, txt()),rospy.Time.now(),"base_link","odom")


              #rospy.loginfo(utm_point)
              pub.publish(utm_point)


       




if __name__ == '__main__':

#  csv_writer = csv.writer(open('./GPS_test.csv', 'w+'))
#  sequence_number=1
  try:
      talker()
  except rospy.ROSInterruptException:
      pass
