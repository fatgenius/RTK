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
#s.write('B5 62 06 08 06 00 01 00 00 01 00 7A 12')
#
print("ok")

global angle

def talker():
     pub = rospy.Publisher('utm', Point, queue_size=10)
     rospy.init_node('talker', anonymous=True)
     rate = rospy.Rate(100) # 10hz
     count=0
     global sequence_numbery

     while not rospy.is_shutdown():
        while True:
            line =str(str(s.readline())[0:])
            if line.startswith('$GNTXT'):
              a = line.split(',')
              angle = a[4]
              print(line)
 #              print("angle is ok")

            if line.startswith('$GNGGA'):
              record_item = []
              #sequence_number += 1
              msg =pynmea2.parse(line)
              lat=msg.latitude
              lgi=msg.longitude
              record_item.append(sequence_number)
              record_item.append(lgi)
              record_item.append(lat)
              


              utm=Proj(proj='utm',zone=48,ellps='WGS84')
              x,y=utm(lgi, lat)
              utm_point = Twist()
              #msg = geom_msg.Pose()
              utm_point.x =x               
              utm_point.y =y
              utm_point.z = angle
              print (utm_point)

              
              record_item.append(x)
              record_item.append(y)
              
              csv_writer.writerow(record_item)
              print sequence_number
              br = tf.TransformBroadcaster()
              br.sendTransform((x, y, 0),
              tf.transformations.quaternion_from_euler(0, 0, angle),rospy.Time.now(),"odom","world")

             #u.linear.x= x
             #  u.linear.y= y

              rospy.loginfo(utm_point)
              pub.publish(utm_point)
            #rate.sleep()

	      #print(count)

       




if __name__ == '__main__':

  csv_writer = csv.writer(open('./GPS_test.csv', 'w+'))
  sequence_number=1
  try:
      talker()
  except rospy.ROSInterruptException:
      pass
