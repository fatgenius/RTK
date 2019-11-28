import csv
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
global chassis_received
global g_chassis_twist

def move_fedback_callback(twist_data):
# 50hz 

  global chassis_received
  global g_chassis_twist

  

  g_chassis_twist = twist_data
  chassis_received = True


def utm_callback(point_data):
# 10hz

  global chassis_received
  global g_chassis_twist
  global sequence_number

  #if not chassis_received:
  #  return


  record_item = []
  record_item.append(sequence_number)
  sequence_number += 1

  record_item.append(point_data.x)
  record_item.append(point_data.y)
  record_item.append(g_chassis_twist.linear.x)
  record_item.append(g_chassis_twist.angular.z)

  csv_writer.writerow(record_item)
  print sequence_number

  

if __name__ == '__main__':
#
  csv_writer = csv.writer(open('./rtk_test.csv', 'w+'))
  chassis_received = False
  g_chassis_twist = Twist()
  sequence_number = 0

  rospy.init_node('rtk_player', anonymous=False)
  rospy.Subscriber("/RTK", Point, utm_callback)
  rospy.Subscriber("/scout_robot/odom ", Odometry , move_fedback_callback)
 
  rospy.spin()
