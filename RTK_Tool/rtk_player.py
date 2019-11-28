import csv
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


def utm_callback(point_data):
#

  num = 0;

  min_distance = 999
  cmd = Twist()
  for list_item in rtk_recoder_data:

    if math.sqrt((list_item[1]-point_data.x)**2+(list_item[2]-point_data.y)**2) < min_distance:

      cmd.linear.x = list_item[3]
      cmd.angular.z = list_item[4]
      num = list_item[0]
      min_distance = math.sqrt((list_item[1]-point_data.x)**2+(list_item[2]-point_data.y)**2)
      print(cmd.linear.x)
      print(cmd.angular.z)
    cmd_pub.publish(cmd)


 # print num


if __name__ == '__main__':
#
  csv_reader = csv.reader(open('./rtk_test.csv')) 
  rtk_recoder_data = []
  for csv_row in csv_reader:
    item = []
    item.append(csv_row[0])
    item.append(float(csv_row[1]))
    item.append(float(csv_row[2]))
    item.append(float(csv_row[3]))
    item.append(float(csv_row[4]))
    rtk_recoder_data.append(item)
    print("read")
  
  rospy.init_node('rtk_player', anonymous=False)
  cmd_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1);
  rospy.Subscriber("/RTK", Point, utm_callback)
  rospy.spin()



