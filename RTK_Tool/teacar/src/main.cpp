#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "can.h"

void t3_move_callback(const geometry_msgs::Twist &msg)
{
    struct car_speed speed;

    speed.linSpeed = msg.linear.x;
    speed.angSpeed = msg.angular.z;
    car_spdSetCurrent(&speed);
    ROS_INFO("linSpd %f angSpd %f", speed.linSpeed, speed.angSpeed);
}

int main(int argc, char *argv[])
{
    struct can_device device;

    ros::init(argc, argv, "teacar");
    
    ros::NodeHandle nh;

    device.devType = 3;
    device.devID = 0;
    device.devChN = 0;
    car_task(&device);

    ros::Subscriber topic_t3_move = nh.subscribe("/teacar/t3/move", 1000, t3_move_callback);
 
    ros::Publisher topic_t3_move_feed = nh.advertise<geometry_msgs::Twist>("/teacar/t3/move_feed", 1000);
    ros::Rate rate(100);

    
    while (ros::ok())
    {
        if (car_spdIsUpdate())
        {
            struct car_speed speed;
            geometry_msgs::Twist moveFeed;

            car_spdGetCurrent(&speed);

            moveFeed.linear.x = speed.linSpeed;
            moveFeed.angular.z = speed.angSpeed;
            topic_t3_move_feed.publish(moveFeed);
        }
        ros::spinOnce();
        
        rate.sleep();
    }

    return 0;
}
