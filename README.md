# RTK
This project is about how I use RTK to make a AGV autopilot.


# How I do it
- recording RTK data 
- recording the AGV Chassis odometry data
- replay the odometry info to the AGV based on the the cloest RTK info

# The accuracy of this project
This project accuracy is highly rely on the RTK. when I deploy the Base station in the crowed city it would not work well
but when I deployed this system in the countryside of china. It works much more better

# system content  
- The teacar folder is C++ code about how I get data from the AGV Chassis through the CAN card and made a agv odometry.
  All those code is based on ROS.
- The utm.py is about get data from RTK. Since RTK only provid me the NEMA format data. I have to format it with UTM format.
  Which is easy to use.
- The rtk_recorder is the python script which I used to record the utm data and odometry in the same time
- The rtk_player is sort of make the AGV running by find the closet UTM~s odometry data and seng it back 


# hardwave
- can card is used by zlg can mini
- RTK is a SHENZHEN provider
![](picture/rtk.jpg)