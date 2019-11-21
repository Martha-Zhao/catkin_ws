## This is the documation for my_robot package
-------------------------

### 2019-11-19

|node | topic name| topic type |contain files|discription|
|---------|---------|------------|---------|---------|
|imu_node | /imu_data | sensor_msgs::Imu |imu_filter.cpp include/my_robot/imu_filter.h | imu driver and puslish imu_Data without UKF |
|imu_node| /imu_filter | my_robot::imu_filter_msg | same as upon | imu_data filtered|
|laser_node | /laser_scan | sensor_msgs::LaserScan |laser_driver.cpp laser_node.cpp include/my_robot/laser_driver.h include/my_robot/laser_node.h|laser driver and publish laser_Scan |

--------------------------
*PLANNING NODE*
|   NODE    | PUBLIC TOPIC |           TYPE          |            NOTICE           |
|-----------|-----------|-------------------------|-----------------------------|
|  imu_node |  imu_data |    sensor_msgs::Imu     |    origin imu_data   |
|  imu_node | imu_filter_data| my_robot::imu_filter_msg| angle_x angle_y angle_z linear_x linear_y linear_z|
| laser_node| laser_scan|  sensor_msgs::LaserScan | origin laser_data|

#### ros plot
rosrun rqt_plot rqt_plot

--------------------------
**check** \
rosrun my_robot imu_node laser_node\
rostopic list : \
    publish: should have /imu_data
- sensor_msgs::Imu
            /imu_filter_data  
- my_robot::imu_filter_msg as ang_x, ang_y, ang_z linear_x, linear_y, linear_z
    /laser_scan 
- sensor_msgs::LaserScan
    subscribe:: /laser_node \
    subscribe:\ /imu_filer_data and print "got imu_filter data" print ang_x paraments


**ERROR**

~~1. laser has no topic published~~ \
error show as
```
rostopic list shows /laser_scan
rostopc echo /laser_scan
nothing show up
```
***checkout！*** \
 should not start ros::spin() in LASER::LASER \
laser test done!

~~2. imu works error!~~

errror show as 
```
Segmentation fault (core dumped) 
```
***checkout!*** \
imu_filter function required arguements!\
nolonger required! \
remember when changed!

### TODO
1. use imu_filter_data in /laser_scan
2. update tf function 
3. finish temp.cpp to use matrix UKF for imu_data
4. remember imu_data picture for paper
