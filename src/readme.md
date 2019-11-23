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


### 2019-11-21
- build with EKF node
    DONE!
- how to use EKF
    create launch file

- create launch files


### 2019-11-22
- finish my launch file

### 2019-11-23
- create test launch file
- folder struct
1. catkin_ws 
    - contain: build/ devel/ src/
2. src/ 
    - contain: package1 ... packagen and CMakeLists.txt
3. package1/ 
    - contain: include/ launch/ params/ src/ srv/ msg/ and CMakeLists.txt package.xml 
    - one package and contain multiple nodes lists in several folders but only has one CMLists.txt and package.xml 

4. src/ and include/ in package1 
    - main .cpp and .h files

5. launch/ and params/ in package1
    - launch file that can launch several nodes once whitch can located in different packages
    - use parames/ if necessary
6. srv/ in package1
    - service if needed
7. msg/ in packag1
    - create your own msg in this folder save as my_msg.msg
    - need to add some items in package's CMakeLIsts.txt and package.xml
8. bags/ in package1
    - save runtime topic data in this filder si that this can be replay

#### TODO
learn .xml 
