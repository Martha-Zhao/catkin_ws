#ifndef LASER_NODE_H
#define LASER_NODE_H

#include "iostream"
#include <std_msgs/Int32.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "laser_driver.h"

using namespace std;

class LASER{
    public:
        LASER();
        ~LASER();

        void laser_origin_pub_func();
        void laser_close();
    private:
        ros::NodeHandle n;
        string laser_link = "laser_link";
        io_driver driver;
        string port = "/dev/laser";

        double angle[PACKLEN + 10];
        double distance[PACKLEN + 10];
        double data[PACKLEN + 10];
        double* p_data;
        double data_intensity[PACKLEN + 10];
        double speed;
        float scan_duration;
        string topic = "laser_scan";

        ros::Publisher laser_pub;
        sensor_msgs::LaserScan scan_msg;
        int ret;

        ros::Time starts, ends; 
};

#endif