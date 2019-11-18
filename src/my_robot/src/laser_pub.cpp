#include "iostream"
#include <std_msgs/Int32.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "my_robot/laser_driver.h"

using namespace std;
string laser_link = "laser_link";
io_driver driver;

void publish_scan(ros::Time start, int count, double scan_time, double* ranges, double* intensities, ros::Publisher *pub){
    static int scan_count = 0;
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = laser_link;

    scan_msg.angle_min = 0.0;
	scan_msg.angle_max = 2 * M_PI;
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min)/(count- 1);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double) (count - 1);

    scan_msg.range_min = 0.1;
    scan_msg.range_max = 10.0;

    scan_msg.ranges.resize(count);
    scan_msg.intensities.resize(count);

    for (unsigned int i = 0 ; i < count; i++, ranges++, intensities++){
        scan_msg.ranges[i] = *ranges / 1000;
        scan_msg.intensities[i] = *intensities;
    }

    pub->publish(scan_msg);
    scan_count++;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "laser_test_node");
    ros::NodeHandle n;
    ros::Rate r(1.0);

    string port = "/dev/laser";
    
    double angle[PACKLEN + 10];
	double distance[PACKLEN + 10];
	double data[PACKLEN + 10];
	double data_intensity[PACKLEN + 10];
	double speed;
    string topic = "laser_scan";
/*     string topic = "laser_test";
    string laser = "laser_link";
    double angle_disable_min = -1;  
    double angle_disable_max = -1;
    bool flag_1 = true;
    bool flag_2 = false; */
   
/*     ros::param::get("~scan_topic", topic);
	ros::param::get("~laser_link", laser);
	ros::param::get("~serial_port", port);
	ros::param::get("~angle_disable_min", angle_disable_min);
	ros::param::get("~angle_disable_max", angle_disable_max);
	ros::param::get("~zero_as_max", flag_1);
	ros::param::get("~min_as_zero", flag_1);
	ros::param::get("~inverted", flag_2); */
    ros::Publisher laser_pub =n.advertise<sensor_msgs::LaserScan>(topic, 1000);
    int ret;
	ret = driver.OpenSerial(port.c_str(), B230400);

	if (ret < 0){
		ROS_ERROR("could not open port:%s", port.c_str());
		return 0;
	}
	else
		ROS_INFO("open port:%s", port.c_str());

    driver.StartScan();
    ROS_INFO("Send start command successfully");

    ros::Time starts = ros::Time::now();
	ros::Time ends = ros::Time::now();


    while(ros::ok()){
        starts = ros::Time::now();
        ros::spinOnce();
        memset(data, 0, sizeof(data));
        int ret = driver.GetScanData(angle, distance, PACKLEN, &speed);
        for (unsigned int i = 0; i < PACKLEN; i ++){
            data[i] = distance[i];
            data_intensity[i] = angle[i];
        }
        //ROS_INFO_THROTTLE(30, "ls01g works fine!");
        ends = ros::Time::now();
        float scan_duration = (ends - starts).toSec() * 1e-3;
        publish_scan(starts, ret, scan_duration, data, data_intensity, &laser_pub);
        ROS_INFO("SCAN ONCE!");
        r.sleep();
    }

    driver.StopScan(STOP_DATA);
    driver.StopScan(STOP_MOTOR);
    driver.CloseSerial();
    ROS_INFO("Keyboard Interrupt, ls01g stop!");

    return 0;
}