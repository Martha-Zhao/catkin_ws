#include "my_robot/laser_node.h"
using std::cout;
using std::endl;


LASER::LASER(){
    laser_pub = n.advertise<sensor_msgs::LaserScan>("/laser_scan", 10);
    ret = driver.OpenSerial(port.c_str(), B230400);
    if (ret < 0){
		ROS_ERROR("could not open port:%s", port.c_str());
		// return 0;
	}
	else
		ROS_INFO("open port:%s", port.c_str());
    
};

LASER::~LASER(){};

void LASER::laser_origin_pub_func(){
    starts = ros::Time::now();
    memset(data, 0, sizeof(data));
    p_data = data;
    ret = driver.GetScanData(angle, distance, PACKLEN, &speed);
    for (unsigned int i = 0; i < PACKLEN; i ++){
        data[i] = distance[i];
        data_intensity[i] = angle[i];
    }
    ends = ros::Time::now();
    scan_duration = (ends - starts).toSec() * 1e-3;
    scan_msg.header.stamp = starts; //should change!!
    scan_msg.header.frame_id = laser_link;
    scan_msg.angle_min = 0.0;
	scan_msg.angle_max = 2 * M_PI;
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min)/(ret- 1);
    scan_msg.scan_time = scan_duration;
    scan_msg.time_increment = scan_duration / (double) (ret - 1);
    scan_msg.range_min = 0.1;
    scan_msg.range_max = 10.0;
    scan_msg.ranges.resize(ret);
    scan_msg.intensities.resize(ret);
    for (unsigned int i = 0 ; i < ret; i++, p_data++){
        scan_msg.ranges[i] = *p_data / 1000;
        scan_msg.intensities[i] = 0;
    }
    laser_pub.publish(scan_msg);
}

void LASER::laser_close(){
    driver.StopScan(STOP_DATA);
    driver.StopScan(STOP_MOTOR);
    driver.CloseSerial();
    ROS_INFO("Keyboard Interrupt, ls01g stop!");
}


int main(int argc, char** argv){
    ros::init(argc, argv, "laser_test_node");
    ros::NodeHandle nh;
    ros::Rate r(1.0);
    LASER laser;
    while(nh.ok()){
        laser.laser_origin_pub_func();
        ros::spinOnce();
        r.sleep();
    }
    laser.laser_close();
    return 0;
}