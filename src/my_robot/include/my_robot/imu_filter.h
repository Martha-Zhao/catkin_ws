#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include <deque>
#include <eigen3/Eigen/Geometry> 
#include <chrono>
#include <locale>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <boost/assert.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

extern "C" {
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h> //  close
#include <string.h> //  strerror
}


#include "my_robot/imu_filter_msg.h"

#define IMU_JUMP_FRAME 600

class IMU{
    public:
        IMU(double input_Q, double input_R);
        ~IMU();
        void imu_pub_func();
        
    private:
    // varibles  
        ros::NodeHandle n;
        std::string name = ros::this_node::getName();
        std::string port = "/dev/imu";
        std::string model = "art_imu_02a";
        int baud = 115200;
        std::string frame_id = "IMU_link";
        double delay;
        boost::asio::io_service io_service;
        boost::asio::serial_port* serial_port = 0;
        int fd_ = -1;
        ros::Publisher imu_data_pub;
        sensor_msgs::Imu msg;
        ros::Publisher imu_fiilter_pub;
        my_robot::imu_filter_msg imu_filter_msg;

        const uint8_t stop[6] = {0xA5, 0x5A, 0x04, 0x02, 0x06, 0xAA};
        const uint8_t mode[6] = {0xA5, 0x5A, 0x04, 0x01, 0x05, 0xAA};
        int data_length = 81;
        uint8_t tmp[81];
        uint8_t data_raw[200];
        

        int imu_init_jump_count = IMU_JUMP_FRAME;
        double sum_acceleration_x = 0, sum_acceleration_y = 0, sum_acceleration_z = 0; //linear acceleration
        double sum_angular_x = 0, sum_angular_y = 0, sum_angular_z = 0; //angular velocity

        double velociry_x = 0, velocity_z = 0;
        double velociry_x_lase = 0, velocity_z_last = 0;
        
        double linear_velocity_x = 0, linear_velocity_y = 0, linear_velocity_z = 0;
        double angle_x = 0, angle_y = 0, angle_z = 0;

        //paraments in UKF
        double last_p = 0, now_p = 0;
        double Q = 1, K = 0, R = 1.3;
        double output = 0;

        //paraments in file edit
        // FILE* filterfile = fopen("filter.data", "r+");


    
    // functions 
        int imu_init(){
            try{
                serial_port->open(port);
            }
            catch (boost::system::system_error &error){
                ROS_ERROR("%s: Failed to open port %s with error %s",
                        name.c_str(), port.c_str(), error.what());
                return -1;
            }

            if (!serial_port->is_open()){
                ROS_ERROR("%s: failed to open serial port %s",
                        name.c_str(), port.c_str());
                return -1;
            }

            typedef boost::asio::serial_port_base sb;

            sb::baud_rate baud_option(baud);
            sb::flow_control flow_control(sb::flow_control::none);
            sb::parity parity(sb::parity::none);
            sb::stop_bits stop_bits(sb::stop_bits::one);

            serial_port->set_option(baud_option);
            serial_port->set_option(flow_control);
            serial_port->set_option(parity);
            serial_port->set_option(stop_bits);

            const char *path = port.c_str();
            fd_ = open(path, O_RDWR);

            if(fd_ < 0){
                ROS_ERROR("Port Error!: %s", path);
                return -1;
            }

            if(model == "art_imu_02a"){
            write(fd_, stop, 6);
            usleep(1000 * 1000);
            write(fd_, mode, 6);
            usleep(1000 * 1000);
            data_length = 40;
            }
            ROS_WARN("Streaming Data...");

        }

        void imu_filter_func(double* ,int );
        
};


#endif