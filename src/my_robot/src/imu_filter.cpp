#include "stdlib.h"
#include "my_robot/imu_filter.h"

using std::cout;
using std::endl;

// FILE* originfile = fopen("test.data", "r+");


static float d2f_acc(uint8_t a[2])
{
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 16384.0f;
}

static float d2f_gyro(uint8_t a[2])
{
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 32.8f;
}

static float d2f_mag(uint8_t a[2])
{
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 1.0f;
}

static float d2f_euler(uint8_t a[2])
{
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 10.0f;
}

static double d2f_latlon(uint8_t a[4])
{
    int64_t high = a[0];
    high = (high << 8) | a[1];

    int64_t low = a[2];
    low = (low << 8) | a[3];
    return (double)((high << 8) | low);
}

static double d2f_gpsvel(uint8_t a[2])
{
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 10.0f;
}

static float d2ieee754(uint8_t a[4])
{
    union fnum {
        float f_val;
        uint8_t d_val[4];
    } f;

    memcpy(f.d_val, a, 4);
    return f.f_val;
}


int uart_set(int fd, int baude, int c_flow, int bits, char parity, int stop)
{
    struct termios options;
 
    if(tcgetattr(fd, &options) < 0)
    {
        perror("tcgetattr error");
        return -1;
    }

    cfsetispeed(&options,B115200);
    cfsetospeed(&options,B115200);

    options.c_cflag |= CLOCAL;
    options.c_cflag |= CREAD;

    switch(c_flow)
    {
        case 0:
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1:
            options.c_cflag |= CRTSCTS;
            break;
        case 2:
            options.c_cflag |= IXON|IXOFF|IXANY;
            break;
        default:
            fprintf(stderr,"Unkown c_flow!\n");
            return -1;
    }

    switch(bits)
    {
        case 5:
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unkown bits!\n");
            return -1;
    }

    switch(parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~INPCK;
            break;

        case 's':
        case 'S':
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;

        case 'o':
        case 'O':
            options.c_cflag |= PARENB;
            options.c_cflag |= PARODD;
            options.c_cflag |= INPCK;
            options.c_cflag |= ISTRIP;
            break;

        case 'e':
        case 'E':
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_cflag |= INPCK;
            options.c_cflag |= ISTRIP;
            break;
        default:
            fprintf(stderr,"Unkown parity!\n");
            return -1;
    }

    switch(stop)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr,"Unkown stop!\n");
            return -1;
    }

    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 1;

    tcflush(fd,TCIFLUSH);

    if(tcsetattr(fd,TCSANOW,&options) < 0)
    {
        perror("tcsetattr failed");
        return -1;
    }

    return 0;

}


IMU::IMU(double input_Q, double input_R){
    /**************set element in UKF********************/
    Q = input_Q;
    R = input_R;
    printf("Q = %f , R = %f \n", Q, R);
    n.setParam("port", port);
    n.setParam("model", model);
    ROS_WARN("Model set to %s", model.c_str());
    n.setParam("baud", baud);
    ROS_WARN("Baudrate set to %d", baud);
    n.param("frame_id", frame_id);
    n.param("delay", delay, 0.0);
    serial_port = new boost::asio::serial_port(io_service);
    imu_init();
    imu_data_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 1);
    imu_fiilter_pub = n.advertise<my_robot::imu_filter_msg>("/imu_filter_data", 1);
};

IMU::~IMU(){};

void IMU::imu_pub_func(){
    double vyaw_sum = 0;
    double vyaw_bias = 0;
    
    while(n.ok()){
        read(fd_, tmp, sizeof(uint8_t) * data_length);
        memcpy(data_raw, tmp, sizeof(uint8_t) * data_length);
        bool found = false;
        int kk;
        for(kk = 0; kk < data_length - 1; ++kk){
            if(model == "art_imu_02a" && data_raw[kk] == 0xA5 && data_raw[kk + 1] == 0x5A){
                unsigned char *data = data_raw + kk;
                uint8_t data_length = data[2];
                // --------------check
                uint32_t checksum = 0;
                for(int i = 0; i < data_length - 1  ; ++i)
                    checksum += (uint32_t) data[i+2];

                uint16_t check = checksum % 256;
                uint16_t check_true = data[data_length+1];

                if (check != check_true){
                    printf("check error,please wait\n");
                    continue;
                }
                Eigen::Vector3d ea0((-vyaw_bias-d2f_euler(data + 3)) * M_PI / 180.0,
                                 0,
                                  0);
                Eigen::Matrix3d R;
                R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
                Eigen::Quaterniond q;
                q = R;

                msg.orientation.w = (double)q.w();
                msg.orientation.x = (double)q.x();
                msg.orientation.y = (double)q.y();
                msg.orientation.z = (double)q.z();

                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = frame_id;
                msg.angular_velocity.x = d2f_gyro(data + 15);
                msg.angular_velocity.y = d2f_gyro(data + 17);
                msg.angular_velocity.z = d2f_gyro(data + 19);
                msg.linear_acceleration.x = d2f_acc(data + 9) * 9.81;
                msg.linear_acceleration.y = d2f_acc(data + 11) * 9.81;
                msg.linear_acceleration.z = d2f_acc(data + 13) * 9.81;  
                if (imu_init_jump_count == 0){
                    printf("imu initlization DDONE!\n");
                    printf("The parament is for linear x is %f\n", sum_acceleration_x/IMU_JUMP_FRAME);
                    printf("The parament is for linear y is %f\n", sum_acceleration_y/IMU_JUMP_FRAME);
                    printf("The parament is for linear z is %f\n", sum_acceleration_z/IMU_JUMP_FRAME);

                    printf("The parament is for angular x is %f\n", sum_angular_x/IMU_JUMP_FRAME);
                    printf("The parament is for angular y is %f\n", sum_angular_y/IMU_JUMP_FRAME);
                    printf("The parament is for angular z is %f\n", sum_angular_z/IMU_JUMP_FRAME);

                    imu_init_jump_count--;
                }

                if (imu_init_jump_count > 0){
                    sum_acceleration_x += msg.linear_acceleration.x;
                    sum_acceleration_y += msg.linear_acceleration.y;
                    sum_acceleration_z += msg.linear_acceleration.z;

                    sum_angular_x += msg.angular_velocity.x;
                    sum_angular_y += msg.angular_velocity.y;
                    sum_angular_z += msg.angular_velocity.z;

                    imu_init_jump_count--;
                }
                else{
                    msg.linear_acceleration.x -= sum_acceleration_x/IMU_JUMP_FRAME;
                    msg.linear_acceleration.y -= sum_acceleration_y/IMU_JUMP_FRAME;
                    msg.linear_acceleration.z -= sum_acceleration_z/IMU_JUMP_FRAME;

                    msg.angular_velocity.x -=  sum_angular_x/IMU_JUMP_FRAME;
                    msg.angular_velocity.y -=  sum_angular_y/IMU_JUMP_FRAME;
                    msg.angular_velocity.z -=  sum_angular_z/IMU_JUMP_FRAME;

                    linear_velocity_x += msg.linear_acceleration.x;
                    linear_velocity_y += msg.linear_acceleration.y;
                    linear_velocity_z += msg.linear_acceleration.z;

                    angle_x += msg.angular_velocity.x;
                    angle_y += msg.angular_velocity.y;
                    angle_z += msg.angular_velocity.z;

                    
                    imu_filter_msg.linear_x = linear_velocity_x;
                    imu_filter_msg.linear_y = linear_velocity_y;
                    imu_filter_msg.linear_z = linear_velocity_z;

                    imu_filter_msg.ang_x = angle_x;
                    imu_filter_msg.ang_y = angle_y;
                    imu_filter_msg.ang_z = angle_z;
                }
                imu_data_pub.publish(msg);
                imu_fiilter_pub.publish(imu_filter_msg);

                found = true;
            }
        }
    }
    ROS_WARN("Wait 0.1s");
    // fclose(originfile);
    // fclose(filterfile);
    ros::Duration(0.1).sleep();
    ::close(fd_);
    
}

void IMU::imu_filter_func(double* measurement, int filter_element_flag){
    if (filter_element_flag == 0){//x axis
        // cout<<"origin measurement = "<<*measurement;
        // fseek(originfile, 0L, SEEK_END);
        // fprintf(originfile, "%lf", *measurement);
        // fputs(" ", originfile);
        now_p = last_p + Q;
        K = now_p/(now_p + R);
        output += K * (*measurement - output);
        last_p = now_p*(1 - K);
        *measurement = output;
        // cout<<"    measurement after filter = "<<*measurement<<endl;
        // fseek(filterfile, 0L, SEEK_END);
        // fprintf(filterfile, "%lf", *measurement);
        // fputs(" ", filterfile);
    }  
}



int main(int argc, char** argv){
    ros::init(argc, argv, "imu_node");
    // ros::NodeHandle n;
    // if (argc != 3){
    //     printf("please set Q and R\n");
    //     return 0;
    // }
    double q = 1, r = 1.5;
    // q = atof(*(argv + argc - 2));
    // r = atof(*(argv + argc - 1));
    IMU imu_test(q, r);
    imu_test.imu_pub_func();
    ROS_WARN("imu_pub_func shut dhwn!");
    // while(ros::ok()){

    // }

    return 0;
}