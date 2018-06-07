#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include <math.h>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"

using namespace std;

int main(int argc, char **argv){

    const string node_name = "rt_usb_9axis_sensor";
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    string port,frame_id;

    //Publisher.
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw",1);
    ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField>("imu/mag",1);
    ros::Publisher temp_pub = n.advertise<sensor_msgs::Temperature>("imu/temperature",1);

    //Meseage.
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;
    sensor_msgs::Temperature temp_msg;

    //Standard deviation.
    double linear_acceleration_stdev;
    double angular_velocity_stdev;
    double magnetic_field_stdev;

    n.param("port", port,string("/dev/ttyACM0"));
    n.getParam(node_name + "/port", port);

    n.param("frame_id", frame_id,string("imu_link"));
    n.param("linear_acceleration_stdev", linear_acceleration_stdev, 0.023145);
    n.param("angular_velocity_stdev", angular_velocity_stdev, 0.0010621);
    n.param("magnetic_field_stdev", magnetic_field_stdev, 0.00000080786);

    double linear_acceleration_cov = (linear_acceleration_stdev * linear_acceleration_stdev);
    double angular_velocity_cov = (angular_velocity_stdev * angular_velocity_stdev);
    double magnetic_field_cov = (magnetic_field_stdev * magnetic_field_stdev);

    imu_msg.linear_acceleration_covariance[0] = linear_acceleration_cov;
    imu_msg.linear_acceleration_covariance[4] = linear_acceleration_cov;
    imu_msg.linear_acceleration_covariance[8] = linear_acceleration_cov;
    imu_msg.angular_velocity_covariance[0] = angular_velocity_cov;
    imu_msg.angular_velocity_covariance[4] = angular_velocity_cov;
    imu_msg.angular_velocity_covariance[8] = angular_velocity_cov;
    mag_msg.magnetic_field_covariance[0] = magnetic_field_cov;
    mag_msg.magnetic_field_covariance[4] = magnetic_field_cov;
    mag_msg.magnetic_field_covariance[8] = magnetic_field_cov;
    temp_msg.variance = 0;

    //Serial
    unsigned char buf[255];
    struct termios tio;
    int baudRate = B115200;
    int i,fd,buf_size,count=0;
    vector<string> imu_data;
    string buf_imu;

    fd = open(port.c_str(),O_RDWR | O_NOCTTY);
    if(fd < 0){
      cout << "Device not found." << endl;
      return -1;
    }

    tio.c_cflag += CREAD;
    tio.c_cflag += CLOCAL;
    tio.c_cflag += CS8;
    tio.c_cflag += 0;
    tio.c_cflag += 0;
    cfsetispeed( &tio,baudRate);
    cfsetospeed( &tio,baudRate);
    cfmakeraw(&tio);
    tcsetattr(fd,TCSANOW,&tio);
    ioctl(fd,TCSETS,&tio);

    int rec_count = 0;
    while(ros::ok()){
        tcflush(fd,TCIFLUSH);
        buf_imu.clear();
        buf_size = read(fd,buf,sizeof(buf));
        if (0 < buf_size){
            rec_count++;
            for(i = 0; i < buf_size; i++){
                if(buf[i]==','|| buf[i]=='\n'){
                    count++;
                    imu_data.push_back(buf_imu);
                    buf_imu.clear();
                }else{
                    buf_imu+=buf[i];
                }
                if(buf[i]=='\n' && count==11 && imu_data[0].find(".")==string::npos){
                    ros::Time ros_time = ros::Time::now();

                    imu_msg.header.frame_id = frame_id;
                    imu_msg.header.stamp = ros_time;
                    mag_msg.header.stamp = ros_time;
                    temp_msg.header.stamp = ros_time;

                    imu_msg.angular_velocity.x = stof(imu_data[1]);
                    imu_msg.angular_velocity.y = stof(imu_data[2]);
                    imu_msg.angular_velocity.z = stof(imu_data[3]);

                    imu_msg.linear_acceleration.x = stof(imu_data[4]) * 9.81;
                    imu_msg.linear_acceleration.y = stof(imu_data[5]) * 9.81;
                    imu_msg.linear_acceleration.z = stof(imu_data[6]) * 9.81;

                    mag_msg.magnetic_field.x =  stof(imu_data[7]) / 1000000;
                    mag_msg.magnetic_field.y =  stof(imu_data[8]) / 1000000;
                    mag_msg.magnetic_field.z =  stof(imu_data[9]) / 1000000;

                    temp_msg.temperature = stof(imu_data[10]);

                    imu_pub.publish(imu_msg);
                    mag_pub.publish(mag_msg);
                    temp_pub.publish(temp_msg);
                    rec_count=0;
                    count = 0;
                    imu_data.clear();
                }
                if(count>11){
                    buf_imu.clear();
                    imu_data.clear();
                    rec_count=0;
                    count=0;
                }
            }
        }
    }
    close(fd);
    return 0;
}
