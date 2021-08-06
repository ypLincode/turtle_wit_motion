/*
 * @Author: your name
 * @Date: 2021-07-19 10:36:43
 * @LastEditTime: 2021-08-06 13:10:40
 * @LastEditors: linyuanpei
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/turtle_wit_motion-master/include/turtle_wit_motion.h
 */
#ifndef __TURTLE_WIT_MOTION_INCLUDE_TURTLE_WIT_MOTION_H__
#define __TURTLE_WIT_MOTION_INCLUDE_TURTLE_WIT_MOTION_H__

#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "vector"
class TurtleWitMotion {
    
public:
    TurtleWitMotion(ros::NodeHandle& nod_);
    void teleopTurtleJoy(ros::NodeHandle& nod_);
    bool openPort();
    void dataRead();

private:
    const uint8_t pack_first = 0x55, linear = 0x51, angular = 0x52, angle = 0x53, msg = 0x59;
    const uint8_t acc_cali=0x67;
    const uint8_t set_20hz=0x64;
    double a[3], w[3], ang[3], PI = 3.1415926;
    int baudrate;
    std::string topic_read, port;
    serial::Serial ser_;
    ros::Time time_wit_motion;
    ros::Publisher wit_pub_,joy_pub_;
    ros::Subscriber wit_sub_,joy_sub_;
    int read_rate_=100;

    int time_wit_delta_;//  为一个长数据数组的多个包增加时间间隔

    void motioncallBack(const sensor_msgs::Imu::ConstPtr& imu);
    void valueComplain(std::vector<uint8_t> &vComplain);
};
#endif