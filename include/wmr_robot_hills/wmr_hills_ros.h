/*
    SOSCO, Wheeled Mobile Robot Hills (WMR_ROBOT_HILLS)
    Copyright (C) 2018  Mohammad Hossein Bamorovat Abadi

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file /include/wmr_robot_hills/wmr_hills_ros.h
 *
 * @brief ROS_Communication!
 *
 * @date December 2018
 **/

#ifndef WMR_HILLS_ROS
#define WMR_HILLS_ROS

#include "wmr_hills.h"
#include "wmr_udp_communication.h"

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>

#include "wmr_robot_hills/motor_cmd.h"
#include "wmr_robot_hills/movement_data.h"
#include "wmr_robot_hills/odometry_data.h"
#include "wmr_robot_hills/pid_data.h"

class wmr_robot_ros {

public:

    static const int Ir_Sensor_Num = 16;
    static const int Ultrasonic_Sensor_Num = 16;

    ros::Subscriber motor_vel_sub;
    ros::Subscriber cmd_vel_sub;

    ros::Publisher PID_Data_Pub;
    ros::Publisher odometry_Data_Pub;
    ros::Publisher Movement_data_Pub;
    ros::Publisher Sonar_Pub;
    ros::Publisher IR_Sharp_Pub;

public:
    void odometry_node();
    void pid_node();
    void movement_node();
    void sensor_node();
};

#endif // WMR_HILLS_ROS
