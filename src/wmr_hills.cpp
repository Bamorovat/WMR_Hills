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
 * @file src/wmr_hills.cpp
 *
 * @brief Robot_Communication!
 *
 * @date December 2018
 **/

#include "wmr_hills.h"
#include "wmr_udp_communication.h"

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>

using namespace std;

bool ini_ok,Sensor_Ok,Pid_Ok,Odometry_Ok,Movement_Ok,Move_Cmd_Ok,Motor_Ok;

wmr_udp_communication wmr_udp;

wmr_robot::wmr_robot()
{

}

wmr_robot::~wmr_robot()
{

}

bool wmr_robot::init() // init - Primiry test & setting
{           
    wmr_robot::Movement_cmd Movement_cmd_BUFFER;
//  Movement_cmd_BUFFER.Mode = 0x04;
    Movement_cmd_BUFFER.Velocity_Y= 0.0;
    Movement_cmd_BUFFER.Velocity_W= 0.0;
    int Movement_cmd_BUFFER_Size = sizeof(Movement_cmd_BUFFER);
    ini_ok = wmr_udp.Movement_CMD(Movement_cmd_BUFFER,Movement_cmd_BUFFER_Size);

    wmr_robot::Motor_cmd Motor_cmd_BUFFER;
    Motor_cmd_BUFFER.Mode = 0x01;
    Motor_cmd_BUFFER.DirR = 0x01; //0x02;
    Motor_cmd_BUFFER.DirL = 0x01; //0x02;
    Motor_cmd_BUFFER.SpeedR = 0; //1700;
    Motor_cmd_BUFFER.SpeedL = 0; //1700;
    int Motor_cmd_BUFFER_Size = sizeof(Motor_cmd_BUFFER);
    Motor_Ok=wmr_udp.Motor_CMD(Motor_cmd_BUFFER,Motor_cmd_BUFFER_Size);

    wmr_robot::Sensor_DATA Sensor_DATA_Buffer;
    int Sensor_DATA_Buffer_Size;
    Sensor_Ok = wmr_udp.Sensor_Status(Sensor_DATA_Buffer,Sensor_DATA_Buffer_Size);

    wmr_robot::PID_DATA PID_Data_Buffer;
    int PID_Data_Buffer_Size;
    Pid_Ok = wmr_udp.PID_Status(PID_Data_Buffer,PID_Data_Buffer_Size);

    wmr_robot::Odometery_DATA Odometery_DATA_Buffer;
    int Odometery_DATA_Buffer_Size;
    Odometry_Ok = wmr_udp.Odometery_Status(Odometery_DATA_Buffer,Odometery_DATA_Buffer_Size);

    wmr_robot::Movement_DATA Movement_DATA_Buffer;
    int Movement_DATA_Buffer_Size;
    Movement_Ok = wmr_udp.Movement_Data(Movement_DATA_Buffer,Movement_DATA_Buffer_Size);
}

void wmr_robot::Sensors(wmr_robot::Sensor_DATA &Sensor_DATA_Buffer,int Sensor_DATA_Buffer_Size)
{
    Sensor_Ok = wmr_udp.Sensor_Status(Sensor_DATA_Buffer,Sensor_DATA_Buffer_Size);
}

void wmr_robot::PID(wmr_robot::PID_DATA &PID_Data_Buffer,int PID_Data_Buffer_Size)
{
    Pid_Ok = wmr_udp.PID_Status(PID_Data_Buffer,PID_Data_Buffer_Size);
}

void wmr_robot::Odometry(wmr_robot::Odometery_DATA &Odometery_DATA_Buffer,int Odometery_DATA_Buffer_Size)
{
    Odometry_Ok = wmr_udp.Odometery_Status(Odometery_DATA_Buffer,Odometery_DATA_Buffer_Size);
}

void wmr_robot::Movement(wmr_robot::Movement_DATA &Movement_DATA_Buffer,int Movement_DATA_Buffer_Size)
{
    Movement_Ok = wmr_udp.Movement_Data(Movement_DATA_Buffer,Movement_DATA_Buffer_Size);
}

void wmr_robot::Movement_Command(float Angular_velocity,float Linear_Velocity)
{
    wmr_robot::Movement_cmd Movement_cmd_BUFFER;
//  Movement_cmd_BUFFER.Mode = 0x04;
    Movement_cmd_BUFFER.Velocity_Y= Angular_velocity;
    Movement_cmd_BUFFER.Velocity_W= Linear_Velocity;
    int Movement_cmd_BUFFER_Size = sizeof(Movement_cmd_BUFFER);
    Move_Cmd_Ok=wmr_udp.Movement_CMD(Movement_cmd_BUFFER,Movement_cmd_BUFFER_Size);
}

void wmr_robot::Motor_Command(Byte mode, Byte Dir_Right, Byte Dir_Left, uint16_t Speed_Right, uint16_t Speed_Left)
{
    wmr_robot::Motor_cmd Motor_cmd_BUFFER;
    Motor_cmd_BUFFER.Mode = mode;
    Motor_cmd_BUFFER.DirR = Dir_Right; //0x02;
    Motor_cmd_BUFFER.DirL = Dir_Left; //0x02;
    Motor_cmd_BUFFER.SpeedR = Speed_Right; //1700;
    Motor_cmd_BUFFER.SpeedL = Speed_Left; //1700;
    int Motor_cmd_BUFFER_Size = sizeof(Motor_cmd_BUFFER);
    Motor_Ok=wmr_udp.Motor_CMD(Motor_cmd_BUFFER,Motor_cmd_BUFFER_Size);
}







