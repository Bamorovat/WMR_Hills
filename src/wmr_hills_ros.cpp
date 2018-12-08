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
 * @file src/wmr_hills_ros.cpp
 *
 * @brief ROS_Communication!
 *
 * @date December 2018
 **/

#include "wmr_hills.h"
#include "wmr_udp_communication.h"
#include "wmr_hills_ros.h"

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>

#include "wmr_robot_hills/motor_cmd.h"      ///Motor Costume Command _ ROS
#include "wmr_robot_hills/movement_data.h"  ///Movement Costume Command _ ROS
#include "wmr_robot_hills/odometry_data.h"  ///Odometry Costume Command _ ROS
#include "wmr_robot_hills/pid_data.h"       ///PID Costume Command _ ROS

geometry_msgs::Twist move_cmd;

wmr_robot WMR_Robot;
wmr_udp_communication wmr_udp1;
wmr_robot_ros WMR_ROBOT_ROS;

float X,Y,th;

void callbak_move_cmd(const geometry_msgs::Twist::ConstPtr& msg)    // <<<<<<<<<<<<<<<<<<<<< cmd_vel Function >>>>>>>>>>>>>>>>>>>>> //
{
  //ROS_INFO("I heard: [%f]", msg->velocity);
    move_cmd.linear.x = msg->linear.x;
    move_cmd.angular.z = msg->angular.z;

    WMR_Robot.Movement_Command(move_cmd.linear.x,move_cmd.angular.z);
}

void callback_motor_cmd(const wmr_robot_hills::motor_cmd& msg)      // <<<<<<<<<<<<<<<<<<<< motor_vel Function >>>>>>>>>>>>>>>>>>>>>> //
{
   wmr_robot::Motor_cmd motor_cmd;

    motor_cmd.Mode = msg.Mode;
    motor_cmd.DirR = msg.DirR;
    motor_cmd.DirL = msg.DirL;
    motor_cmd.SpeedR = msg.SpeedR;
    motor_cmd.SpeedL = msg.SpeedL;

    WMR_Robot.Motor_Command(motor_cmd.Mode,motor_cmd.DirR,motor_cmd.DirL,motor_cmd.SpeedR,motor_cmd.SpeedL);
}

void wmr_robot_ros::odometry_node()             // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< ODOMETRY ROS NODE FUNCTION >>>>>>>>>>>>>>>>>>>>>>>>>> //
{
    wmr_robot::Odometery_DATA Odometry_Data_Buffer;
    int Odometry_Data_Buffer_Size = sizeof(Odometry_Data_Buffer);
    WMR_Robot.Odometry(Odometry_Data_Buffer,Odometry_Data_Buffer_Size);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();


    X += Odometry_Data_Buffer.Result_X;
    Y += Odometry_Data_Buffer.Result_Y;
    th += Odometry_Data_Buffer.Result_Teta;


     tf::TransformBroadcaster broadcaster;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Odometry_Data_Buffer.Result_Teta);
 //    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

     //first, we'll publish the transform over tf
     geometry_msgs::TransformStamped odom_trans;
     odom_trans.header.stamp = current_time;
     odom_trans.header.frame_id = "odom";
     odom_trans.child_frame_id = "base_link";

     odom_trans.transform.translation.x = Odometry_Data_Buffer.Result_X;
     odom_trans.transform.translation.y = Odometry_Data_Buffer.Result_Y;
//     odom_trans.transform.translation.x = X;
//     odom_trans.transform.translation.y = Y;
     odom_trans.transform.translation.z = 0.0;
     odom_trans.transform.rotation = odom_quat;

     //send the transform
     broadcaster.sendTransform(odom_trans);

     //Laser publish the transform over tf
     geometry_msgs::TransformStamped laser_trans;
     laser_trans.header.stamp = ros::Time::now();
     laser_trans.header.frame_id = "base";
     laser_trans.child_frame_id = "laser";

     laser_trans.transform.translation.z = 0.035;       // Should be regulate for the robot
     geometry_msgs::Quaternion laser_quat = tf::createQuaternionMsgFromYaw(0);
     laser_trans.transform.rotation = laser_quat;
     //send the transform
     broadcaster.sendTransform(laser_trans);

     //next, we'll publish the odometry message over ROS
     nav_msgs::Odometry odom;
     odom.header.stamp = current_time;
     odom.header.frame_id = "odom";

     //set the position
     odom.pose.pose.position.x += Odometry_Data_Buffer.Result_X;
     odom.pose.pose.position.y += Odometry_Data_Buffer.Result_Y;
 //    odom.pose.pose.position.x = X;
 //    odom.pose.pose.position.y = Y;
     odom.pose.pose.position.z = 0.0;
     odom.pose.pose.orientation = odom_quat;

     //set the velocity
     odom.child_frame_id = "base_link";
     odom.twist.twist.linear.x  = move_cmd.linear.x;
     odom.twist.twist.linear.y  = 0;
     odom.twist.twist.linear.z  = 0;
     odom.twist.twist.angular.x = 0;
     odom.twist.twist.angular.y = 0;
     odom.twist.twist.angular.z = move_cmd.angular.z;

     tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(Odometry_Data_Buffer.Result_Teta), tf::Vector3(Odometry_Data_Buffer.Result_X,Odometry_Data_Buffer.Result_Y, 0)), odom.pose.pose);
//     tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(th), tf::Vector3(X,Y, 0)), odom.pose.pose);
     //publish the message
     WMR_ROBOT_ROS.odometry_Data_Pub.publish(odom);

     last_time = current_time;


    }

void wmr_robot_ros::pid_node()                     // <<<<<<<<<<<<<<<<<<<<<<<<< PID ROS NODE FUNCTION >>>>>>>>>>>>>>>>>>>>>>>>>>> //
{
    wmr_robot::PID_DATA PID_Data_Buffer;

    int PID_Data_Buffer_Size = sizeof(PID_Data_Buffer);
    WMR_Robot.PID(PID_Data_Buffer,PID_Data_Buffer_Size);

    wmr_robot_hills::pid_data pid_msg;

    pid_msg.UP_REF_M1 = PID_Data_Buffer.UP_REF_M1;
    pid_msg.UI_REF_M1 = PID_Data_Buffer.UI_REF_M1;
    pid_msg.Err_ENcoder_M1_1 = PID_Data_Buffer.Err_ENcoder_M1_1;
    pid_msg.Err_ENcoder_M1_2 = PID_Data_Buffer.Err_ENcoder_M1_2;
    pid_msg.Err_ENcoder_M1_3 = PID_Data_Buffer.Err_ENcoder_M1_3;
    pid_msg.Err_ENcoder_M1_4 = PID_Data_Buffer.Err_ENcoder_M1_4;
    pid_msg.UP_REF_M2 = PID_Data_Buffer.UP_REF_M2;
    pid_msg.UI_REF_M2 = PID_Data_Buffer.UI_REF_M2;
    pid_msg.Err_ENcoder_M2_1 = PID_Data_Buffer.Err_ENcoder_M2_1;
    pid_msg.Err_ENcoder_M2_2 = PID_Data_Buffer.Err_ENcoder_M2_2;
    pid_msg.Err_ENcoder_M2_3 = PID_Data_Buffer.Err_ENcoder_M2_3;
    pid_msg.Err_ENcoder_M2_4 = PID_Data_Buffer.Err_ENcoder_M2_4;

    WMR_ROBOT_ROS.PID_Data_Pub.publish(pid_msg);
}

void wmr_robot_ros::movement_node()
{
    wmr_robot::Movement_DATA Movement_DATA_Buffer;
    int Movement_DATA_Buffer_Size = sizeof(Movement_DATA_Buffer);
    WMR_Robot.Movement(Movement_DATA_Buffer,Movement_DATA_Buffer_Size);

    wmr_robot_hills::movement_data movement_msg;

    movement_msg.byte_1       = Movement_DATA_Buffer.byte_1;
    movement_msg.Direction_R  = Movement_DATA_Buffer.Direction_R;
    movement_msg.Direction_L  = Movement_DATA_Buffer.Direction_L;
    movement_msg.Pulse_MR_LSB = Movement_DATA_Buffer.Pulse_MR_LSB;
    movement_msg.Pulse_MR_MSB = Movement_DATA_Buffer.Pulse_MR_MSB;
    movement_msg.Pulse_ML_LSB = Movement_DATA_Buffer.Pulse_ML_LSB;
    movement_msg.Pulse_ML_MSB = Movement_DATA_Buffer.Pulse_ML_MSB;
    movement_msg.Feed_Dir_R   = Movement_DATA_Buffer.Feed_Dir_R;
    movement_msg.Feed_Dir_L   = Movement_DATA_Buffer.Feed_Dir_L;

    WMR_ROBOT_ROS.Movement_data_Pub.publish(movement_msg);
}

void wmr_robot_ros::sensor_node()               // <<<<<<<<<<<<<<<<<<<< SENSOR ROS NODE FUNCTION >>>>>>>>>>>>>>>>>>>>> //
{

    float ultrasonic[16][2];
    float ir[16][2];
    wmr_udp1.Sensor_Status_diff(ultrasonic,ir);

    sensor_msgs::PointCloud pointCloud1;
    pointCloud1.header.stamp = ros::Time::now();
    pointCloud1.header.frame_id = "ir_link";
    for(int i=1; i<=WMR_ROBOT_ROS.Ir_Sensor_Num; i++) {
        geometry_msgs::Point32 p;
        p.x = ir[i][1];
        p.y = ir[i][2];
        p.z = 0;
        pointCloud1.points.push_back(p);
    }
    WMR_ROBOT_ROS.IR_Sharp_Pub.publish(pointCloud1);

    sensor_msgs::PointCloud pointCloud2;
    pointCloud2.header.stamp = ros::Time::now();
    pointCloud2.header.frame_id = "sonar_link";
    for(int i=1; i<=WMR_ROBOT_ROS.Ultrasonic_Sensor_Num; i++) {
        geometry_msgs::Point32 p;
        p.x = ultrasonic[i][1];
        p.y = ultrasonic[i][2];
        p.z = 0;
        pointCloud2.points.push_back(p);
    }
    WMR_ROBOT_ROS.Sonar_Pub.publish(pointCloud2);
}

int main(int argc, char** argv)                 // <<<<<<<<<<<<<<<<< MAIN FUNCTION >>>>>>>>>>>>>>> //
{
    ros::init(argc, argv, "wmr_hills_ros");
    ros::NodeHandle n;

    ROS_INFO_STREAM(" .......... Connecting to robot ..........\n"
                    " Robot ip address: "<<"192.168.1.1"<<"\n"<<
                    " Sensors port: "<<"5002"<<"\n"<<
                    " PID port: "<<"10121"<<"\n"<<
                    " Odometery port: "<<"5003"<<"\n"<<
                    " Movement_Data Port: "<< "3023"<<"\n"
                    " Movement_CMD Port: "<< "6000" <<"\n"
                    " Motor_CMD Port: "<< "2222");

    WMR_ROBOT_ROS.cmd_vel_sub       = n.subscribe<geometry_msgs::Twist>("cmd_vel",1,&callbak_move_cmd);
    WMR_ROBOT_ROS.motor_vel_sub     = n.subscribe("/motor_vel",1,&callback_motor_cmd);
    WMR_ROBOT_ROS.PID_Data_Pub      = n.advertise<wmr_robot_hills::pid_data>("/pid",50);
    WMR_ROBOT_ROS.Movement_data_Pub = n.advertise<wmr_robot_hills::movement_data>("/movement",50);
    WMR_ROBOT_ROS.odometry_Data_Pub = n.advertise<nav_msgs::Odometry>("/odometry",50);
    WMR_ROBOT_ROS.Sonar_Pub         = n.advertise<sensor_msgs::PointCloud>("/sonar",50);;
    WMR_ROBOT_ROS.IR_Sharp_Pub      = n.advertise<sensor_msgs::PointCloud>("/ir",50);;

    bool init_ok;
    init_ok = WMR_Robot.init();

    if(!init_ok)
    {
        ROS_INFO_STREAM(" ..... !!! Connection Failed !!! .....");
        return 0;
    }

    ROS_INFO_STREAM(" .......... Robot is Connected ..........");

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
          WMR_ROBOT_ROS.pid_node();
          WMR_ROBOT_ROS.odometry_node();
          WMR_ROBOT_ROS.movement_node();
          WMR_ROBOT_ROS.sensor_node();

          ros::spinOnce();
          loop_rate.sleep();
    }

    return 0;
}
