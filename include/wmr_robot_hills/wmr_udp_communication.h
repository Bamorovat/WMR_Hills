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
 * @file /include/wmr_robot_hills/wmr_udp_communication.h
 *
 * @brief UDP_RObot_Communication!
 *
 * @date October 2018
 **/

#ifndef WMR_UDP_COMMUNICATION
#define WMR_UDP_COMMUNICATION

#include "wmr_hills.h"
#include <stdint.h>

typedef uint8_t Byte;

class wmr_udp_communication{
protected:

    #define Sensors_Data_Port   5002   //Robot to Pc
    #define PID_Data_Port       10121  //Robot to Pc
    #define Odometery_Data_Port 5003   //Robot to Pc
    #define Movement_Data_Port  3023   //Robot to Pc
    #define Movement_CMD_Port   6000   //PC to Robot
    #define Motor_Command_Port  2222   //PC to Robot

public:

    #define ROWS 16
    #define COLS 2

    float angle;
    static const int Ir_Sensor_Num = 16;
    static const int Ultrasonic_Sensor_Num = 16;
    Byte value;

public:

    bool Sensor_Status(wmr_robot::Sensor_DATA &Sensor_DATA_Buffer, int &Sensor_DATA_Buffer_Size);
    bool Sensor_Status_diff(float (&ultrasonic)[ROWS][COLS], float (&ir)[ROWS][COLS]);
    bool PID_Status(wmr_robot::PID_DATA &PID_Data_Buffer, int &PID_Data_Buffer_Size);
    bool Odometery_Status(wmr_robot::Odometery_DATA &Odometery_DATA_Buffer, int &Odometery_DATA_Buffer_Size);
    bool Movement_Data(wmr_robot::Movement_DATA &Movement_DATA_Buffer, int &Movement_DATA_Buffer_Size);
    bool Movement_CMD(wmr_robot::Movement_cmd &Movement_cmd_BUFFER, int &Movement_cmd_BUFFER_Size);
    bool Motor_CMD(wmr_robot::Motor_cmd &Motor_cmd_BUFFER, int &Motor_cmd_BUFFER_Size);
    void sharpsensor(char *bufptr, float (&ir_Sensor)[ROWS][COLS]);
    void ultrasonicsensor(char *bufptr, float (&Ultrasonic_Sensor)[ROWS][COLS]);
};

#endif // WMR_UDP_COMMUNICATION

