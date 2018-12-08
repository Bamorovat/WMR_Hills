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
 * @file /include/wmr_robot_hills/wmr_hills.h
 *
 * @brief Robot_Communication!
 *
 * @date December 2018
 **/

#ifndef WMR_HILLS
#define WMR_HILLS

#include <stdint.h>
#include <iostream>
#include <algorithm>

typedef uint8_t Byte;


//namespace wmr{
class wmr_robot{

public:

    // #pragma pack(push, 1) // exact fit - no padding
    struct __attribute__((packed))Sensor_DATA{

        public:
        Byte Ultrasonic_1;
        Byte Ultrasonic_2;
        Byte Ultrasonic_3;
        Byte Ultrasonic_4;
        Byte Ultrasonic_5;
        Byte Ultrasonic_6;
        Byte Ultrasonic_7;
        Byte Ultrasonic_8;
        Byte Ultrasonic_9;
        Byte Ultrasonic_10;
        Byte Ultrasonic_11;
        Byte Ultrasonic_12;
        Byte Ultrasonic_13;
        Byte Ultrasonic_14;
        Byte Ultrasonic_15;
        Byte Ultrasonic_16;
        Byte IR_Sharp_1;
        Byte IR_Sharp_2;
        Byte IR_Sharp_3;
        Byte IR_Sharp_4;
        Byte IR_Sharp_5;
        Byte IR_Sharp_6;
        Byte IR_Sharp_7;
        Byte IR_Sharp_8;
        Byte IR_Sharp_9;
        Byte IR_Sharp_10;
        Byte IR_Sharp_11;
        Byte IR_Sharp_12;
        Byte IR_Sharp_13;
        Byte IR_Sharp_14;
        Byte IR_Sharp_15;
        Byte IR_Sharp_16;
    };

    //  #pragma pack(pop) //back to whatever the previous packing mode was


    // #pragma pack(push, 1) // exact fit - no padding
    struct __attribute__((packed))PID_DATA{

        public:
        float UP_REF_M1;
        float UI_REF_M1;
        Byte Err_ENcoder_M1_1;
        Byte Err_ENcoder_M1_2;
        Byte Err_ENcoder_M1_3;
        Byte Err_ENcoder_M1_4;
        float UP_REF_M2;
        float UI_REF_M2;
        Byte Err_ENcoder_M2_1;
        Byte Err_ENcoder_M2_2;
        Byte Err_ENcoder_M2_3;
        Byte Err_ENcoder_M2_4;
    };
    //  #pragma pack(pop) //back to whatever the previous packing mode was

    // #pragma pack(push, 1) // exact fit - no padding
    struct __attribute__((packed))Odometery_DATA{

        public:
        Byte RES_1;
        Byte RES_2;
        Byte RES_3;
        Byte RES_4;
        Byte RES_5;
        Byte RES_6;
        Byte RES_7;
        Byte RES_8;
        Byte RES_9;
        float Result_Teta;
        float Result_X;
        float Result_Y;
        float Speed_Motor_R;
        float Speed_Motor_L;
    };
    //  #pragma pack(pop) //back to whatever the previous packing mode was

    // #pragma pack(push, 1) // exact fit - no padding
    struct __attribute__((packed))Movement_DATA{

        public:
        Byte byte_1;
        Byte Direction_R;
        Byte Direction_L;
        Byte Pulse_MR_LSB;
        Byte Pulse_MR_MSB;
        Byte Pulse_ML_LSB;
        Byte Pulse_ML_MSB;
        Byte Feed_Dir_R;
        Byte Feed_Dir_L;
    };
    //  #pragma pack(pop) //back to whatever the previous packing mode was



    // #pragma pack(push, 1) // exact fit - no padding
    struct __attribute__((packed))Movement_cmd{

        public:
        const Byte Mode = 0x04;
       // Byte Mode;
        float Velocity_Y;
        float Velocity_W;
    };
    //  #pragma pack(pop) //back to whatever the previous packing mode was


    // #pragma pack(push, 1) // exact fit - no padding
    struct __attribute__((packed))Motor_cmd{

        public:
        Byte Mode; // 0x00-0x02
        Byte DirR; //0x00-0x02;
        Byte DirL;  //0x00-0x02;
        uint16_t SpeedR; //1-1700;
        uint16_t SpeedL; //1-1700;
    };
  //  #pragma pack(pop) //back to whatever the previous packing mode was


public:

    wmr_robot();
    virtual ~wmr_robot();
    bool init();
    void sensor_diff();
    void robot_status( char *bufptr, int len);
    void sharpsensor(wmr_robot::Sensor_DATA &Sensor_DATA_Buffer,int Sensor_DATA_Buffer_Size);
    void ultrasonicsensor(char *bufptr, int Ultra_Sensor_Num);
    void Sensors(wmr_robot::Sensor_DATA &Sensor_DATA_Buffer,int Sensor_DATA_Buffer_Size);
    void PID(PID_DATA &PID_Data_Buffer, int PID_Data_Buffer_Size);
    void Odometry(Odometery_DATA &Odometery_DATA_Buffer,int Odometery_DATA_Buffer_Size);
    void Movement(Movement_DATA &Movement_DATA_Buffer,int Movement_DATA_Buffer_Size);
    void Movement_Command(float Angular_velocity,float Linear_Velocity);
    void Motor_Command(Byte mode, Byte Dir_Right, Byte Dir_Left, uint16_t Speed_Right, uint16_t Speed_Left);



};
//}


#endif // WMR_HILLS
