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
 * @file src/wmr_udp_communication.cpp
 *
 * @brief UDP_RObot_Communication!
 *
 * @date October 2018
 **/


#include "wmr_hills.h"
#include "wmr_udp_communication.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <netdb.h>

using namespace std;

void wmr_udp_communication::sharpsensor(char *bufptr, float (&ir_Sensor)[ROWS][COLS])
{

    for(int i=0;i<Ir_Sensor_Num;i++) {
        value = bufptr[i+16];
        angle = i*24;
        ir_Sensor[i][1] = value*cos(angle*(M_PI/180)); // x
        ir_Sensor[i][2] = value*sin(angle*(M_PI/180)); // y
    }
}

void wmr_udp_communication::ultrasonicsensor(char *bufptr, float (&Ultrasonic_Sensor)[ROWS][COLS])
{
    for(int i=0;i<Ultrasonic_Sensor_Num;i++) {
        value = bufptr[i];
        angle = i*24;
        Ultrasonic_Sensor[i][1] = value*cos(angle*(M_PI/180)); // x
        Ultrasonic_Sensor[i][2] = value*sin(angle*(M_PI/180)); // y
    }
}

              /**************** Sensor Status UDP Connection  *******************************/
bool wmr_udp_communication::Sensor_Status(wmr_robot::Sensor_DATA &Sensor_DATA_Buffer,int &Sensor_DATA_Buffer_Size)
{
    int sd, rc;
    struct sockaddr_in serveraddr, clientaddr;
    socklen_t clientaddrlen = sizeof(clientaddr);
    int serveraddrlen = sizeof(serveraddr);
    if((sd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
    perror("UDP server - socket(Sensor Status) error");
    exit(-1);
    }
    else

    /* bind to address */
    memset(&serveraddr, 0x00, serveraddrlen);
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(Sensors_Data_Port);
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
    if((rc = bind(sd, (struct sockaddr *)&serveraddr, serveraddrlen)) < 0)
    {
    perror("UDP server - bind(Sensor Status) error");
    close(sd);
    exit(-1);
    }
    else

    rc = recvfrom(sd, (char*)&Sensor_DATA_Buffer, Sensor_DATA_Buffer_Size, 0, (struct sockaddr *)&clientaddr, &clientaddrlen);
    if(rc < 0)
    {
    perror("UDP Server - recvfrom(Sensor Status) error");
    close(sd) ;
    exit(-1);
    }
    else

    close(sd);
    return true;
}

              /**************** Another Sensor Status UDP Connection  ***********************/
bool wmr_udp_communication::Sensor_Status_diff(float (&ultrasonic)[ROWS][COLS],float (&ir)[ROWS][COLS])
{
    int sd, rc;
    struct sockaddr_in serveraddr, clientaddr;
    socklen_t clientaddrlen = sizeof(clientaddr);
    int serveraddrlen = sizeof(serveraddr);

    char buffer[100];
    char *bufptr = buffer;

    int buflen = sizeof(bufptr);
    if((sd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
    perror("UDP server - socket(Another Sensor Status) error");
    exit(-1);
    }
    else

    /* bind to address */
    memset(&serveraddr, 0x00, serveraddrlen);
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(Sensors_Data_Port);
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
    if((rc = bind(sd, (struct sockaddr *)&serveraddr, serveraddrlen)) < 0)
    {
    perror("UDP server - bind(Another Sensor Status) error");
    close(sd);
    exit(-1);
    }
    else

    rc = recvfrom(sd, bufptr, buflen, 0, (struct sockaddr *)&clientaddr, &clientaddrlen);
    if(rc < 0)
    {
    perror("UDP Server - recvfrom(Another Sensor Status) error");
    close(sd) ;
    exit(-1);
    }
    else

    sharpsensor(buffer,ir);
    ultrasonicsensor(buffer,ultrasonic);

    close(sd);
    return true;
}

              /**************** PID Status UDP Connection  **********************************/
bool wmr_udp_communication::PID_Status(wmr_robot::PID_DATA &PID_Data_Buffer,int &PID_Data_Buffer_Size)
{
        int sd, rc;
        struct sockaddr_in serveraddr, clientaddr;
        socklen_t clientaddrlen = sizeof(clientaddr);
        int serveraddrlen = sizeof(serveraddr);

        if((sd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        {
        perror("UDP server - socket(PID Status) error");
        exit(-1);
        }
        else

        /* bind to address */
        memset(&serveraddr, 0x00, serveraddrlen);
        serveraddr.sin_family = AF_INET;
        serveraddr.sin_port = htons(PID_Data_Port);
        serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
        if((rc = bind(sd, (struct sockaddr *)&serveraddr, serveraddrlen)) < 0)
        {
        perror("UDP server - bind(PID Status) error");
        close(sd);
        exit(-1);
        }
        else

        rc = recvfrom(sd, (char*)&PID_Data_Buffer, PID_Data_Buffer_Size, 0, (struct sockaddr *)&clientaddr, &clientaddrlen);
        if(rc < 0)
        {
        perror("UDP Server - recvfrom(PID Status) error");
        close(sd) ;
        exit(-1);
        }
        else

        close(sd);
        return true;
}

              /**************** Odometry Status UDP Connection  ******************************/
bool wmr_udp_communication::Odometery_Status(wmr_robot::Odometery_DATA &Odometery_DATA_Buffer,int &Odometery_DATA_Buffer_Size)
{
        int sd, rc;
        struct sockaddr_in serveraddr, clientaddr;
        socklen_t clientaddrlen = sizeof(clientaddr);
        int serveraddrlen = sizeof(serveraddr);

        if((sd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        {
        perror("UDP server - socket(Odometry Status) error");
        exit(-1);
        }
        else

        /* bind to address */
        memset(&serveraddr, 0x00, serveraddrlen);
        serveraddr.sin_family = AF_INET;
        serveraddr.sin_port = htons(Odometery_Data_Port);
        serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
        if((rc = bind(sd, (struct sockaddr *)&serveraddr, serveraddrlen)) < 0)
        {
        perror("UDP server - bind(Odometry Status) error");
        close(sd);
        exit(-1);
        }
        else

        rc = recvfrom(sd, (char*)&Odometery_DATA_Buffer, Odometery_DATA_Buffer_Size, 0, (struct sockaddr *)&clientaddr, &clientaddrlen);
        if(rc < 0)
        {
        perror("UDP Server - recvfrom(Odometry Status) error");
        close(sd) ;
        exit(-1);
        }
        else

        close(sd);
        return true;
}

              /**************** Movement Data UDP Connection  ********************************/
bool wmr_udp_communication::Movement_Data(wmr_robot::Movement_DATA &Movement_DATA_Buffer,int &Movement_DATA_Buffer_Size)
{
    int sd, rc;
    struct sockaddr_in serveraddr, clientaddr;
    socklen_t clientaddrlen = sizeof(clientaddr);
    int serveraddrlen = sizeof(serveraddr);

    if((sd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
    perror("UDP server - socket(Movement Data) error");
    exit(-1);
    }
    else

    /* bind to address */
    memset(&serveraddr, 0x00, serveraddrlen);
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(Movement_Data_Port);
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
    if((rc = bind(sd, (struct sockaddr *)&serveraddr, serveraddrlen)) < 0)
    {
    perror("UDP server - bind(Movement Data) error");
    close(sd);
    exit(-1);
    }
    else

    rc = recvfrom(sd, (char*)&Movement_DATA_Buffer, Movement_DATA_Buffer_Size, 0, (struct sockaddr *)&clientaddr, &clientaddrlen);
    if(rc < 0)
    {
    perror("UDP Server - recvfrom(Movement Data) error");
    close(sd) ;
    exit(-1);
    }
    else

    close(sd);
    return true;
}

              /**************** Movement CMD UDP Connection  *********************************/
bool wmr_udp_communication::Movement_CMD(wmr_robot::Movement_cmd &Movement_cmd_BUFFER,int &Movement_cmd_BUFFER_Size)
{
    int sd, rc;
    struct sockaddr_in serveraddr, clientaddr;
    socklen_t clientaddrlen = sizeof(clientaddr);
    int serveraddrlen = sizeof(serveraddr);

    /* get a socket descriptor */
    if((sd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
    perror("UDP server - socket(Movement CMD) error");
    exit(-1);
    }
    else

    /* bind to address */
    memset(&serveraddr, 0x00, serveraddrlen);
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(Movement_CMD_Port);
   // serveraddr.sin_addr.s_addr = inet_addr("192.168.1.20"); /* set destination IP number */
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);

    clientaddr.sin_family = AF_INET;
    clientaddr.sin_port = htons(Movement_CMD_Port);
    clientaddr.sin_addr.s_addr = inet_addr("192.168.1.1");

    if((rc = bind(sd, (struct sockaddr *)&serveraddr, serveraddrlen)) < 0)
    {
    perror("UDP server - bind(Movement CMD) error");
    close(sd);
    exit(-1);
    }
    else

    rc = sendto(sd, (char*)&Movement_cmd_BUFFER, Movement_cmd_BUFFER_Size, 0, (struct sockaddr *)&clientaddr, clientaddrlen);
    if(rc < 0)
     {
      perror("UDP server - sendto(Movement CMD) error");
      close(sd) ;
      exit(-1);
     }
     else

    close(sd);
    return true;
}

              /**************** Motor CMD UDP Connection  ************************************/
bool wmr_udp_communication::Motor_CMD(wmr_robot::Motor_cmd &Motor_cmd_BUFFER,int &Motor_cmd_BUFFER_Size)
{
    int sd, rc;
    struct sockaddr_in serveraddr, clientaddr;
    socklen_t clientaddrlen = sizeof(clientaddr);
    int serveraddrlen = sizeof(serveraddr);

    /* get a socket descriptor */
    if((sd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
    perror("UDP server - socket(Motor CMD) error");
    exit(-1);
    }
    else

    /* bind to address */
    memset(&serveraddr, 0x00, serveraddrlen);
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(Motor_Command_Port);
   // serveraddr.sin_addr.s_addr = inet_addr("192.168.1.20"); /* set destination IP number */
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);

    clientaddr.sin_family = AF_INET;
    clientaddr.sin_port = htons(Motor_Command_Port);
    clientaddr.sin_addr.s_addr = inet_addr("192.168.1.1");

    if((rc = bind(sd, (struct sockaddr *)&serveraddr, serveraddrlen)) < 0)
    {
    perror("UDP server - bind(Motor CMD) error");
    close(sd);
    exit(-1);
    }
    else

    rc = sendto(sd, (char*)&Motor_cmd_BUFFER, Motor_cmd_BUFFER_Size, 0, (struct sockaddr *)&clientaddr, clientaddrlen);
    if(rc < 0)
     {
      perror("UDP server - sendto(Motor CMD) error");
      close(sd) ;
      exit(-1);
     }
     else

    close(sd);
    return true;
}
