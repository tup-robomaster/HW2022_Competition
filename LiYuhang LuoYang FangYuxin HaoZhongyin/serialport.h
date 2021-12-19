#ifndef SERIALPORT_H
#define SERIALPORT_H
/**
 *@class  SerialPort
 *@brief  set serialport,recieve and send
 *@param  int fd
 */
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <iostream>
#include "CRC_Check.h"

using namespace std;


#define TRUE 1
#define FALSE 0

//模式
#define CmdID0 0x00; //关闭视觉
#define CmdID1 0x01; //识别红色
#define CmdID2 0x02; //识别蓝色
#define CmdID3 0x03; //小幅
#define CmdID4 0x04; //大幅

//串口的相关参数
#define BAUDRATE 115200//波特率
#define UART_DEVICE "/dev/ttyUSB0"//默认的串口名称

//C_lflag
#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)

//字节数为4的结构体
typedef union
{
    float f;
    signed char c[1];
}floatuchar;

//字节数为2的uchar数据类型
typedef union
{
    int16_t d;
    unsigned char c[0];
} char1;



//地图识别地方机器人信息
typedef struct
{
char1 data_length;

} Mapdata;


class SerialPort
{
private:
    int fd; //串口号
    int speed, databits, stopbits, parity;
    unsigned char rdata[255]; //raw_data
    unsigned char Tdata[4];  //transfrom data

	void set_Brate();
	int set_Bit();
public:
    SerialPort();
    SerialPort(char *);
    bool initSerialPort();
    bool get_Mode(int &mode, int &sentry_mode, int &base_mode);
	void TransformData(const Mapdata &data); //主要方案
	void send();
	void closePort();
	void TransformDataFirst(int Xpos, int Ypos, int dis);//方案1

};

#endif //SERIALPORT_H

