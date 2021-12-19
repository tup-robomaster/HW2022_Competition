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
    unsigned char c[4];
} float2uchar;

//字节数为2的uchar数据类型
typedef union
{
    int16_t d;
    unsigned char c[2];
} int16uchar;

//用于保存目标相关角度和距离信息及瞄准情况
typedef struct
{
	float2uchar yaw_angle;//偏航角
	float2uchar pitch_angle;//俯仰角
	float2uchar dis;//目标距离
    int ismiddle;//设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，目前暂不使用，默认置0
	int isFindTarget;//当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
    int isfindDafu;
    int nearFace;
} VisionData;

//地图识别地方机器人信息
typedef struct
{
int16uchar data_length;
int16uchar target_robot_ID;
float2uchar target_position_x;
float2uchar target_position_y;
} Mapdata;


class SerialPort
{
private:
    int fd; //串口号
    int speed, databits, stopbits, parity;
    unsigned char rdata[255]; //raw_data
    unsigned char Tdata[30];  //transfrom data

void set_Brate()
{
    int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
					   B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
					  };
    int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
					  115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
					 };
    int   i;
    int   status;
    struct termios   Opt;
    tcgetattr(fd, &Opt);

	for (i = 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
	{
		if (speed == name_arr[i])
		{
			tcflush(fd, TCIOFLUSH);//清空缓冲区的内容
			cfsetispeed(&Opt, speed_arr[i]);//设置接受和发送的波特率
			cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt); //使设置立即生效

			if (status != 0)
			{
                perror("tcsetattr fd1");
                return;
            }

			tcflush(fd, TCIOFLUSH);

        }
    }
}

int set_Bit()
{
    struct termios termios_p;

	if (tcgetattr(fd, &termios_p)  !=  0)
	{
        perror("SetupSerial 1");
		return (FALSE);
    }

    termios_p.c_cflag |= (CLOCAL | CREAD);  //接受数据
	termios_p.c_cflag &= ~CSIZE;//设置数据位数

    switch (databits)
    {
	case 7:
		termios_p.c_cflag |= CS7;
		break;

	case 8:
		termios_p.c_cflag |= CS8;
		break;

	default:
		fprintf(stderr, "Unsupported data size\n");
		return (FALSE);
    }

	//设置奇偶校验位double
    switch (parity)
{
	case 'n':
	case 'N':
		termios_p.c_cflag &= ~PARENB;   /* Clear parity enable */
		termios_p.c_iflag &= ~INPCK;     /* Enable parity checking */
		break;

	case 'o':
	case 'O':
		termios_p.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
		termios_p.c_iflag |= INPCK;             /* Disnable parity checking */
		break;

	case 'e':
	case 'E':
		termios_p.c_cflag |= PARENB;     /* Enable parity */
		termios_p.c_cflag &= ~PARODD;   /* 转换为偶效验*/
		termios_p.c_iflag |= INPCK;       /* Disnable parity checking */
		break;

	case 'S':
	case 's':  /*as no parity*/
		termios_p.c_cflag &= ~PARENB;
		termios_p.c_cflag &= ~CSTOPB;
		break;

	default:
		fprintf(stderr, "Unsupported parity\n");
		return (FALSE);

    }

    /* 设置停止位*/
    switch (stopbits)
    {
	case 1:
		termios_p.c_cflag &= ~CSTOPB;
		break;

	case 2:
		termios_p.c_cflag |= CSTOPB;
		break;

	default:
		fprintf(stderr, "Unsupported stop bits\n");
		return (FALSE);

    }

    /* Set input parity option */
    if (parity != 'n')
        termios_p.c_iflag |= INPCK;

	tcflush(fd, TCIFLUSH); //清除输入缓存区
    termios_p.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    termios_p.c_cc[VMIN] = 0;  //最小接收字符
    termios_p.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input原始输入*/
	termios_p.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    termios_p.c_iflag &= ~(ICRNL | IGNCR);
    termios_p.c_oflag &= ~OPOST;   /*Output禁用输出处理*/

	if (tcsetattr(fd, TCSANOW, &termios_p) != 0) /* Update the options and do it NOW */
    {
        perror("SetupSerial 3");
        return (FALSE);
    }

    return (TRUE);
}
public:

SerialPort()
{

	fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);

    speed = BAUDRATE;
    databits = 8;
    stopbits = 1;
	parity = 'N';
}

SerialPort(char *portpath)
{

	fd = open(portpath, O_RDWR | O_NOCTTY | O_NDELAY);

    speed = BAUDRATE;
    databits = 8;
    stopbits = 1;
	parity = 'N';
}

bool initSerialPort()
{

	if (fd == -1)
	{
        perror(UART_DEVICE);
        return false;
    }

	std::cout << "Opening..." << std::endl;
    set_Brate();

	if (set_Bit() == FALSE)
	{
        printf("Set Parity Error\n");
		exit(0);
    }
    printf("Open successed\n");
    return true;
}

bool get_Mode(int &mode, int &sentry_mode, int &base_mode)
{
    int bytes;
    char *name = ttyname(fd);
    if (name != NULL)printf("device:%s\n",name);
    else printf("tty is null\n");
    int result = ioctl(fd, FIONREAD, &bytes);
    if (result == -1)return false;


    if (bytes == 0)
    {
//        cout << "缓冲区为空" << endl;
        return true;
    }

    bytes = read(fd, rdata, 22);

    if (rdata[0] == 0xA5 && Verify_CRC8_Check_Sum(rdata, 3))
    {
        //判断针头和CRC校验是否正确
        mode  = (int)rdata[1]; //通过此数据控制线程的开启	0关闭自瞄1开启自瞄2小能量机关3大能量机关
        printf("接收到的指令:%d\r\n", mode);
	//----------No use---------//    
        sentry_mode  = (int)rdata[15];
        printf("Is in sentry mode ? :%d\r\n", sentry_mode);
        base_mode  = (int)rdata[16];
        printf("Is in base mode ? :%d\r\n", base_mode);

    }
    return true;
}

void TransformData(const Mapdata &data)
{

    Tdata[0] = 0xA5;
    Tdata[1] = data.data_length.c[0];
    Tdata[2] = data.data_length.c[1];
    Tdata[3] = 0;
	Append_CRC8_Check_Sum(Tdata,5);

    Tdata[5] = data.target_robot_ID.c[0];
    Tdata[6] = data.target_robot_ID.c[1];

    Tdata[7] = data.target_position_x.c[0];
    Tdata[8] = data.target_position_x.c[1];
    Tdata[9] = data.target_position_x.c[2];
    Tdata[10] = data.target_position_x.c[3];

    Tdata[11] = data.target_position_y.c[0];
    Tdata[12] = data.target_position_y.c[1];
    Tdata[13] = data.target_position_y.c[2];
    Tdata[14] = data.target_position_y.c[3];

    Tdata[15] = 0;
    Tdata[16] = 0;
    Tdata[17] = 0;
    Tdata[18] = 0;

    Tdata[19] = 0;

	Tdata[20] = 0;
    Tdata[21] = 0;

	Append_CRC16_Check_Sum(Tdata, 24);

} 

void send()
{
	write(fd, Tdata, 24);
}

void closePort()
{
    close(fd);
}
//void TransformDataFirst(int Xpos, int Ypos, int dis);//方案1
//  int set_disp_mode(int);
//  void TransformTarPos(const VisionData &data);
};

#endif //SERIALPORT_H

