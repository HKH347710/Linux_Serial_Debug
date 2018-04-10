#ifndef SERIAL_H
#define SERIAL_H

#include <string.h>
#include <iostream>

#include<stdio.h>      //标准输入输出定义
#include<stdlib.h>     //标准函数库定义
#include<unistd.h>     //Unix标准函数定义
#include<sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include<sys/stat.h>
#include<fcntl.h>      //文件控制定义
#include<termios.h>    //PPSIX终端控制定义
#include<errno.h>      //错误号定义

#include <linux/serial.h> //rs485


typedef enum {DISABLE = 0, ENABLE} RS485_ENABLE_t;


#define MAXLINE 4096

using namespace std;

class Serial
{
public:

    /*
     *函数名：Serial
     *函数功能：构造函数
     *输入参数：无
     *输出参数：无
     *涉及的全局变量：无
     *返回值：无
     *修改日志：白杰，2016/07/04， 编写该函数原型
    */
    Serial();

    /*
     *函数名：Open_Port
     *函数功能：打开指定的串口终端设备
     *输入参数：char * port 串口终端设备名称
     *输出参数：输出该串口终端设备是否成功打开
     *涉及的全局变量：S_fd
     *返回值：true 打开串口终端设备成功
             false 打开串口终端设备失败
     *修改日志：白杰，2016/07/04， 编写该函数原型
    */
    bool Open_Port(char * port);

    /*
     *函数名：Set_Port_Para
     *函数功能：设置串口终端参数
     *输入参数：int speed 波特率
              int databits 数据位
              int stopbits 停止位
              char parity 奇偶校验
     *输出参数：无
     *涉及的全局变量：S_fd
     *返回值：true 设置串口终端设备成功
             false 设置串口终端设备失败
     *修改日志：白杰，2016/07/04， 编写该函数原型
    */
    bool Set_Port_Para(int speed, int databits, int stopbits, char parity);

    /*
     *函数名：Show_Port_Para
     *函数功能：显示串口终端参数
     *输入参数：无
     *输出参数：波特率，数据位，停止位，奇偶校验
     *涉及的全局变量：S_fd
     *返回值：无
     *修改日志：白杰，2016/07/04， 编写该函数原型
    */
    void Show_Port_Para();

    /*
     *函数名：Receive_Port
     *函数功能：串口终端数据接收
     *输入参数：char *receive_buf 接收缓存区
     *输出参数：无
     *涉及的全局变量：S_fd
     *返回值：int s_NRead 接收到的字节数
     *修改日志：白杰，2016/07/04， 编写该函数原型
    */
    int Receive_Port(char *receive_buf);

    /*
     *函数名：Send_Port
     *函数功能：串口终端数据发送
     *输入参数：char *send_buf 发送缓存区
     *输出参数：无
     *涉及的全局变量：S_fd
     *返回值：int s_NWrite 发送的字节数
     *修改日志：白杰，2016/07/04， 编写该函数原型
    */
    int Send_Port(char *send_buf);

    /*
<<<<<<< Updated upstream
=======
     *函数名：Rs485_Enable
     *函数功能：开启rs485功能
     *输入参数：无
     *输出参数：
     *涉及的全局变量：
     *返回值：设置成功返回正值，设置失败返回负值
     *修改日志：白杰，2016/07/05， 编写该函数原型
    */
    int Rs485_Enable(const RS485_ENABLE_t enable);

    int Rs485_Write(const char *tx_buf, const int num);

    int Rs485_Read(char *rx_buf, const int num);

    /*
>>>>>>> Stashed changes
     *函数名：Close_Port
     *函数功能：关闭串口终端设备
     *输入参数：无
     *输出参数：无
     *涉及的全局变量：S_fd
     *返回值：无
     *修改日志：白杰，2016/07/04， 编写该函数原型
    */
    void Close_Port();//关闭串口

    int S_fd;//打开的串口文件描述符

    struct termios tty_oldattr;
};
#endif // SERIAL_H
