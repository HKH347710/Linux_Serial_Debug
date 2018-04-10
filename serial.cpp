#include "serial.h"
#include <pthread.h>

#ifndef TIOCSRS485
    #define TIOCSRS485     0x542F
#endif

#ifndef TIOCGRS485
    #define TIOCGRS485     0x542E
#endif
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

using namespace std;

Serial::Serial(){}

bool Serial::Open_Port(char *port)
{
    cout<<"open_port"<<port << endl;
    S_fd = open(port, O_RDWR | O_NOCTTY |O_NDELAY);
    if(S_fd == -1)
    {
        cout<<"Can't Open Serial Port" << endl;
        return (false);
    }
    else
        cout<<"Open serial port success!"<< endl;
    return (true);

}

bool Serial::Set_Port_Para(int speed, int databits, int stopbits, char parity)
{
    int fd = S_fd;
    int speed_arr[] = {B57600, B115200, B230400, B38400, B19200, B9600, B4800, B2400, B1200, B300, B38400, B19200, B9600, B4800, B2400, B1200, B300};
        int name_arr[] = {57600, 115200, 230400, 38400, 19200, 9600, 4800, 2400, 1200, 300, 38400, 19200, 9600, 4800, 2400, 1200, 300};
        int i;
        struct termios Opt;
        //获取与终端相关的参数，并将获得的信息保存在Opt变量中
        if(tcgetattr(fd, &Opt) != 0)
        {
            cout<<("tcgetattr fd");
            return (false);
        }

        if(tcgetattr(fd, &tty_oldattr) != 0)
        {
            cout<<("tcgetattr fd");
            return (false);
        }

        /*set up other port settings*/
        Opt.c_cflag|=CREAD|CLOCAL;
        Opt.c_lflag&=(~(ICANON|ECHO|ECHOE|ECHOK|ECHONL|ISIG));
        Opt.c_iflag&=(~(INPCK|IGNPAR|PARMRK|ISTRIP|ICRNL|IXANY));
        Opt.c_oflag&=(~OPOST);
        Opt.c_cc[VMIN]=0;
        Opt.c_cc[VINTR] = _POSIX_VDISABLE;
        Opt.c_cc[VQUIT] = _POSIX_VDISABLE;
        Opt.c_cc[VSTART] = _POSIX_VDISABLE;
        Opt.c_cc[VSTOP] = _POSIX_VDISABLE;
        Opt.c_cc[VSUSP] = _POSIX_VDISABLE;

        //识别波特率
        for(i = 0; i < sizeof(speed_arr)/sizeof(int); i++)
        {
            if(speed == name_arr[i])
            {
                //刷清输入输出队列
                tcflush(fd, TCIOFLUSH);
                //设置串口通信速率
#ifdef CBAUD

                Opt.c_cflag &= ~CBAUD;
                Opt.c_cflag |= speed_arr[i];
#else
                cfsetispeed(&Opt, speed_arr[i]);
                cfsetospeed(&Opt, speed_arr[i]);

#endif
            }

        }
        //使设置即刻生效
        if(tcsetattr(fd, TCSANOW, &Opt) != 0)
        {
            perror("tcsetattr fd baudrate");
            return (false);
        }
        tcflush(fd, TCIOFLUSH);

        //去除数据位中的位掩码，然后才能重新设置
            Opt.c_cflag &= ~CSIZE;
            //忽略所有调制解调器的状态行，启用字符接收器
            Opt.c_cflag |= (CLOCAL | CREAD);
            //设置数据位
            switch(databits)
            {
                case 7:
                    Opt.c_cflag |= CS7;
                    break;
                case 8:
                    Opt.c_cflag |= CS8;
                    break;
                default:
                    fprintf(stderr,"Unsupported data size\n");
                    return (false);
            }
            //使设置即刻生效
            if(tcsetattr(fd, TCSANOW, &Opt) != 0)
            {
                perror("tcsetattr fd baudrate");
                return (false);
            }
            tcflush(fd, TCIOFLUSH);
            //设置校验位
            switch(parity)
            {
                case 'n':
                case 'N':
                    //关闭奇偶校验码的生成和检测功能
                    Opt.c_cflag &= ~PARENB;
                    //对接收到的字符关闭奇偶校验
                    Opt.c_iflag &= ~INPCK;
                    break;
                case 'o':
                case 'O':
                    //PARODD使用奇校验
                    Opt.c_cflag |= (PARODD | PARENB);
                    Opt.c_iflag |= INPCK;
                    break;
                case 'e':
                case 'E':
                    Opt.c_cflag |= PARENB;
                    //	转换为偶效验
                    Opt.c_cflag &= ~PARODD;
                    Opt.c_iflag |= INPCK;
                    break;
                case 's':
                case 'S':
                    //清除校验位
                    Opt.c_cflag &= ~PARENB;
                    //每个字符使用一个停止位
                    Opt.c_cflag &= ~CSTOPB;
                    break;
                default:
                    fprintf(stderr,"Unsupported parity\n");
                    return (false);

            }
            if(tcsetattr(fd, TCSANOW, &Opt) != 0)
            {
                perror("tcsetattr fd baudrate");
                return (false);
            }
            tcflush(fd, TCIOFLUSH);

            //设置停止位
            switch(stopbits)
            {
                case 1:
                    Opt.c_cflag &= ~CSTOPB;
                    break;
                case 2:
                    Opt.c_cflag |= CSTOPB;
                    break;
                default:
                    fprintf(stderr,"Unsupported stop bits\n");
                    return (false);
            }

            if(tcsetattr(fd, TCSANOW, &Opt) != 0)
            {
                perror("tcsetattr fd baudrate");
                return (false);
            }
            tcflush(fd, TCIOFLUSH);
            //针对不是开发终端，使用串口传输数据，不需要处理数据，使用原始模式
            Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            Opt.c_oflag &= ~OPOST;

            /* Set input parity option */
            if (parity != 'n')
                Opt.c_iflag |= INPCK;

            /* Set flow contral option */
            Opt.c_lflag &= ~CRTSCTS;
            Opt.c_iflag &= (~(IXON|IXOFF|IXANY));

            Opt.c_cc[VTIME] = 20; // 2 seconds
            Opt.c_cc[VMIN] = 0;

            /* Update the Opt and do it NOW */
            if (tcsetattr(fd,TCSANOW,&Opt) != 0)
            {
                perror("tcgetattr fd when setup serial 2");
                return (false);
            }
             tcflush(fd,TCIOFLUSH);
           return true;
}

void Serial::Show_Port_Para()
{
    int fd = Serial::S_fd;
    struct termios Npt;
    if(tcgetattr(fd, &Npt) != 0)
    {
        perror("tcgetattr fd New termios");
        return;
    }

    cout<<"baud rate is "<<(Npt.c_cflag & CBAUD)<<endl; //band rate B115200 is 4098
    cout<<"data bit is "<<(Npt.c_cflag & CSIZE)<<endl;  //data bits SC8 is 48
    cout<<"stop bit is "<<(Npt.c_cflag & CSTOPB)<<endl; //stop bit is 0
    cout<<"parity is "<<(Npt.c_cflag & PARENB)<<endl;   //no parity is 0
    cout<<"flow control is "<<(Npt.c_cflag & CRTSCTS)<<endl; //no flow contral is 0
}

int Serial::Receive_Port(char *receive_buf)
{
    int s_NRead = read(S_fd, receive_buf, MAXLINE);
    return s_NRead;
}

int Serial::Send_Port(char *send_buf)
{
    int s_NWrite = write(S_fd, send_buf, strlen(send_buf));
    return s_NWrite;
}


int Serial::Rs485_Enable(const RS485_ENABLE_t enable)
{
    struct serial_rs485 rs485conf;
    int res;

    res = ioctl(S_fd, TIOCGRS485, &rs485conf);
    if(res < 0)
    {
        perror("ioctl error on getting 485 configure:");
        close(S_fd);
        return res;
    }

    if(enable)
    {
        rs485conf.flags |= SER_RS485_ENABLED;
    }
    else
    {
        rs485conf.flags &= ~(SER_RS485_ENABLED);
    }

    rs485conf.delay_rts_before_send = 0x00000004;

    res = ioctl(S_fd, TIOCSRS485, &rs485conf);
    if(res < 0)
    {
        perror("ioctl error on setting 485 configure:");
        close(S_fd);
    }

    return res;
}


void Serial::Close_Port()
{
    if(tcsetattr(S_fd, TCSANOW, &tty_oldattr) == -1)
    {
        perror("Restitute error: ");
    }

    cout<<"close port " << "with fd: "<<S_fd<< endl;
    close(S_fd);

}



int Serial::Rs485_Write(const char *tx_buf, const int num)
{
    int res;

    /* Write data to rs485 */
    pthread_mutex_lock(&mutex);

    if ((res = write(S_fd, tx_buf, strlen(tx_buf))) == -1) {
        fprintf(stderr, "Write error on %s\n", strerror(errno));
    }

    pthread_mutex_unlock(&mutex);

    return res;
}

int Serial::Rs485_Read(char *rx_buf, const int num)
{
    int res = 0;

    /* Get bytes number in buffer */
    while (res < 1) {
        if (ioctl(S_fd, FIONREAD, &res) < 0) {
            perror("Error on get bytes number in buffer");
            return errno;
        } else {
            usleep(100000);
        }
    }

    /* Read data from rs485 */
    pthread_mutex_lock(&mutex);

    (res > num) ? (res = num) : res;
    if ((res = read(S_fd, rx_buf, res)) == -1) {
        fprintf(stderr, "Read error on %s\n", strerror(errno));
    } else {
        rx_buf[res] = '\0';
    }

    pthread_mutex_unlock(&mutex);

    return res;
}
















