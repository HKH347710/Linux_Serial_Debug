#include <iostream>
#include "serial.h"
#include "usage.h"
#include <pthread.h>
#define BUF_SIZE  128
#define END_FLAG  0x23
using namespace std;
Serial serial;
void rs485_test();

int main(int argc, char *argv[])
{
    int count = 1;
    char *Port = "/dev/ttyS1";
    int Port_Speed = 115200;
    int Port_Data_Bits = 8;
    int Port_Stop_Bits = 1;
    char Port_Parity = 'n';
    char rs485 = '\0';
    int RS485;


    int next_option;
    Usage usage;
    do
    {
       next_option = getopt_long(argc, argv,usage.short_options, usage.long_options,NULL);
       switch(next_option)
       {
       case 'h':
           usage.print_usage(stdout, argv[0], EXIT_SUCCESS);
       case 'd':
           Port = optarg;
           break;
       case 'b':
           Port_Speed = atoi(optarg);
           break;
       case 'D':
           Port_Data_Bits = atoi(optarg);
           break;
       case 's':
           Port_Stop_Bits = atoi(optarg);
           break;
       case 'p':
           Port_Parity = optarg[0];
           break;
       case 'R':
           rs485 = optarg[0];
           break;
       case '?':
           usage.print_usage(stderr, argv[0], EXIT_FAILURE);
       case -1:
           break;
       default:
           abort();
       }
    }
    while(next_option != -1);

    cout << "dev: " << Port << endl;
    cout << "Port_Speed: " << Port_Speed << endl;
    cout << "Port_Data_Bits: " << Port_Data_Bits << endl;
    cout << "Port_Stop_Bits: " << Port_Stop_Bits << endl;
    cout << "Port_Parity: " << Port_Parity << endl;
    cout << "rs485: "<< rs485 << endl;

    if(rs485 == 'E' || rs485 == 'D')
    {
        RS485 = 1;
    }
    else
    {
        RS485 = 0;
    }
    char *receive_buf = new char[MAXLINE];
    char *send_buf = new char[MAXLINE];

    memset(receive_buf, 0, MAXLINE);
    memset(send_buf, 0, MAXLINE);

    cout<<"serial process start to run"<<endl;
    cout << "-----------------" << endl;
    cout<<"start to open port: " << Port << endl;
    serial.Open_Port(Port);
    cout<<"set serial port para"<<endl;
    serial.Set_Port_Para(Port_Speed, Port_Data_Bits, Port_Stop_Bits, Port_Parity);
    serial.Show_Port_Para();
    if(!serial.Open_Port(Port))
    {
        exit(-1);
    }

    if(RS485)
    {
        if(rs485 == 'E')
        {
            if(serial.Rs485_Enable(ENABLE) < 0)
            {
                cerr << "485 mode enable fail!"<<endl;
                exit (-1);

            }
            else
            {
                cout << "485 mode enable success!"<<endl;
            }
        }
        else
        {
            if(serial.Rs485_Enable(DISABLE) < 0)
            {
                cerr << "485 mode disable fail!"<<endl;
                exit (-1);

            }
            else
            {
                cout << "485 mode disable success!"<<endl;
            }
        }
    }
    cout<<"set serial port para"<<endl;
    serial.Set_Port_Para(Port_Speed, Port_Data_Bits, Port_Stop_Bits, Port_Parity);
//    serial.Show_Port_Para();

    rs485_test();

    serial.Close_Port();
    return 0;
}

void *recv_thread(void *arg)
{
    char rx_buf[BUF_SIZE] = { 0 };
    int  msg_ind = 0;;
    int  res;

    while (1) {
        if ((res = serial.Rs485_Read(&rx_buf[msg_ind], BUF_SIZE - msg_ind - 1)) > 0) {
            msg_ind += res;
            if ((rx_buf[msg_ind - 1] == END_FLAG) ||
                (msg_ind == (BUF_SIZE - 1))) {
                rx_buf[msg_ind - 1] = '\0';
                printf("Receive %d bytes msg: %s\n", msg_ind - 1, rx_buf);
                memset(rx_buf, 0, BUF_SIZE);
                msg_ind = 0;
            }
        }
    }
}
void rs485_test()
{
    char tx_buf[BUF_SIZE] = { 0 };
    pthread_t id;
    int res;

    /* Create a thread for receive message */
    pthread_create(&id, NULL, recv_thread, NULL);

    /* Send message */
    while (1)
    {
        memset(tx_buf, 0, BUF_SIZE);
        gets(tx_buf);
        tx_buf[strlen(tx_buf)] = 0x23;
        if ((res = serial.Rs485_Write(tx_buf, strlen(tx_buf))) > 0)
        { // Send to slave
            tx_buf[strlen(tx_buf) - 1] = '\0';
            printf("Send %d bytes msg: %s\n", res - 1, tx_buf);
        }
    }
}



