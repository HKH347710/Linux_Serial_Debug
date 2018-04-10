#include "usage.h"
using namespace std;

char * Usage::short_options = "hd:b:D:s:p:R:";
struct option Usage::long_options[] =
{
    {"help", 0, NULL, 'h'},
    {"dev", 1, NULL, 'd'},
    {"baud_rate", 1, NULL, 'b'},
    {"data_bits", 1, NULL, 'D'},
    {"stop_bits", 1, NULL, 's'},
    {"parity", 1, NULL, 'p'},
    {"rs485", 1, NULL, 'R'},
    {NULL, 0, NULL, 0},
};

Usage::Usage(){
}
void Usage::print_usage(FILE * stream, char *app_name, int exit_code)
{
    fprintf(stream, "Usage: %s [options]\n", app_name);
    fprintf(stream,
            " -h --help                    Display this usage information.\n"
            " -d --dev <device_file>       Use <device_file> as serial device.\n"
            "                              The default device file is '/dev/ttyS0'\n"
            " -b --baud_rate <baud_rate>   Use <baud_rate> as baud rate.\n"
            "                              The default baud rate is '115200'\n"
            " -D --data_bits <data_bits>   Use <data_bits> as data bits .\n"
            "                              The default data bits is '8'\n"
            " -s --stop_bits <stop_bits>   Use <stop_bits> as stop bits.\n"
            "                              The default stop bits is '1'\n"
            " -p --parity <parity>         Use <parity> as parity.\n"
            "                              The default parity is 'n'\n"
            " -R --rs485 <ENABLE>          Use <E> to eanble 485 mode.\n"
            "                              The default 485 mode is 'disable'D'\n");
    exit(exit_code);
}
