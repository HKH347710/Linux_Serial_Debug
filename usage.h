#ifndef USAGE_H
#define USAGE_H
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <stdio.h>

using namespace std;
class Usage
{
public:
    Usage();
    void print_usage(FILE * stream, char *app_name, int exit_code);
    static char *short_options;
    static struct option long_options[];
};

#endif // USAGE_H
