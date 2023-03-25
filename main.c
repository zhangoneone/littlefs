#include <stdio.h>
#include "lfs_port.h"
#include <stdlib.h>

void main()
{
    int32_t res = 0;
    lfs_init();
    while (1) {
        res = lfs_testcase();
        printf("result:%d\n", res);
        usleep(1000*1000);
    }

}