#include <stdio.h>
#include <stdlib.h> // for atof
#include "asm.h"    // your assembly function prototype

int main(int argc, char *argv[])
{
    if (argc != 4)
    {
        printf("Usage: %s <value1> <value2> <limit>\n", argv[0]);
        return 1;
    }

    double value1 = atof(argv[1]);
    double value2 = atof(argv[2]);
    double limit = atof(argv[3]);

    int result = check_limit(value1, value2, limit);

    if (result == 1)
    {
        printf("Limit exceeded (|%.2f - %.2f| > %.6f)\n",
               value1, value2, limit);
    }
    else
    {
        printf("Within acceptable range (|%.2f - %.2f| <= %.6f)\n",
               value1, value2, limit);
    }

    return 0;
}
