#include "camera.h"

int camera_confirm(int num)
{
    int count = 0;
    while (1)
    {
        if (uart2_buffer[0] == 'b')
            count++;
        if (count == 10)
            return 1;
    }
}
