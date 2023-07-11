#include "bb_uart.h"

int bytebeam_usb_serial_send(void *data, size_t size)
{
    int ret_val = 0;

    ret_val = nwy_usb_serial_send(data, size);

    return ret_val;
}
