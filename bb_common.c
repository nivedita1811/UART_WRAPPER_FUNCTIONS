#ifndef BB_COMMON
#define BB_COMMON

#include "bytebeam.h"
#include "bb_common.h"
#include "bb_usb_serial.h"

void bytebeam_ext_echo(char *fmt, ...)
{
    static char echo_str[NWY_EXT_SIO_RX_MAX];
    static bytebeam_osiMutex_t *echo_mutex = NULL;
    va_list a;
    int i, size;

    if (NULL == echo_mutex)
        echo_mutex = bytebeam_create_mutex();
    if (NULL == echo_mutex)
        return;
    bytebeam_lock_mutex(echo_mutex, 0);
    va_start(a, fmt);
    vsnprintf(echo_str, NWY_EXT_SIO_RX_MAX, fmt, a);
    va_end(a);
    size = strlen((char *)echo_str);
    i = 0;
    while (1)
    {
        int tx_size;

        tx_size = bytebeam_usb_serial_send((char *)echo_str + i, size - i);
        if (tx_size <= 0)
            break;
        i += tx_size;
        if ((i < size))
            bytebeam_sleep(10);
        else
            break;
    }
    bytebeam_unlock_mutex(echo_mutex);
}
