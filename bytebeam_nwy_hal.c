#include "neoway_api.h"
#include "nwy_bb_uart.h"

int bytebeam_uart_init(bytebeam_uart_config_t config) 
{
    int ret_val = nwy_uart_init(config.name, config.mode);
    if (ret_val < 0) 
    {
        return -1;  // Failure
    }
    return ret_val;  // Success, return the ret_val
}

bool bytebeam_uart_set_baud(uint8_t hd, uint32_t baud) 
{
    return nwy_uart_set_baud(hd, baud);
}

bool bytebeam_uart_get_baud(uint8_t hd, uint32_t* baud) 
{
    return nwy_uart_get_baud(hd, baud);
}

bool bytebeam_uart_set_parameters(uint8_t hd, uart_parity_t parity, uart_data_bits_t data_size, uart_stop_bits_t stop_size, bool flow_ctrl) 
{
    return nwy_uart_set_para(hd, parity, data_size, stop_size, flow_ctrl);
}

bool bytebeam_uart_get_parameters(uint8_t hd, uart_parity_t* parity, uart_data_bits_t* data_size, uart_stop_bits_t* stop_size, bool* flow_ctrl) 
{
    return nwy_uart_get_para(hd, parity, data_size, stop_size, flow_ctrl);
}

int bytebeam_uart_send_data_byte_in_bytes(uint8_t hd, uint8_t* data_ptr, uint32_t length, uint32_t delay_ms) 
{
    int ret_val = 0;

    for (uint32_t loop_var = 0; loop_var < length; loop_var++) 
    {
        ret_val = nwy_uart_send_data(hd, &data_ptr[loop_var], 1);

        if (ret_val != 0) 
        {
            return -1;  // Failure
        }
        bb_sleep(delay_ms);
    }
    return 0;  // Success
}

int bytebeam_uart_send_data_block(uint8_t hd, uint8_t* data_ptr, uint32_t length) 
{
    int ret_val = 0;

    ret_val = nwy_uart_send_data(hd, data_ptr, length);
    
    if (ret_val != 0) 
    {
        return -1;  // Failure
    } 
    else 
    {
        return 0;  // Success
    }
}

int bytebeam_uart_send_single_byte(uint8_t hd, uint8_t data) 
{
    int ret_val = 0;

    ret_val = nwy_uart_send_data(hd, &data, 1);

    if (ret_val != 0) 
    {
        return -1;  // Failure
    }
    return 0;  // Success
}

// Byte by byte receive function
bool bytebeam_receive_byte_by_byte(bytebeam_uart_recv_callback_t recv_cb) 
{
    // Here we just call the provided function, this will receive data byte by byte.
    return nwy_uart_reg_recv_cb((nwy_uart_recv_callback_t)recv_cb);
}

// Receive all data at once
bool bytebeam_receive_data_block(bytebeam_uart_recv_callback_t recv_cb) 
{
    bool result = true;
    char data;

    // Loop until there is no data left to read.
    while((data = nwy_uart_reg_recv_cb((nwy_uart_recv_callback_t)recv_cb)) != EOF) 
    {
        // Call the callback for each byte of data.
        if(!recv_cb(data)) 
        {
            // If the callback returns false, stop receiving and return false.
            result = false;
            break;
        }
    }

    return result;
}

// Receive single byte
bool bytebeam_receive_single_byte(bytebeam_uart_recv_callback_t recv_cb) 
{
    char data;

    // Read single byte
    data = nwy_uart_reg_recv_cb((nwy_uart_recv_callback_t)recv_cb);

    // If EOF is returned, it means no data is available.
    if(data == EOF) 
    {
        return false;
    } 
    else 
    {
        // Call the callback for the byte of data.
        if(!recv_cb(data)) 
        {
            // If the callback returns false, return false.
            return false;
        }
    }

    return true;
}

bool bytebeam_uart_register_receive_callback(uint8_t hd, bytebeam_uart_recv_callback_t recv_cb) 
{
    return nwy_uart_reg_recv_cb(hd, (nwy_uart_recv_callback_t)recv_cb);
}

bool bytebeam_uart_register_transmit_callback(uint8_t hd, bytebeam_uart_send_callback_t tx_cb) 
{
    return nwy_uart_reg_tx_cb(hd, (nwy_uart_send_callback_t)tx_cb);
}

bool bytebeam_uart_deinit(uint8_t hd) 
{
    return nwy_uart_deinit(hd);
}

bool bytebeam_set_rx_frame_timeout(uint8_t hd, int time) 
{
    return nwy_set_rx_frame_timeout(hd, time);
}

bool bytebeam_at_uart_send(bytebeam_uart_port_t port, void* data, size_t size) 
{
    int result = nwy_at_uart_send((nwy_uart_port_t)port, data, size);
    return result >= 0;
}