#include "bb_ble.h"
#include "bb_file.h"
#include "bb_osi_api.h"
#include "bb_pm.h"
#include "bb_uart.h"
#include "bb_usb_serial.h"

#define STM_UART        NWY_NAME_UART2   // pin 69/70 please refer to pin definition guide

extern bool get_MQTT_COnnection_Status(void);

extern volatile int http_download_flag;

extern char ota_action_id[10];

uint8_t ign_status = 0;

static unsigned long long last_can_rx_msg_time = 0;

uint8_t whole_bin_file_array_u8[81920] = {0};

int uart_thread_entry = 0;

float batt_voltage = 0.0;
float input_voltage = 0.0;

typedef struct __attribute__((packed)) can_msg_tst_tag
{
    uint32_t can_id;
    uint8_t can_data_buff[8];
    uint8_t seq;
} can_msg_tst;

int heart_beat_received_counter = 0;

/**
=====================================================================================================================================================

@fn Name            : void bytebeam_init_uart_params(bytebeam_uart_config_t* uart_config)
@b Scope            : Public
@n@n@b Description  : Initializes a bytebeam_uart_config_t structure with predefined values.
@param Input Data   : bytebeam_uart_config_t* uart_config - Pointer to the bytebeam_uart_config_t structure to be initialized.
@note               : uart_config must be allocated before calling this function.

=====================================================================================================================================================
*/
void bytebeam_init_uart_params(bytebeam_uart_config_t* uart_config)
{
    uart_config->name = STM_UART;
    uart_config->baud = 2000000;
    uart_config->parity = UART_PARITY_NONE;
    uart_config->data_size = UART_DATA_BITS_8;
    uart_config->stop_size = UART_STOP_BITS_1;
    uart_config->flow_ctrl = false;
    uart_config->mode = UART_MODE_DATA;
    uart_config->recv_cb = bytebeam_uart_recv_handle;
    uart_config->tx_cb = NULL;
}

/**
=====================================================================================================================================================

@fn Name            : UART_Status bytebeam_uart_init(bytebeam_uart_config_t* uart_config)
@b Scope            : Public
@n@n@b Description  : Initialize the UART interface with the configuration specified by uart_config.
@param Input Data   : bytebeam_uart_config_t* uart_config - Pointer to a bytebeam_uart_config_t structure containing the desired UART settings.
@return Return Value: UART_INIT_OK if the operation was successful, an error code otherwise.

=====================================================================================================================================================
*/
UART_Status bytebeam_uart_init(bytebeam_uart_config_t* uart_config)
{
    if (uart_config == NULL)
    {
        bytebeam_ext_echo("Error: Invalid uart_config pointer.\n");
        return UART_INIT_NULL_CFG;
    }

    uint32_t name = uart_config->name;
    uint32_t baud = uart_config->baud;
    uart_parity_t parity = uart_config->parity;
    uart_data_bits_t data_size = uart_config->data_size;
    uart_stop_bits_t stop_size = uart_config->stop_size;
    bool flow_ctrl = uart_config->flow_ctrl;
    nwy_uart_mode_t mode = uart_config->mode;
    void (*recv_cb)(uint8_t* data, uint32_t length) = uart_config->recv_cb;
    void (*tx_cb)(void) = uart_config->tx_cb;

    int ret_val = nwy_uart_init(name , mode);
    if (ret_val < 0)
    {
        bytebeam_ext_echo("Error: UART initialization failed.\n");
        return UART_INIT_FAIL;
    }

    if (bytebeam_uart_set_baud(name, baud) < 0)
    {
        bytebeam_ext_echo("Error: UART baud rate setting failed.\n");
        return UART_SET_BAUD_FAIL;
    }

    if (bytebeam_uart_reg_recv_cb(name, recv_cb) < 0)
    {
        bytebeam_ext_echo("Error: UART receive callback registration failed.\n");
        return UART_REG_CB_FAIL;
    }

    if (bytebeam_set_rx_frame_timeout(name, 2) < 0)
    {
        bytebeam_ext_echo("Error: Setting UART RX frame timeout failed.\n");
        return UART_SET_TIMEOUT_FAIL;
    }

    if (message_queue_init() < 0)
    {
        bytebeam_ext_echo("Error: Message queue initialization failed.\n");
        return UART_QUEUE_INIT_FAIL;
    }

    return UART_INIT_OK;  
}

/**
=====================================================================================================================================================

@fn Name            : bool bytebeam_uart_set_baud(uint8_t uart_handle, uint32_t baud)
@b Scope            : Public
@n@n@b Description  : Set the baud rate for a UART channel.
@param Input Data   : uint8_t uart_handle - The handle of the UART whose baud rate to set.
                      uint32_t baud - The desired baud rate.
@return Return Value: True if the operation was successful, false otherwise.

=====================================================================================================================================================
*/
bool bytebeam_uart_set_baud(uint8_t uart_handle, uint32_t baud)
{
    return nwy_uart_set_baud(uart_handle, baud);
}

/**
=====================================================================================================================================================

@fn Name            : bool bytebeam_uart_get_baud(uint8_t uart_handle, uint32_t* baud)
@b Scope            : Public
@n@n@b Description  : Get the baud rate for a UART channel.
@param Input Data   : 
                      uint8_t uart_handle - The handle of the UART whose baud rate to retrieve.
                      uint32_t* baud - Pointer to a variable to store the current baud rate.
@return Return Value: True if the operation was successful, false otherwise.

=====================================================================================================================================================
*/
bool bytebeam_uart_get_baud(uint8_t uart_handle, uint32_t* baud)
{
    return nwy_uart_get_baud(uart_handle, baud);
}

/**
=====================================================================================================================================================

@fn Name            : bool bytebeam_uart_set_parameters(uint8_t uart_handle, uart_parity_t parity, uart_data_bits_t data_size, uart_stop_bits_t stop_size, bool flow_ctrl)
@b Scope            : Public
@n@n@b Description  : Set the parameters for a UART channel.
@param Input Data   : uint8_t uart_handle - The handle of the UART whose parameters to set.
                      uart_parity_t parity - Parity setting for the UART.
                      uart_data_bits_t data_size - Number of data bits.
                      uart_stop_bits_t stop_size - Number of stop bits.
                      bool flow_ctrl - Flow control setting.
@return Return Value: True if the operation was successful, false otherwise.

=====================================================================================================================================================
*/
bool bytebeam_uart_set_parameters(uint8_t uart_handle, uart_parity_t parity, uart_data_bits_t data_size, uart_stop_bits_t stop_size, bool flow_ctrl)
{
    return nwy_uart_set_para(uart_handle, parity, data_size, stop_size, flow_ctrl);
}

/**
=====================================================================================================================================================

@fn Name            : bool bytebeam_uart_get_parameters(uint8_t uart_handle, uart_parity_t* parity, uart_data_bits_t* data_size, uart_stop_bits_t* stop_size, bool* flow_ctrl)
@b Scope            : Public
@n@n@b Description  : Get the parameters for a UART channel.
@param Input Data   : uint8_t uart_handle - The handle of the UART whose parameters to retrieve.
                      uart_parity_t* parity - Pointer to a variable to store the current parity setting.
                      uart_data_bits_t* data_size - Pointer to a variable to store the current number of data bits.
                      uart_stop_bits_t* stop_size - Pointer to a variable to store the current number of stop bits.
                      bool* flow_ctrl - Pointer to a variable to store the current flow control setting.
@return Return Value: True if the operation was successful, false otherwise.

=====================================================================================================================================================
*/
bool bytebeam_uart_get_parameters(uint8_t uart_handle, uart_parity_t* parity, uart_data_bits_t* data_size, uart_stop_bits_t* stop_size, bool* flow_ctrl)
{
    return nwy_uart_get_para(uart_handle, parity, data_size, stop_size, flow_ctrl);
}

/**
=====================================================================================================================================================

@fn Name            : uint32_t bytebeam_uart_send_data_in_bytes(uint8_t uart_handle, uint8_t* data_ptr, uint32_t length, uint32_t delay_ms)
@b Scope            : Public
@n@n@b Description  : Send data in bytes through UART with a specified delay between each byte.
@param Input Data   : uint8_t uart_handle - The handle of the UART to send data over.
                      uint8_t* data_ptr - Pointer to the data to send.
                      uint32_t length - Length of the data to send.
                      uint32_t delay_ms - Delay in milliseconds to pause between sending each byte.
@return Return Value: The total number of bytes sent.

=====================================================================================================================================================
*/
uint32_t bytebeam_uart_send_data_in_bytes(uint8_t uart_handle, uint8_t* data_ptr, uint32_t length, uint32_t delay_ms)
{
    int8_t ret_val = 0;
    uint32_t total_bytes_sent = 0;

    for (uint32_t loop_var = 0; loop_var < length; loop_var++)
    {
        ret_val = nwy_uart_send_data(uart_handle, &data_ptr[loop_var], 1);

        if (ret_val <= 0) // Checking if send operation failed
        {
            return 1;
        }
        total_bytes_sent += ret_val;
        bb_sleep(delay_ms);
    }
    return total_bytes_sent;
}

/**
=====================================================================================================================================================

@fn Name            : int bytebeam_uart_send_data_block(uint8_t uart_handle, uint8_t* data_ptr, uint32_t length)
@b Scope            : Public
@n@n@b Description  : Send a block of data over a UART channel.
@param Input Data   : uint8_t uart_handle - The handle of the UART to send data over.
                      uint8_t* data_ptr - Pointer to the data to send.
                      uint32_t length - Length of the data to send.
@return Return Value: The number of bytes sent, or 1 if the send operation failed.

=====================================================================================================================================================
*/
int bytebeam_uart_send_data_block(uint8_t uart_handle, uint8_t* data_ptr, uint32_t length)
{
    int ret_val = nwy_uart_send_data(uart_handle, data_ptr, length);

    if (ret_val <= 0)
    {
        return 1;
    }
    else
    {
        return ret_val;
    }
}

/**
=====================================================================================================================================================

@fn Name            : bool bytebeam_uart_register_transmit_callback(uint8_t uart_handle, bytebeam_uart_send_callback_t tx_cb)
@b Scope            : Public
@n@n@b Description  : Register a transmit callback function for a UART channel.
@param Input Data   : uint8_t uart_handle - The handle of the UART to register the callback for.
                      bytebeam_uart_send_callback_t tx_cb - The callback function to register.
@return Return Value: True if the operation was successful, false otherwise.

=====================================================================================================================================================
*/
bool bytebeam_uart_register_transmit_callback(uint8_t uart_handle, bytebeam_uart_send_callback_t tx_cb)
{
    return nwy_uart_reg_tx_cb(uart_handle, (nwy_uart_send_callback_t)tx_cb);
}

/**
=====================================================================================================================================================

@fn Name            : bool bytebeam_uart_deinit(uint8_t uart_handle)
@b Scope            : Public
@n@n@b Description  : Deinitialize a UART channel.
@param Input Data   : uint8_t uart_handle - The handle of the UART to deinitialize.
@return Return Value: True if the operation was successful, false otherwise.

=====================================================================================================================================================
*/
bool bytebeam_uart_deinit(uint8_t uart_handle)
{
    return nwy_uart_deinit(uart_handle);
}

/**
=====================================================================================================================================================

@fn Name            : bool bytebeam_set_rx_frame_timeout(uint8_t uart_handle, int time)
@b Scope            : Public
@n@n@b Description  : Set the receive frame timeout for a UART channel.
@param Input Data   : uint8_t uart_handle - The handle of the UART to set the timeout for.
                      int time - The timeout in milliseconds.
@return Return Value: True if the operation was successful, false otherwise.

=====================================================================================================================================================
*/
bool bytebeam_set_rx_frame_timeout(uint8_t uart_handle, int time)
{
    return nwy_set_rx_frame_timeout(uart_handle, time);
}

/**
=====================================================================================================================================================

@fn Name            : bool bytebeam_at_uart_send(bytebeam_uart_port_t port, void* data, size_t size)
@b Scope            : Public
@n@n@b Description  : Send data over a UART port.
@param Input Data   : bytebeam_uart_port_t port - The port to send data over.
                      void* data - Pointer to the data to send.
                      size_t size - The size of the data to send.
@return Return Value: number of bytes sent if the send operation was successful, 0 otherwise.

=====================================================================================================================================================
*/
bool bytebeam_at_uart_send(bytebeam_uart_port_t port, void* data, size_t size)
{
    int result = nwy_at_uart_send((nwy_uart_port_t)port, data, size);
    if(result)
    {
        return result;
    }
    else
    {
        return 0;
    }
}

/**
=====================================================================================================================================================

@fn Name            : bytebeam_uart_recv_handle
@b Scope            : Public
@n@n@b Description  : This function handles the received UART data, queues it if the UART thread is running, and maintains a timestamp for received data.
@param Input Data   : const char* str - The received UART data string.
                      uint32_t length - The length of the received UART data string.
@return Return Value: QueuePutStatus - Enumeration indicating the status of queue operations. 

=====================================================================================================================================================
*/
QueuePutStatus bytebeam_uart_recv_handle(const char* str, uint32_t length)
{
    static unsigned long long timestamp_start = 0;
    unsigned long long timestamp_end;

    if (timestamp_start == 0) 
    {
        timestamp_start = bytebeam_get_timestamp(TIMESTAMP_START);
    }

    timestamp_end = bytebeam_get_timestamp(TIMESTAMP_END, length);

    if ((timestamp_end - timestamp_start) > 1000) 
    {
        bytebeam_ext_echo("\r\n In uart callback Length Got: %d\r\n", length);
        timestamp_start = timestamp_end;
    }

    if (uart_thread_entry == 1)
    {
        for (int i = 0; i < length; i++)
        {
            if (SUCCESS != uart_rx_array_queue_put(str[i]))
            {
                bytebeam_ext_echo("Failed to put data into queue. Queue is full.\n");
                return QUEUE_PUT_FULL;
            }
            else
            {
                bytebeam_ext_echo("Successfully put data into queue.\n");
            }
        }
        return QUEUE_PUT_SUCCESS;
    }
    else
    {
        bytebeam_ext_echo("Failed to put data into queue. UART thread entry flag is not set.\n");
        return QUEUE_PUT_NO_ENTRY;
    }
}


/**
=====================================================================================================================================================

@fn Name            : print_status bytebeam_uart_print_data_struct(UART_data_struct* uart_struct)
@b Scope            : Public
@n@n@b Description  : Print the data contained in a UART_data_struct to stdout. This function is typically used for debugging.
@param Input Data   : UART_data_struct* uart_struct - The UART_data_struct to print.
@return Return Value: print_status indicating the status of the print operation.

=====================================================================================================================================================
*/
print_status print_uart_data_structure(UART_data_struct uart_strcut)
{
    if (uart_strcut.payload == NULL)
    {
        bytebeam_ext_echo("Error: Invalid UART data structure pointer.\n");
        return PRINT_FAILURE_NULL_POINTER;
    }
    bytebeam_ext_echo("STX: %x CMD:%x Length:%d \r\nPayload:", uart_strcut.stx,
                      uart_strcut.cmd,
                      uart_strcut.length_u16);

    for (int i = 0; i < uart_strcut.length_u16; i++)
    {
        if (i % 10 == 0)
        {
            bytebeam_ext_echo("\r\n");
        }
        bytebeam_ext_echo("%x ", uart_strcut.payload[i]);
    }

    bytebeam_ext_echo("\r\n");
    bytebeam_ext_echo("CRC: %d timestamp:%llu etx:%x", uart_strcut.crc_u16,
                      uart_strcut.timestamp,
                      uart_strcut.etx);

    return PRINT_SUCCESS;
}

/**
=====================================================================================================================================================

@fn Name            : publish_status send_can_mqtt(UART_data_struct *uart_struct)
@b Scope            : Public
@n@n@b Description  : Publish CAN messages to MQTT. The function checks if there is space in the MQTT publish message queue. 
                      If the queue is full, a warning message is printed. Otherwise, the UART data is put in the queue for publishing.
                      Note that the message is not published immediately, but placed in a queue for later processing.
@param Input Data   : UART_data_struct *uart_struct - A pointer to the UART_data_struct containing the CAN message to be published.
@return Return Value: publish_status indicating the status of the MQTT publish operation.

=====================================================================================================================================================
*/
publish_status send_can_mqtt(UART_data_struct *uart_strcut)
{
    static uint16_t package_count = 0;

    if (bytebeam_get_queue_spaceevent_cnt(mqtt_publish_msg_queue) == 0)
    {
        bytebeam_ext_echo("\r\nMQTT Queue full agidhe\r\n");
        return PUBLISH_FAILURE_QUEUE_FULL;
    }
    else
    {
        bytebeam_put_msg_que(mqtt_publish_msg_queue, uart_strcut, 0);
        return PUBLISH_SUCCESS;
    }

    return 0;
}

/**
=====================================================================================================================================================

@fn Name            : send_status send_ack_nack(uint8_t status, uint8_t command, uint8_t *payload, uint16_t length)
@b Scope            : Public
@n@n@b Description  : Send an acknowledgment or negative acknowledgment message over UART.
@param Input Data   : uint8_t status - The status value.
                      uint8_t command - The command value.
                      uint8_t *payload - Pointer to the payload data.
                      uint16_t length - Length of the payload data.
@return Return Value: send_status The status code indicating the success or failure of the send operation.

=====================================================================================================================================================
*/
send_status send_ack_nack(uint8_t status, uint8_t command, uint8_t *payload, uint16_t length)
{
    uint16_t len = 13;
    uint8_t can_data[100] = {0};
    length = length;

    can_data[0] = 0xDE;
    can_data[1] = 0xAD;
    can_data[2] = command;
    can_data[3] = (len)&0xFF;
    can_data[4] = (len) >> 8;
    can_data[5] = status;

    can_data[len + 5] = 0x00;
    can_data[len + 6] = 0x00;
    can_data[len + 7] = 0x03;

    if (bytebeam_semaphore_acquire(uart_tx_semaphore, 0xFFFF) == true)
    {
        for (int i = 0; i < 8 + len; i++)
        {
            bytebeam_uart_send_data_in_bytes(STM_UART_fd, can_data, 8 + len, 50);
            bytebeam_usleep(1);
            return SEND_SUCCESS;
        }
        bytebeam_sleep(1);
        bytebeam_semaphore_release(uart_tx_semaphore);
    }

     return SEND_FAILURE_SEMAPHORE;
}

/**
=====================================================================================================================================================

@fn Name            : int32_t execute_uart_command(UART_data_struct* uart_struct)
@b Scope            : Public
@n@n@b Description  : Execute a command received over UART and performs the corresponding actions based on the command value.
@param Input Data   : UART_data_struct* uart_struct - Pointer to the UART data structure containing the command information.
@return Return Value: Status code (always returns 0).

=====================================================================================================================================================
*/
int32_t execute_command(UART_data_struct *uart_strcut)
{

    static int count = 0;
    static unsigned long long timestamp_start = 0;

    if (timestamp_start == 0)
    {
        timestamp_start = bytebeam_get_timestamp(TIMESTAMP_START, 0); 
    }

    switch (uart_strcut->cmd)
    {
    case 0x00:
    {
        heart_beat_received_counter = 0;
        bytebeam_ext_echo("Execute command %d\r\n", uart_strcut->cmd);
        snprintf(s32_app_fw_ver, 15, "\"%d.%d.%d\"", uart_strcut->payload[0], uart_strcut->payload[1], uart_strcut->payload[2]);
        input_voltage = ((float)(((uint16_t)uart_strcut->payload[6] << 8) | uart_strcut->payload[5]) * 3.3 * 6.0) / 4096;
        batt_voltage = ((float)(((uint16_t)uart_strcut->payload[4] << 8) | uart_strcut->payload[3]) * 3.3 * 3.0) / 4096;
        ign_status = uart_strcut->payload[7];
        send_ack_nack(0, uart_strcut->cmd, NULL, 0);
        break;
    }
    case 0xCE:
    {
        snprintf(tork_fw_ver, 15, "\"%c%c%c%c\"", uart_strcut->payload[0], uart_strcut->payload[1], uart_strcut->payload[2], uart_strcut->payload[3]);
        snprintf(tork_hw_ver, 15, "\"%c%c%c%c\"", uart_strcut->payload[4], uart_strcut->payload[5], uart_strcut->payload[6], uart_strcut->payload[7]);
    }
    case 0x04:
    {
        count++;
        unsigned long long timestamp_end = bytebeam_get_timestamp(TIMESTAMP_END);
        last_can_rx_msg_time = timestamp_end;

        if ((timestamp_end - timestamp_start) > 950)
        {
            count = 0;
            timestamp_start = bytebeam_get_timestamp(TIMESTAMP_START);
        }

        send_ack_nack(0, uart_strcut->cmd, NULL, 0);
        if (http_download_flag == 0)
        {
            send_can_mqtt(uart_strcut);
        }
        break;
    }
    case TORK_BLDR_INIT_ACK:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("Tork bootloader activation successful\r\n");
            tork_bldr_cmd_e = TORK_BLDR_CODE_SIZE_CMD;
        }
        break;
    }
    case TORK_BLDR_CODE_SIZE_CMD_ACK:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("Tork bootloader code size transmit successful\r\n");
            tork_bldr_cmd_e = TORK_BLDR_ERASE_CMD;
        }
        break;
    }
    case TORK_BLDR_ERASE_CMD_ACK:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("Tork bootloader flash erase successful\r\n");
            tork_bldr_cmd_e = TORK_BLDR_SEND_DATA_CMD;
        }
        break;
    }
    case TORK_BLDR_WRITE_DATA_BLOCK_ACK:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            if (all_blocks_sent == 1)
            {
                bytebeam_ext_echo("Tork bootloader entire firmware image write successful\r\n");
                tork_bldr_cmd_e = TORK_BLDR_SEND_JMP_CMD;
            }
            else
            {
                bytebeam_ext_echo("Tork bootloader block write successful\r\n");
                tork_bldr_cmd_e = TORK_BLDR_SEND_DATA_CMD;
            }
        }
        break;
    }
    case TORK_BLDR_JMP_CMD_ACK:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("Tork bootloader jump to main app successful\r\n");
            tork_bldr_cmd_e = TORK_BLDR_APP_UPDATE_SUCCESS;
        }
        break;
    }
    /*******************************************************************************************************************/
    case BOOTLOADER_MODE_ENABLE_CMD_SEND:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("Bootloader mode activated cmd  ACK Success received\r\n");
            bootloader_uart_tx_cmd_e = BOOTLOADER_CONFIG_DATA_CMD_SEND;
        }
        else
        {
            bytebeam_ext_echo("Bootloader mode activated cmd  ACK  Failer received\r\n");
        }
    }
    break;
    /*******************************************************************************************************************/
    case BOOTLOADER_CONFIG_DATA_CMD_SEND:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("Bootloader config data send  cmd  ACK Success received\r\n");
            bootloader_uart_tx_cmd_e = BOOTLOADER_1KB_CRC_DATA_SEND;
        }
        else
        {
            bytebeam_ext_echo("Bootloader config data send  cmd  ACK failed received\r\n");
        }
    }
    break;
    /*******************************************************************************************************************/
    case BOOTLOADER_1KB_CRC_DATA_SEND:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("Bootloader 1kb  data send  cmd  ACK Success received\r\n");
            bootloader_uart_tx_cmd_e = BOOTLOADER_1KB_CRC_DATA_SEND;
            // if(new_firmware_file_size_u32 != 0)
            // {
            //     bootloader_uart_tx_cmd_e = BOOTLOADER_1KB_CRC_DATA_SEND;
            // }
            // else
            // {
            //     //bootloader_uart_tx_cmd_e = BOOTLOADER_END_OF_DATA_CMD_SEND;
            // }
        }
        else
        {
            bytebeam_ext_echo("Bootloader config data send  cmd  ACK failed received\r\n");
        }
    }
    break;
    /*******************************************************************************************************************/
    case BOOTLOADER_END_OF_DATA_CMD_SEND:
    {
        // if(uart_rx_data.payload[0]  == 0xAA)
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("Bootloader BOOTLOADER_END_OF_DATA_CMD_SEND  ACK Success received\r\n");
            bootloader_uart_tx_cmd_e = BOOTLOADER_WHOLE_BIN_CRC_CHECK_CMD_SEND;
        }
        else
        {
            bytebeam_ext_echo("Bootloader BOOTLOADER_END_OF_DATA_CMD_SEND ACK failed received\r\n");
        }
    }
    break;

    /*******************************************************************************************************************/
    case BOOTLOADER_WHOLE_BIN_CRC_CHECK_CMD_SEND:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("Bootloader BOOTLOADER_WHOLE_BIN_CRC_CHECK_CMD_SEND  ACK Success received\r\n");
            bootloader_uart_tx_cmd_e = BOOTLOADER_JUMPING_TO_BOOTLOADER_CMD_SEND;
        }
        else
        {
            if (s32_update_retries > 0)
            {
                s32_update_retries--;
                bootloader_uart_tx_cmd_e = BOOTLOADER_CONFIG_DATA_CMD_SEND;
            }
            else
            {
                publish_action_status(ota_action_id, 85, "Failed", "S32 CRC Mismatch");
                bytebeam_sleep(3000);
                publish_action_status(ota_action_id, 85, "Failed", "S32 CRC Mismatch");
                bytebeam_sleep(3000);
                bytebeam_power_off(2);
            }
            bytebeam_ext_echo("Bootloader BOOTLOADER_WHOLE_BIN_CRC_CHECK_CMD_SEND ACK failed received\r\n");
        }
    }
    break;

    /*******************************************************************************************************************/
    case BOOTLOADER_JUMPING_TO_BOOTLOADER_CMD_SEND:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("Bootloader BOOTLOADER_JUMPING_TO_BOOTLOADER_CMD_SEND  ACK Success received\r\n");
            bootloader_uart_tx_cmd_e = BOOTLOADER_INT_APP_ERASE_CMD_SEND;
        }
        else
        {
            bytebeam_ext_echo("Bootloader BOOTLOADER_JUMPING_TO_BOOTLOADER_CMD_SEND ACK failed received\r\n");
        }
    }
    break;
    /*******************************************************************************************************************/
    case BOOTLOADER_INT_APP_ERASE_CMD_SEND:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("Bootloader BOOTLOADER_INT_APP_ERASE_CMD_SEND  ACK Success received\r\n");
            bootloader_uart_tx_cmd_e = BOOTLOADER_INT_APP_WRITE_CMD_SEND;
        }
        else
        {
            bytebeam_ext_echo("Bootloader BOOTLOADER_INT_APP_ERASE_CMD_SEND ACK failed received\r\n");
        }
    }
    break;
    /*******************************************************************************************************************/
    case BOOTLOADER_INT_APP_WRITE_CMD_SEND:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("Bootloader BOOTLOADER_INT_APP_WRITE_CMD_SEND  ACK Success received\r\n");
            bootloader_uart_tx_cmd_e = BOOTLOADER_JUMPING_APPLICATION_CMD_SEND;
        }
        else
        {
            bytebeam_ext_echo("Bootloader BOOTLOADER_INT_APP_WRITE_CMD_SEND ACK failed received\r\n");
        }
    }
    break;
    /********************************************************************************************************************/
    case BOOTLOADER_JUMPING_APPLICATION_CMD_SEND:
    {
        if (uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("New Firmware Running Succesfully\r\n");
            bootloader_uart_tx_cmd_e = BOOTLOADER_STATE_UNKNOWN;
        }
        else
        {
            bytebeam_ext_echo("Bootloader BOOTLOADER_JUMPING_APPLICATION_CMD_SEND ACK failed received\r\n");
        }
    }
    break;
    /********************************************************************************************************************/
    case BOOTLOADER_TIMEOUT_ERROR_WITH_DIAG_TOOL_CMD_SEND:
    {
    }
    break;
    /********************************************************************************************************************/
    case BOOTLOADER_APPLICATION_NOT_AVAILABLE_CMD_SEND:
    {
        bootloader_uart_tx_cmd_e = BOOTLOADER_MODE_ENABLE_CMD_SEND;
        bytebeam_ext_echo("BOOTLOADER_APPLICATION_NOT_AVAILABLE_CMD_ACK_RECEIVED\r\n");
    }
    break;
    case 0xBB:
    {
        // retrieve_ble_data
        uint8_t can_buff[8] = {0};
        uint32_t msg_id = ((uint32_t)(uart_strcut->payload[0])) |
                          (((uint32_t)(uart_strcut->payload[1])) << 8) |
                          (((uint32_t)(uart_strcut->payload[2])) << 16) |
                          (((uint32_t)(uart_strcut->payload[3])) << 24);
        // bytebeam_ext_echo("received ble can msg, ID is %X\r\n",msg_id);

        if (msg_id == 0x777)
        {
            bytebeam_power_off(2);
            return 0;
        }
        if (http_download_flag == 0)
            bytebeam_ble_send_data(12, uart_strcut->payload);
        break;
    }
#if 0
    case 0xCC:
    {
        if(uart_strcut->payload[0] == 0xAA)
        {
            bytebeam_ext_echo("SOS received\r\n");

            unsigned long long timestamp1 = bytebeam_get_timestamp(TIMESTAMP_START); // Call to get timestamp
            uint32_t sos_button_status = 1;
            static uint32_t sq_id = 0;
            sq_id++;
            memset(json_sos_message_buff_gau8, 0, JSON_SOS_MESSAGES_BUFF_LEN);
            snprintf(json_sos_message_buff_gau8, sizeof(json_sos_message_buff_gau8), sos_mess_json,
                    timestamp1,
                    sos_button_status,
                    sq_id);
            bytebeam_ext_echo("%s", json_sos_message_buff_gau8); // For the verification of the JSON frame

            UART_data_struct sos_publish_msg;
            sos_publish_msg.cmd = 0xCC;
            sos_publish_msg.length_u16 = strlen((char *)json_sos_message_buff_gau8);

            memcpy(sos_publish_msg.payload, json_sos_message_buff_gau8, sos_publish_msg.length_u16);

            // if (get_MQTT_COnnection_Status() && (store_msg_flag == false) && (bytebeam_get_queue_spaceevent_cnt(mqtt_publish_msg_queue)!= 0))
            if (get_MQTT_COnnection_Status())
            {
                bytebeam_ext_echo("publishing SOS in MQTT\r\n");
                bytebeam_put_msg_que(mqtt_publish_msg_queue, &sos_publish_msg, 0);
            }
        }
    }

    
    break;
#endif
#if 1
    case 0xDD:
    {
        ign_status = uart_strcut->payload[0];
        // bytebeam_ext_echo("Ignition message\r\n");

        //unsigned long long timestamp1 = bytebeam_get_timestamp(TIMESTAMP_START); // Call to get timestamp
        //uint32_t ignition_button_status = (uint32_t)uart_strcut->payload[0];
        //static uint32_t sq_id = 0;
        //sq_id++;
        // memset(json_ignition_message_buff_gau8, 0, JSON_IGN_MESSAGES_BUFF_LEN);
        // snprintf(json_ignition_message_buff_gau8, sizeof(json_ignition_message_buff_gau8), ignition_mess_json,
        //             timestamp1,
        //             ignition_button_status,
        //             sq_id);
        // bytebeam_ext_echo("%s", json_ignition_message_buff_gau8); // For the verification of the JSON frame

        // UART_data_struct ign_publish_msg;
        // ign_publish_msg.cmd = 0xDD;
        // ign_publish_msg.length_u16 = strlen((char *)json_ignition_message_buff_gau8);

        // memcpy(ign_publish_msg.payload, json_ignition_message_buff_gau8, ign_publish_msg.length_u16);

        // // if (get_MQTT_COnnection_Status() && (store_msg_flag == false) && (bytebeam_get_queue_spaceevent_cnt(mqtt_publish_msg_queue)!= 0))
        // if (get_MQTT_COnnection_Status())
        // {
        //     bytebeam_ext_echo("publishing ignition in MQTT\r\n");
        //     bytebeam_put_msg_que(mqtt_publish_msg_queue, &ign_publish_msg, 0);
        // }
    }
    break;
#endif
    case 0xEE:
    {
        send_ack_nack(0, uart_strcut->cmd, NULL, 0);
        bytebeam_sleep(500);
        bytebeam_power_off(1);
    }
    break;
    default:
        break;
    }
    return 0;
}

/**
=====================================================================================================================================================

@fn Name            : send_can_status tork_send_can_data(uint32_t can_msg_id, uint8_t *can_data_arr, uint8_t seq)
@b Scope            : Public
@n@n@b Description  : Constructs a CAN message packet and sends it over UART.
@param Input Data   : uint32_t can_msg_id - The CAN message ID.
                      uint8_t *can_data_arr - Pointer to the CAN data array.
                      uint8_t seq - The sequence number.
@return Return Value: send_can_status The status code indicating the success or failure of the send operation.

=====================================================================================================================================================
*/
send_can_status tork_send_can_data(uint32_t can_msg_id, uint8_t *can_data_arr, uint8_t seq)
{
    uint8_t command = 0xAB;
    uint16_t length = 13;
    uint8_t can_data[21] = {
        0,
    };

    can_msg_tst can_msg_st_obj;
    memset((void *)&can_msg_st_obj, 0, sizeof(can_msg_tst));
    can_msg_st_obj.can_id = can_msg_id;
    can_msg_st_obj.seq = seq;

    memcpy((uint8_t *)can_msg_st_obj.can_data_buff, can_data_arr, 8);

    can_data[0] = 0xDE;
    can_data[1] = 0xAD;
    can_data[2] = command;

    can_data[3] = (length)&0xFF; // length lo byte
    can_data[4] = (length) >> 8; // length hi byte

    memcpy(&can_data[5], (uint8_t *)&can_msg_st_obj, 13);

    can_data[18] = 0x00;
    can_data[19] = 0x00;
    can_data[20] = 0x03;

    if (bytebeam_semaphore_acquire(uart_tx_semaphore, 0xFFFF) == true)
    {
        int ret_val = bytebeam_uart_send_data_in_bytes(STM_UART_fd, can_data, 21, 50);
        bytebeam_semaphore_release(uart_tx_semaphore);
        if (ret_val > 0)
        {
            return SEND_CAN_SUCCESS;
        }
        else
        {
            return SEND_CAN_FAILURE;
        }
    }

    return SEND_CAN_SEMAPHORE_FAILURE;
}

/**
=====================================================================================================================================================

@fn Name            : int tork_transmit_512_bytes()
@b Scope            : Public
@n@n@b Description  : Transmits 512 bytes of data over CAN in blocks of 64 bytes.
@param Input Data   : None
@return Return Value: 0 on success, or the error code on failure.

=====================================================================================================================================================
*/
send_can_status tork_transmit_512_bytes()
{
    send_can_status ret_val = SEND_CAN_SUCCESS;
    uint8_t can_buff[8] = {0};
    uint32_t can_msg_id = 0;
    static int current_block = 0;
    bytebeam_ext_echo("Current block:- %d\r\n", current_block);
    if (current_block < num_of_complete_blocks)
    {
        for (int loop_var_1 = 0; loop_var_1 < 64; loop_var_1++)
        {
            for (int loop_var_2 = 0; loop_var_2 < 8; loop_var_2++)
            {
                can_buff[loop_var_2] = whole_bin_file_array_u8[(512 * current_block) + (8 * loop_var_1) + loop_var_2];
            }
            // can_msg_id = ((((uint32_t)(loop_var_1+1)) << 16) & 0xFFFF0000) | 0xF08C;
            can_msg_id = ((frame_counter_u32 << 16) & 0xFFFF0000) | 0xF08C;
            frame_counter_u32++;
            if (frame_counter_u32 > 0xFF)
            {
                frame_counter_u32 = 0;
            }
            ret_val = tork_send_can_data(can_msg_id, can_buff, loop_var_1);
            if (ret_val != SEND_CAN_SUCCESS)
            {
                return ret_val;  // Return failure
            }
            bytebeam_usleep(2000);
        }
        for (int loop_var = 0; loop_var < 8; loop_var++)
        {
            can_buff[loop_var] = 0;
        }
        can_msg_id = ((frame_counter_u32 << 16) & 0xFFFF0000) | 0xF08D;
        frame_counter_u32++;
        if (frame_counter_u32 > 0xFF)
        {
            frame_counter_u32 = 0;
        }
        ret_val = tork_send_can_data(can_msg_id, can_buff, 0);
        if (ret_val != SEND_CAN_SUCCESS)
        {
            return ret_val;  // Return failure
        }
        // bytebeam_ext_echo("Data command:- %X\r\n",can_msg_id);
        current_block++;
        if (current_block == num_of_complete_blocks)
        {
            all_blocks_sent = 1;
            current_block = 0;
        }
    }
    // else
    // {
    //     all_blocks_sent = 1;
    //     current_block = 0;
    // }
    return ret_val;  // Return success
}

