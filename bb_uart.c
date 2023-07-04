#include "bb_ble.h"
#include "bb_file.h"
#include "bb_osi_api.h"
#include "bb_pm.h"
#include "bb_uart.h"
#include "bb_usb_serial.h"

JSMN_JSON_CAN_MESSAGE_tst JSMN_JSON_CAN_MESSAGE_st_obj;

extern bool get_MQTT_COnnection_Status(void);

extern volatile int http_download_flag;

extern char ota_action_id[10];

extern const char n58_app_fw_ver[15];
extern const char n58_base_fw_ver[8];
char s32_app_fw_ver[15] = "\"NULL\"";
char tork_fw_ver[10] = "\"NULL\"";
char tork_hw_ver[10] = "\"NULL\"";
extern char ble_mac_id_g[50];
extern uint8_t nw_csq_val;
extern uint8_t ble_status;
extern uint8_t imu_status;

uint8_t ign_status = 0;

extern int network_thread_counter;
static int device_shadow_counter = 0;

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

/************tork update params********/
tork_bldr_cmds_list_te tork_bldr_cmd_e = TORK_BLDR_INIT;

int tork_file_present = 0;
int all_blocks_sent = 0;
int num_of_complete_blocks = 0;
int tork_firmware_file_size = 0;
// uint8_t tork_bin_file_array_u8[81920]={0};
uint8_t can_data_buff_g[8] = {0};
uint32_t frame_counter_u32 = 0;
uint32_t tork_app_update_start = 0;
/************************************/

/***********s32 update params********/
typedef struct __attribute__((packed)) bootloader_config_tst_tag
{

    uint32_t new_firmware_size_u32;
    uint32_t new_firmware_crc_u32;
    uint8_t new_firmware_status_u8;
} bootloader_config_tst;

bootloader_config_tst bootloader_confif_msg_gst;
s32_bldr_cmd_ack_list_te bootloader_uart_tx_cmd_e = BOOTLOADER_MODE_ENABLE_CMD_SEND;

uint32_t total_bin_file_data_read_u32 = 0;
int new_firmware_file_size_u32 = 0;
int s32_update_complete = 0;
int num_of_one_kb_blocks = 0;
int last_block_byte_count = 0;
uint32_t whole_bin_crc_u32 = 0;
uint32_t s32_update_retries = 3;
/************************************/

/**
 * @brief Generate a CRC-32 checksum for a given buffer.
 *
 * This function calculates the CRC-32 checksum for the specified buffer.
 *
 * @param buf Pointer to the buffer.
 * @param len Length of the buffer in bytes.
 * @return    The calculated CRC-32 checksum.
 */
uint32_t bootloader_app_crc32_generate_u32(uint8_t *buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < len; i++)
    {
        uint8_t ch = buf[i];
        for (uint32_t j = 0; j < 8; j++)
        {
            uint32_t b = (ch ^ crc) & 1;
            crc >>= 1;
            if (b)
                crc = crc ^ 0xEDB88320;
            ch >>= 1;
        }
    }

    bytebeam_ext_echo("Calculated CRC: 0x%08X\n", ~crc); // Log the calculated CRC value
    return ~crc;
}

/**
 * \brief Initialize the UART communication with the necessary parameters.
 *
 * This function dynamically allocates a configuration structure,
 * sets its parameters, initializes the UART with this configuration, and returns
 * the UART's handle, which can be used in subsequent calls to UART functions.
 * 
 * \return The handle of the initialized UART on success, or -1 on failure.
 */

int bytebeam_uart_init(void) 
{
    bytebeam_uart_config_t* uart_config = malloc(sizeof(bytebeam_uart_config_t));

    if (uart_config == NULL) {
        // Error handling for memory allocation failure
        bytebeam_ext_echo("Error: Memory allocation for uart_config failed.\n");
        return -1;
    }

    uart_config->name = STM_UART;
    uart_config->baud = 2000000;
    uart_config->parity = UART_PARITY_NONE;
    uart_config->data_size = UART_DATA_BITS_8;
    uart_config->stop_size = UART_STOP_BITS_1;
    uart_config->flow_ctrl = false;
    uart_config->recv_cb = uart_recv_handle;
    uart_config->tx_cb = NULL;
    uart_config->mode = UART_MODE_AT;

    int ret_val = nwy_uart_init(uart_config->name, uart_config->mode);
    if (ret_val < 0) 
    {
        // Error handling for UART initialization failure
        bytebeam_ext_echo("Error: UART initialization failed.\n");
        return -1;  // Failure
    }
    
    return ret_val;  
}

/**
 * \brief Set the baud rate for a UART channel.
 *
 * \param uart_handle The handle of the UART whose baud rate to set.
 * \param baud        desired baud rate.
 *
 * \return True if the operation was successful, false otherwise.
 */

bool bytebeam_uart_set_baud(uint8_t uart_handle, uint32_t baud) 
{
    return nwy_uart_set_baud(uart_handle, baud);
}

/**
 * \brief Get the baud rate for a UART channel.
 *
 * \param uart_handle The handle of the UART whose baud rate to retrieve.
 * \param baud        Pointer to a variable to store the current baud rate.
 *
 * \return True if the operation was successful, false otherwise.
 */

bool bytebeam_uart_get_baud(uint8_t uart_handle, uint32_t* baud) 
{
    return nwy_uart_get_baud(uart_handle, baud);
}

/**
 * \brief Set the parameters for a UART channel.
 *
 * \param uart_handle  The handle of the UART whose parameters to set.
 * \param parity       Parity setting for the UART.
 * \param data_size    Number of data bits.
 * \param stop_size    Number of stop bits.
 * \param flow_ctrl    Flow control setting.
 *
 * \return True if the operation was successful, false otherwise.
 */

bool bytebeam_uart_set_parameters(uint8_t uart_handle, uart_parity_t parity, uart_data_bits_t data_size, uart_stop_bits_t stop_size, bool flow_ctrl) 
{
    return nwy_uart_set_para(uart_handle, parity, data_size, stop_size, flow_ctrl);
}

/**
 * \brief Get the parameters for a UART channel.
 *
 * \param uart_handle The handle of the UART whose parameters to retrieve.
 * \param parity      Pointer to a variable to store the current parity setting.
 * \param data_size   Pointer to a variable to store the current number of data bits.
 * \param stop_size   Pointer to a variable to store the current number of stop bits.
 * \param flow_ctrl   Pointer to a variable to store the current flow control setting.
 *
 * \return True if the operation was successful, false otherwise.
 */

bool bytebeam_uart_get_parameters(uint8_t uart_handle, uart_parity_t* parity, uart_data_bits_t* data_size, uart_stop_bits_t* stop_size, bool* flow_ctrl) 
{
    return nwy_uart_get_para(uart_handle, parity, data_size, stop_size, flow_ctrl);
}

/**
 * \brief Send data in bytes through UART.
 *
 * \param uart_handle The handle of the UART to send data over.
 * \param data_ptr    Pointer to the data to send.
 * \param length      Length of the data to send.
 * \param delay_ms    Delay in milliseconds to pause between sending each byte.
 *
 * \return The total number of bytes sent.
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
 * \brief Send a block of data over a UART channel.
 *
 * \param uart_handle The handle of the UART to send data over.
 * \param data_ptr Pointer to the data to send.
 * \param length Length of the data to send.
 *
 * \return The number of bytes sent, or 1 if the send operation failed.
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
 * \brief Send a single byte over a UART channel.
 *
 * \param uart_handle The handle of the UART to send data over.
 * \param data        The byte to send.
 *
 * \return The number of bytes sent (should be 1), or 1 if the send operation failed.
 */

int bytebeam_uart_send_single_byte(uint8_t uart_handle, uint8_t data) 
{
    int ret_val = nwy_uart_send_data(uart_handle, &data, 1);

    if (ret_val <= 0)  
    {
        return 1; 
    }
    return ret_val;  
}

/**
 * \brief Register a transmit callback function for a UART channel.
 *
 * \param uart_handle The handle of the UART to register the callback for.
 * \param tx_cb       The callback function to register.
 *
 * \return True if the operation was successful, false otherwise.
 */

bool bytebeam_uart_register_transmit_callback(uint8_t uart_handle, bytebeam_uart_send_callback_t tx_cb) 
{
    return nwy_uart_reg_tx_cb(uart_handle, (nwy_uart_send_callback_t)tx_cb);
}

/**
 * \brief Deinitialize a UART channel.
 *
 * \param uart_handle The handle of the UART to deinitialize.
 *
 * \return True if the operation was successful, false otherwise.
 */

bool bytebeam_uart_deinit(uint8_t uart_handle) 
{
    return nwy_uart_deinit(uart_handle);
}

/**
 * \brief Set the receive frame timeout for a UART channel.
 *
 * \param uart_handle The handle of the UART to set the timeout for.
 * \param time        The timeout in milliseconds.
 *
 * \return True if the operation was successful, false otherwise.
 */

bool bytebeam_set_rx_frame_timeout(uint8_t uart_handle, int time) 
{
    return nwy_set_rx_frame_timeout(uart_handle, time);
}

/**
 * \brief Send data over a UART port.
 *
 * \param port The port to send data over.
 * \param data Pointer to the data to send.
 * \param size The size of the data to send.
 *
 * \return True if the send operation was successful, false otherwise.
 */

bool bytebeam_at_uart_send(bytebeam_uart_port_t port, void* data, size_t size) 
{
    int result = nwy_at_uart_send((nwy_uart_port_t)port, data, size);
    return result >= 0;
}

/**
 * \brief Configure a UART channel.
 *
 * \param uart_config Pointer to a structure containing the desired configuration settings.
 *
 * \return The handle of the UART channel, or -1 if the initialization failed.
 */

void bytebeam_uart_config(bytebeam_uart_config_t* uart_config)
{
   if (uart_config == NULL) {
        // Error handling for NULL pointer
        bytebeam_ext_echo("Error: uart_config pointer is NULL.\n");
        return -1;
    }

    int STM_UART_fd = nwy_uart_init(uart_config->name, uart_config->mode);

    if (STM_UART_fd < 0)
    {
        // Error handling for UART initialization failure
        bytebeam_ext_echo("Error: UART initialization failed.\n");
        return -1;
    }
    else
    {
        bytebeam_uart_set_baud(STM_UART_fd, uart_config->baud);
        bytebeam_uart_reg_recv_cb(STM_UART_fd, uart_config->recv_cb);
        bytebeam_set_rx_frame_timeout(STM_UART_fd, 2);
        message_queue_init();
    }

    return STM_UART_fd; 
}

/**
 * \brief Handles received UART data.
 *
 * This function is registered as the receive callback for the UART. 
 * It processes the received data and enqueues it for later processing.
 *
 * \param str    The received data.
 * \param length The length of the received data.
 */

void uart_recv_handle(const char* str, uint32_t length)
{
    static struct timeval tv;
    static double s = 0;
    static double ms = 0;
    static unsigned long long timestamp_start = 0;
    unsigned long long timestamp_end = 0;

    gettimeofday(&tv, NULL);
    s = tv.tv_sec;
    ms = ((double)tv.tv_usec) / 1.0e3;
    if (timestamp_start == 0) {
        timestamp_start = (unsigned long long)(s * 1000 + ms);
    }

    timestamp_end = (unsigned long long)(s * 1000 + ms);

    if ((timestamp_end - timestamp_start) > 1000) {
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
            }
            else
            {
                bytebeam_ext_echo("Successfully put data into queue.\n");
            }
        }
    } 
    else 
    {
        bytebeam_ext_echo("Failed to put data into queue. UART thread entry flag is not set.\n");
    }
}

/**
 * \brief Print the data contained in a UART_data_struct to stdout.
 *
 * This function is typically used for debugging. It prints the contents of a UART_data_struct
 * in a human-readable format. This includes the start byte (STX), command (CMD), length of the payload,
 * the payload itself, the CRC, the timestamp, and the end byte (ETX). The payload is printed
 * with a line break after every 10 bytes for readability.
 *
 * \param uart_strcut The UART_data_struct to print.
 */

void print_uart_data_structure(UART_data_struct uart_strcut)
{
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
}

/**
 * \brief Publish CAN messages to MQTT.
 *
 * This function takes a UART_data_struct containing CAN message data and publishes it to MQTT.
 * The function checks if there is space in the MQTT publish message queue. If the queue is full,
 * a warning message is printed. Otherwise, the UART data is put in the queue for publishing.
 * Note that the message is not published immediately, but placed in a queue for later processing.
 * The function keeps track of how many packages it has handled in a static variable 'package_count'.
 *
 * \param uart_strcut A pointer to the UART_data_struct containing the CAN message to be published.
 * \return            0 as a default return value.
 */

int32_t send_can_mqtt(UART_data_struct *uart_strcut)
{
    static uint16_t package_count = 0;

    if (bytebeam_get_queue_spaceevent_cnt(mqtt_publish_msg_queue) == 0)
    {
        bytebeam_ext_echo("\r\nMQTT Queue full agidhe\r\n");
    }
    else
    {
        bytebeam_put_msg_que(mqtt_publish_msg_queue, uart_strcut, 0);
    }

    return 0;   
}

/**
 * @brief Send an acknowledgment or negative acknowledgment message over UART.
 *
 * This function sends an acknowledgment or negative acknowledgment message over UART.
 *
 * @param status  The status value.
 * @param command The command value.
 * @param payload Pointer to the payload data.
 * @param length  Length of the payload data.
 * @return        Status code (always returns 0).
 */
int32_t send_ack_nack(uint8_t status, uint8_t command, uint8_t *payload, uint16_t length)
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
        }
        bytebeam_sleep(1);
        bytebeam_semaphore_release(uart_tx_semaphore);
    }

    return 0;
}

/**
 * @brief Execute a command received over UART.
 *
 * This function executes a command received over UART and performs the corresponding actions based on the command value.
 *
 * @param uart_struct Pointer to the UART data structure containing the command information.
 * @return Status code (always returns 0).
 */

int32_t execute_command(UART_data_struct *uart_strcut)
{

    struct timeval tv;
    static int count = 0;
    static double s = 0;
    static double ms = 0;
    static unsigned long long timestamp_start;

    if (s == 0)
    {
        gettimeofday(&tv, NULL);
        s = tv.tv_sec;
        ms = ((double)tv.tv_usec) / 1.0e3;
        timestamp_start = (unsigned long long)(s * 1000 + ms);
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
        // bytebeam_ext_echo("Execute command switch %d\r\n", uart_strcut.cmd);
        gettimeofday(&tv, NULL);
        double s = tv.tv_sec;
        double ms = ((double)tv.tv_usec) / 1.0e3;
        unsigned long long timestamp_end = (unsigned long long)(s * 1000 + ms);
        last_can_rx_msg_time = timestamp_end;

        if ((timestamp_end - timestamp_start) > 950)
        {
            // bytebeam_ext_echo("\r\nTime elapsed:%llu count:%d, \r\n", timestamp_end-timestamp_start, count*60);
            count = 0;
            timestamp_start = timestamp_end;
            // bytebeam_ext_echo("The queue size is: %d", 10 - bytebeam_get_queue_spaceevent_cnt(mqtt_publish_msg_queue));
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
                gettimeofday(&tv, NULL);

                double s = tv.tv_sec;
                double ms = ((double)tv.tv_usec) / 1.0e6;

                unsigned long long timestamp1 = (unsigned long long)(s + ms) * 1000;
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
        // gettimeofday(&tv, NULL);

        // double s = tv.tv_sec;
        // double ms = ((double)tv.tv_usec) / 1.0e6;

        // unsigned long long timestamp1 = (unsigned long long)(s + ms) * 1000;
        // uint32_t ignition_button_status = (uint32_t)uart_strcut->payload[0];
        // static uint32_t sq_id = 0;
        // sq_id++;
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
 * @brief Sends CAN data over UART.
 *
 * This function constructs a CAN message packet and sends it over UART.
 *
 * @param can_msg_id   The CAN message ID.
 * @param can_data_arr Pointer to the CAN data array.
 * @param seq          The sequence number.
 * @return 0 on success, -1 on failure to acquire semaphore.
 */

int tork_send_can_data(uint32_t can_msg_id, uint8_t *can_data_arr, uint8_t seq)
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
        return ret_val;  
    }
    
    return -1;  // Failure in acquiring semaphore
}

/**
 * @brief Transmits 512 bytes of data over CAN.
 *
 * This function transmits 512 bytes of data over CAN in blocks of 64 bytes.
 *
 * @return 0 on success, or the error code on failure.
 */

int tork_transmit_512_bytes()
{
    int ret_val = 0;
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
            if (ret_val != 0)
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
        if (ret_val != 0)
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

/**
 * @brief Performs Tork application update.
 *
 * This function is responsible for performing the Tork application update.
 * It reads the firmware file, transmits the necessary commands and data over CAN,
 * and handles the different stages of the update process.
 *
 * @param param A pointer to the parameters (not used in the function).
 */

void tork_app_update(void *param)
{
    static int tork_app_update_complete = 0;
    uint32_t can_msg_id = 0;
    uint32_t crc_val_u32 = 0;
    static int idle_state_counter = 0;
    bb_sleep(100);
    while (1)
    {
        if (bytebeam_semaphore_acquire(tork_update_semaphore, 0) == true)
        {
            tork_new_fw_file_read();
            while (1)
            {
                // should be blocked on semaphore
                // bytebeam_ext_echo("Entering task\r\n");
                if (tork_file_present == 0)
                {
                    bytebeam_semahpore_release(s32_update_semaphore);
                    publish_action_status(ota_action_id, 85, "Progress", "");
                    break;
                }
                switch (tork_bldr_cmd_e)
                {
                case TORK_BLDR_INIT:
                {
                    idle_state_counter = 0;
                    tork_app_update_start = 1;
                    bytebeam_suspend_thread(ble_app_thread_handle);
                    bytebeam_suspend_thread(network_app_thread);
                    // bytebeam_suspend_thread(mqtt_publish_thread);
                    // send_ack_nack(0, 0x55, NULL, 0);
                    // bytebeam_sleep(22000);
                    bytebeam_ext_echo("Transmitting bootloader activation command\r\n");
                    for (int loop_var = 0; loop_var < 8; loop_var++)
                    {
                        can_data_buff_g[loop_var] = 0;
                    }
                    can_data_buff_g[0] = 0x8A;
                    can_data_buff_g[1] = 0xF0;
                    can_data_buff_g[4] = 0x8A;
                    can_data_buff_g[5] = 0xF0;
                    bb_usleep(3000);
                    can_msg_id = ((frame_counter_u32 << 16) & 0xFFFF0000) | 0xF08A;
                    frame_counter_u32++;
                    tork_send_can_data(can_msg_id, can_data_buff_g, 0x00);
                    tork_bldr_cmd_e = TORK_BLDR_IDLE_STATE;
                }
                break;
                case TORK_BLDR_CODE_SIZE_CMD:
                {
                    idle_state_counter = 0;
                    bytebeam_ext_echo("Transmitting code size\r\n");
                    // tork_new_fw_file_read();
                    can_data_buff_g[7] = (uint8_t)((tork_firmware_file_size >> 24) & 0xFF);
                    can_data_buff_g[6] = (uint8_t)((tork_firmware_file_size >> 16) & 0xFF);
                    can_data_buff_g[5] = (uint8_t)((tork_firmware_file_size >> 8) & 0xFF);
                    can_data_buff_g[4] = (uint8_t)(tork_firmware_file_size & 0xFF);
                    for (int loop_var = 0; loop_var < 4; loop_var++)
                    {
                        can_data_buff_g[loop_var] = 0;
                    }
                    bb_usleep(1000);
                    can_msg_id = ((frame_counter_u32 << 16) & 0xFFFF0000) | 0xF08A;
                    frame_counter_u32++;
                    tork_send_can_data(can_msg_id, can_data_buff_g, 0x00);
                    tork_bldr_cmd_e = TORK_BLDR_IDLE_STATE;
                    bytebeam_ext_echo("code size transmitted\r\n");
                }
                break;
                case TORK_BLDR_ERASE_CMD:
                {
                    idle_state_counter = 0;
                    bytebeam_ext_echo("Transmitting flash erase command\r\n");
                    for (int loop_var = 0; loop_var < 8; loop_var++)
                    {
                        can_data_buff_g[loop_var] = 0;
                    }
                    bb_usleep(1000);
                    can_msg_id = ((frame_counter_u32 << 16) & 0xFFFF0000) | 0xF08E;
                    frame_counter_u32++;
                    tork_send_can_data(can_msg_id, can_data_buff_g, 0x00);
                    tork_bldr_cmd_e = TORK_BLDR_IDLE_STATE;
                }
                break;
                case TORK_BLDR_SEND_DATA_CMD:
                {
                    idle_state_counter = 0;
                    tork_transmit_512_bytes();
                    bytebeam_ext_echo("Wrote block\r\n");
                    tork_bldr_cmd_e = TORK_BLDR_IDLE_STATE;
                }
                break;
                case TORK_BLDR_SEND_JMP_CMD:
                {
                    idle_state_counter = 0;
                    bytebeam_ext_echo("Transmitting Jump to main app command\r\n");
                    can_data_buff_g[3] = (uint8_t)((tork_firmware_file_size >> 24) & 0xFF);
                    can_data_buff_g[2] = (uint8_t)((tork_firmware_file_size >> 16) & 0xFF);
                    can_data_buff_g[1] = (uint8_t)((tork_firmware_file_size >> 8) & 0xFF);
                    can_data_buff_g[0] = (uint8_t)(tork_firmware_file_size & 0xFF);

                    for (int loop_var = 4; loop_var < 8; loop_var++)
                    {
                        can_data_buff_g[loop_var] = 0;
                    }
                    bb_usleep(1000);
                    can_msg_id = ((frame_counter_u32 << 16) & 0xFFFF0000) | 0xF08F;
                    tork_send_can_data(can_msg_id, can_data_buff_g, 0x00);
                    tork_bldr_cmd_e = TORK_BLDR_IDLE_STATE;
                }
                break;
                case TORK_BLDR_APP_UPDATE_SUCCESS:
                {
                    bytebeam_ext_echo("Tork App update competed\r\n");
                    all_blocks_sent = 0;
                    frame_counter_u32 = 0;
                    tork_bldr_cmd_e = TORK_BLDR_INIT;
                    tork_app_update_complete = 1;
                    // bytebeam_power_off(2);
                    // tork_bldr_cmd_e = TORK_BLDR_IDLE_STATE;
                }
                break;
                case TORK_BLDR_IDLE_STATE:
                {
                    // do nothing
                    idle_state_counter++;
                    if (idle_state_counter == 2500)
                    {
                        publish_action_status(ota_action_id, 75, "Failed", "");
                        bb_sleep(3000);
                        publish_action_status(ota_action_id, 75, "Failed", "");
                        bb_sleep(3000);
                        bb_power_off(2);
                    }
                    int i = 2;
                }
                break;
                default:
                    break;
                }
                if (tork_app_update_complete == 1)
                {
                    idle_state_counter = 0;
                    tork_file_present = 0;
                    bytebeam_semahpore_release(s32_update_semaphore);
                    publish_action_status(ota_action_id, 85, "Progress", "");
                    // http_download_flag = 0;
                    // bytebeam_resume_thread(gps_app_thread);
                    // bytebeam_resume_thread(ble_app_thread_handle);
                    // bytebeam_resume_thread(network_app_thread);
                    // // bytebeam_resume_thread(mqtt_publish_thread);
                    // send_ack_nack(0, 0x55, NULL, 0);
                    // tork_app_update_complete = 0;
                    // tork_app_update_start = 0;
                    break;
                }
                bb_sleep(20);
            }
        }
        bb_sleep(50);
    }
}

/**
 * @brief Sends acknowledgment for a bootloader application UART command.
 *
 * This function sends an acknowledgment for a bootloader application UART command over CAN.
 * It constructs the CAN data packet with the command and status information, and sends it over UART.
 *
 * @param cmd         The command byte to be acknowledged.
 * @param cmd_ack_sts The status byte indicating the acknowledgment status.
 */

void bootloader_app_uart_cmd_ack_send_v(uint8_t cmd, uint8_t cmd_ack_sts)
{
    uint8_t can_data[100] = {0};
    uint16_t length = 12;

    can_data[0] = 0xDE;
    can_data[1] = 0xAD;
    can_data[2] = cmd;
    can_data[3] = (length + 1) & 0xFF; // length lo byte //status length is one more byte
    can_data[4] = (length + 1) >> 8;   // length hi byte
    can_data[5] = cmd_ack_sts;
    can_data[length + 6] = 0xDE;
    can_data[length + 7] = 0xAD; // TODO::implement CRC check
    can_data[length + 8] = 0x03;

    for (int i = 0; i < 9 + length; i++)
    {
        // bytebeam_ext_echo("%x ", can_data[i]);
        bytebeam_uart_send_data(STM_UART_fd, (uint8_t *)&can_data[i], 1);
        bb_usleep(50);
    }
}

/**
 * @brief Reads the new firmware file from the SD card.
 *
 * This function reads the new firmware file from the SD card into memory.
 * It retrieves the file size, calculates the number of 1KB blocks and the size of the last block.
 * It then opens the file in read-only mode and reads its contents into the `whole_bin_file_array_u8` buffer.
 * If the read operation is successful, it calculates the CRC32 value of the file.
 *
 * @note This function assumes that the SD card is mounted and accessible.
 */

void bootloader_new_firmware_file_read_v(void)
{
    int read_count_u32 = 0;
    char new_firmware_file_name_string[100] = {0};
    int new_firmware_file_fd = 0;
    int temp_fw_size = 0;
    // if (sd_card_mounted_flag == true)
    if (bytebeam_sdcard_status() == true)
    {
        sprintf(new_firmware_file_name_string, "/sdcard0/test_app.bin");

        new_firmware_file_fd = bytebeam_fopen(new_firmware_file_name_string, NWY_RDWR);
        if (new_firmware_file_fd < 0)
        {
            bytebeam_ext_echo("\r\nfile open fail\r\n");
            return;
        }
        // bytebeam_ext_echo("\r\nfile call 3\r\n");
        new_firmware_file_size_u32 = bytebeam_fsize(new_firmware_file_fd);
        temp_fw_size = new_firmware_file_size_u32;
        num_of_one_kb_blocks = new_firmware_file_size_u32 / 1024;

        if (num_of_one_kb_blocks * 1024 < new_firmware_file_size_u32)
        {
            last_block_byte_count = new_firmware_file_size_u32 % 1024;
            int t_var = 0;
            for (t_var = new_firmware_file_size_u32; t_var < ((num_of_one_kb_blocks + 1) * 1024); t_var++)
            {
                whole_bin_file_array_u8[t_var] = 0xFF;
            }
            new_firmware_file_size_u32 = (num_of_one_kb_blocks + 1) * 1024;
            num_of_one_kb_blocks = num_of_one_kb_blocks + 1;
        }
        else if (num_of_one_kb_blocks * 1024 == new_firmware_file_size_u32)
        {
            last_block_byte_count = 0;
        }

        bytebeam_ext_echo("\r\n New_Fimrware_App_Size = %d\r\n", new_firmware_file_size_u32);
        bytebeam_fclose(new_firmware_file_fd);

        new_firmware_file_fd = bytebeam_fopen(new_firmware_file_name_string, NWY_RDONLY);
        bytebeam_ext_echo("\r\n file open status  = %d\r\n", new_firmware_file_fd);
        read_count_u32 = bytebeam_fread(new_firmware_file_fd, whole_bin_file_array_u8, new_firmware_file_size_u32);
        bytebeam_ext_echo("\r\n file read status  = %d\r\n", read_count_u32);
        bytebeam_fclose(new_firmware_file_fd);
        if (read_count_u32 == temp_fw_size)
        {
            whole_bin_crc_u32 = bootloader_app_crc32_generate_u32(whole_bin_file_array_u8, (uint32_t)(temp_fw_size));
            bytebeam_ext_echo("\r\n Whole bin crc = %d\r\n", whole_bin_crc_u32);
        }
        else
        {
            bytebeam_ext_echo("\r\n whole file read failed\r\n");
            bootloader_uart_tx_cmd_e = BOOTLOADER_STATE_UNKNOWN;
        }
    }
    else
    {
        bytebeam_ext_echo("\r\n SD Card mount failed\r\n");
        bootloader_uart_tx_cmd_e = BOOTLOADER_STATE_UNKNOWN;
    }
}

/**
 * @brief Sends UART data with command and status using the bootloader protocol.
 *
 * This function sends UART data using the bootloader protocol. It constructs the
 * command packet with the provided command, status, and data, and sends it over UART.
 *
 * @param cmd The command byte to be sent.
 * @param cmd_sts The status byte to be sent.
 * @param data_au8 Pointer to the data array to be sent.
 * @param length_u16 The length of the data array.
 */

void bootloader_app_uart_data_send_v(uint8_t cmd, uint8_t cmd_sts, uint8_t *data_au8, uint16_t length_u16)
{
    uint16_t test_len = 13;
    uint8_t can_data[75] = {
        0,
    };
    can_data[0] = 0xDE;
    can_data[1] = 0xAD;
    can_data[2] = cmd;
    can_data[3] = (test_len)&0xFF; // length lo byte //status length is one more byte
    can_data[4] = (test_len) >> 8; // length hi byte
    can_data[5] = cmd_sts;
    if (length_u16 != 0)
    {
        // memory_copy_u8_array_v(&can_data[6], data_au8, length_u16);
        memcpy(&can_data[6], data_au8, length_u16);
    }
    can_data[18] = 0x00;
    can_data[19] = 0x00;
    can_data[20] = 0x03;
    if (bytebeam_semaphore_acquire(uart_tx_semaphore, 0xFFFF) == true)
    {
        for (int i = 0; i < 21; i++)
        {
            // bytebeam_ext_echo("%x ", can_data[i]);
            bytebeam_uart_send_data_in_bytes(STM_UART_fd, (uint8_t *)&can_data[i], 1);
            // bb_sleep(1);
            bb_usleep(50);
        }
        bytebeam_semahpore_release(uart_tx_semaphore);
    }
}


void bootloader_new_firmware_file_read_and_uart_send_handling_v(void)
{
    static int loop_var = 1;
    if (num_of_one_kb_blocks > 0)
    {
        bytebeam_ext_echo("1024 Block id = %d \r\n", loop_var);
        for (int temp_var = 0; temp_var < 128; temp_var++)
        {
            bootloader_app_uart_data_send_v(BOOTLOADER_1KB_CRC_DATA_SEND, 0xAA,
                                            &whole_bin_file_array_u8[total_bin_file_data_read_u32], 8);
            total_bin_file_data_read_u32 += 8;
            bb_usleep(2000);
        }
        bytebeam_ext_echo("1024 bytes sent\r\n");

        num_of_one_kb_blocks--;
        loop_var++;
    }
    else
    {
        bytebeam_ext_echo("All Fimrware Sent Succesfully and now sending end of data\r\n");
        bootloader_app_uart_cmd_ack_send_v(BOOTLOADER_END_OF_DATA_CMD_SEND, 0xAA);
    }
}

/**
=====================================================================================================================================================

@fn Name			:
@b Scope            :
@n@n@b Description  :
@param Input Data   :
@return Return Value:

=====================================================================================================================================================
*/
int publish_device_shadow()
{
    struct timeval tval;
    // static int count = 0;
    double sec = 0;
    double msec = 0;

    bytebeam_ext_echo("Device shadow message\r\n");
    gettimeofday(&tval, NULL);

    sec = tval.tv_sec;
    msec = ((double)tval.tv_usec) / 1.0e6;

    unsigned long long timestamp1 = (unsigned long long)(sec + msec) * 1000;
    char *device_status = "\"Device is active\"";
    // uint32_t ignition_button_status = (uint32_t)uart_strcut->payload[0];
    static uint32_t sq_id = 0;
    memset(json_dev_shadow_msg_buff_gau8, 0, DEV_SHADOW_MSG_BUFF_LEN);
    snprintf(json_dev_shadow_msg_buff_gau8, sizeof(json_dev_shadow_msg_buff_gau8), dev_shadow_mess_json,
             timestamp1,
             device_status,
             ble_status,
             nw_csq_val,
             ign_status,
             imu_status,
             ble_mac_id_g,
             n58_base_fw_ver,
             n58_app_fw_ver,
             s32_app_fw_ver,
             batt_voltage,
             input_voltage,
             tork_fw_ver,
             tork_hw_ver,
             sq_id);
    // bytebeam_ext_echo("\r\n%s\r\n", json_dev_shadow_msg_buff_gau8); // For the verification of the JSON frame

    UART_data_struct dev_shadow_publish_msg;
    dev_shadow_publish_msg.cmd = 0xDE;
    dev_shadow_publish_msg.length_u16 = strlen((char *)json_dev_shadow_msg_buff_gau8);

    memcpy(dev_shadow_publish_msg.payload, json_dev_shadow_msg_buff_gau8, dev_shadow_publish_msg.length_u16);

    // if (get_MQTT_COnnection_Status() && (store_msg_flag == false) && (bytebeam_get_queue_spaceevent_cnt(mqtt_publish_msg_queue)!= 0))
    if (get_MQTT_COnnection_Status())
    {
        sq_id++;
        if (bytebeam_get_queue_spaceevent_cnt(mqtt_publish_msg_queue) == 0)
        {
            // bytebeam_ext_echo("\r\nMQTT Queue full agidhe\r\n");
        }
        else
        {
            bytebeam_ext_echo("\r\npublishing device shadow in MQTT\r\n");
            bytebeam_put_msg_que(mqtt_publish_msg_queue, &dev_shadow_publish_msg, 0);
        }
    }
    return 0;
}
