#include "nwy_bb_uart.h"
#include "nwy_bb_ble.h"
#include "nwy_bb_osi_api.h"
#include "nwy_bb_usb_serial.h"
#include "nwy_bb_pm.h"

void uart_config(void)
{
   bytebeam_uart_config_t uart_config;
    
   uart_config.name = STM_UART;
   uart_config.baud = 2000000;
   uart_config.parity = UART_PARITY_NONE;
   uart_config.data_size = UART_DATA_BITS_8;
   uart_config.stop_size = UART_STOP_BITS_1;
   uart_config.flow_ctrl = false;
   uart_config.recv_cb = uart_recv_handle;
   uart_config.tx_cb = NULL;
    
   STM_UART_fd = bytebeam_uart_init(uart_config);
    
    if (STM_UART_fd != INVALID_UART_FD)
    {
        bytebeam_uart_set_baud(STM_UART_fd,uart_config.baud);
        bytebeam_uart_reg_recv_cb(STM_UART_fd,uart_config.recv_cb);
        bytebeam_set_rx_frame_timeout(STM_UART_fd, 2); 
        message_queue_init();
    }
    else
    {
        // Error handling for UART initialization failure
    }
}

void bytebeam_uart_printf(const char *format, ...)
{
    char send_buf[512] = "\r\n";
    va_list ap;
    size_t size = 2;
    va_start(ap, format);
    size += vsnprintf(send_buf + size, sizeof(send_buf) - size, format, ap);
    va_end(ap);
    bytebeam_uart_send_data_block(STM_UART_fd, (uint8_t *)send_buf, size);
}

void uart_recv_handle(const char *str, uint32_t length)
{

    static struct timeval tv;
    static double s = 0;
    static double ms = 0;
    static unsigned long long timestamp_start = 0;
    unsigned long long timestamp_end = 0;

    gettimeofday(&tv, NULL);
    s = tv.tv_sec;
    ms = ((double)tv.tv_usec) / 1.0e3;
    if (timestamp_start == 0)
    {
        timestamp_start = (unsigned long long)(s * 1000 + ms);
    }

    timestamp_end = (unsigned long long)(s * 1000 + ms);

    if ((timestamp_end - timestamp_start) > 1000)
    {
        bytebeam_ext_echo("\r\n In uart callback Length Got: %d\r\n", length);
        timestamp_start = timestamp_end;
    }

    // bytebeam_ext_echo("value of data received\r\n");
    if (uart_thread_entry == 1)
    {
        for (int i = 0; i < length; i++)
        {
            //    bytebeam_ext_echo("%X ",str[i]);
            if (FAILED == uart_rx_array_queue_put(str[i]))
            {
                bytebeam_ext_echo("Put queue failure of data: %d", str[i]);
            }
        }
    }
    // bytebeam_ext_echo("\r\n");
}

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

    return 0; // return value needs to be set according to the function's requirements.
}

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
            bytebeam_uart_send_data_byte_by_byte(STM_UART_fd, can_data, 8 + len, 1);
            bytebeam_usleep(1);
        }
        bytebeam_sleep(1);
        bytebeam_semaphore_release(uart_tx_semaphore);
    }

    return 0;
}

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
        int ret_val = bytebeam_uart_send_data_byte_by_byte(STM_UART_fd, can_data, 21, 50);
        bytebeam_semaphore_release(uart_tx_semaphore);
        return ret_val;  
    }
    
    return -1;  // Failure in acquiring semaphore
}

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
