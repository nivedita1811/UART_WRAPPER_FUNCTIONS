#include "nwy_bb_ble.h"
#include "nwy_bb_osi_api.h"
#include "nwy_bb_usb_serial.h"
#include "nwy_bb_pm.h"

/**
 * ==============================================================================================
 * Enum to represent the type of UART parity. 
 * 
 * UART_PARITY_NONE: No parity is used.
 * UART_PARITY_EVEN: Even parity is used.
 * UART_PARITY_ODD: Odd parity is used.
 * ==============================================================================================
 */
typedef enum 
{
    UART_PARITY_NONE,   // No parity
    UART_PARITY_EVEN,   // Even parity
    UART_PARITY_ODD     // Odd parity
} uart_parity_t;

/**
 * ==============================================================================================
 * Enum to represent the size of UART data bits. 
 * 
 * UART_DATA_BITS_7: 7 data bits are used.
 * UART_DATA_BITS_8: 8 data bits are used.
 * ==============================================================================================
 */
typedef enum 
{
    UART_DATA_BITS_7,   // 7 data bits
    UART_DATA_BITS_8    // 8 data bits
} uart_data_bits_t;

/**
 * ==============================================================================================
 * Enum to represent the size of UART stop bits. 
 * 
 * UART_STOP_BITS_1: 1 stop bit is used.
 * UART_STOP_BITS_2: 2 stop bits are used.
 * ==============================================================================================
 */
typedef enum 
{
    UART_STOP_BITS_1,   // 1 stop bit
    UART_STOP_BITS_2    // 2 stop bits
} uart_stop_bits_t;

/**
 * ==============================================================================================
 * Enum to represent the mode of UART. 
 * 
 * UART_MODE_AT: AT mode is used.
 * UART_MODE_DATA: Data mode is used.
 * ==============================================================================================
 */
typedef enum 
{
    UART_MODE_AT = 0,
    UART_MODE_DATA = 1
} nwy_uart_mode_t;

/**
 * ==============================================================================================
 * Struct to represent the configuration of ByteBeam UART.
 * 
 * Contains configuration parameters including name, baud rate, parity, data size, stop size, 
 * flow control, mode, and receive and transmit callbacks.
 * ==============================================================================================
 */
typedef struct 
{
    uint32_t name;
    uint32_t baud;
    uart_parity_t parity;
    uart_data_bits_t data_size;
    uart_stop_bits_t stop_size;
    bool flow_ctrl;
    nwy_uart_mode_t mode; 
    void (*recv_cb)(uint8_t* data, uint32_t length);
    void (*tx_cb)(void);
} bytebeam_uart_config_t;

/**
=====================================================================================================================================================

@enum Name           : uart_status
@b Scope             : Public
@n@n@b Description   : Enumerates the possible statuses after attempting UART initialization.
@param Input Data    : None
@return Return Value : None

=====================================================================================================================================================
*/
typedef enum {
    UART_INIT_OK = 0,               // UART initialized successfully
    UART_INIT_NULL_CFG,             // Null configuration error during UART initialization
    UART_INIT_FAIL,                 // UART initialization failed
    UART_SET_BAUD_FAIL,             // Baud rate setting failed
    UART_REG_CB_FAIL,               // Callback registration failed
    UART_SET_TIMEOUT_FAIL,          // Timeout setting failed
    UART_QUEUE_INIT_FAIL            // Queue initialization failed
} uart_status;

/**
=====================================================================================================================================================

@enum Name           : print_status
@b Scope             : Public
@n@n@b Description   : Enumerates the possible statuses after attempting to print.
@param Input Data    : None
@return Return Value : None

=====================================================================================================================================================
*/
typedef enum {
    PRINT_SUCCESS,                    // Print operation was successful
    PRINT_FAILURE_NULL_POINTER,       // Failed due to null pointer
    PRINT_FAILURE                     // Print operation failed
} print_status;

/**
=====================================================================================================================================================

@enum Name           : publish_status
@b Scope             : Public
@n@n@b Description   : Enumerates the possible statuses after attempting to publish.
@param Input Data    : None
@return Return Value : None

=====================================================================================================================================================
*/
typedef enum {
    PUBLISH_SUCCESS,                 // Publish operation was successful
    PUBLISH_FAILURE_QUEUE_FULL,      // Failed due to the queue being full
} publish_status;

/**
=====================================================================================================================================================

@enum Name           : send_status
@b Scope             : Public
@n@n@b Description   : Enumerates the possible statuses after attempting to send.
@param Input Data    : None
@return Return Value : None

=====================================================================================================================================================
*/
typedef enum {
    SEND_SUCCESS,                    // Send operation was successful
    SEND_FAILURE_SEMAPHORE,          // Failed due to semaphore
} send_status;

/**
=====================================================================================================================================================

@enum Name           : send_can_status
@b Scope             : Public
@n@n@b Description   : Enumerates the possible statuses after attempting to send over CAN.
@param Input Data    : None
@return Return Value : None

=====================================================================================================================================================
*/
typedef enum {
    SEND_CAN_SUCCESS,                // Send operation over CAN was successful
    SEND_CAN_FAILURE,                // Send operation over CAN failed
    SEND_CAN_SEMAPHORE_FAILURE       // Failed due to semaphore when attempting to send over CAN
} send_can_status;


/**
 * ==============================================================================================
 * Enum to represent the status of queue operation. 
 * 
 * QUEUE_PUT_SUCCESS: Queue operation was successful.
 * QUEUE_PUT_FAILURE: Queue operation failed.
 * QUEUE_PUT_FULL: Queue is full.
 * QUEUE_PUT_NO_ENTRY: No entry in the queue.
 * ==============================================================================================
 */
typedef enum {
    QUEUE_PUT_SUCCESS,
    QUEUE_PUT_FAILURE,
    QUEUE_PUT_FULL,
    QUEUE_PUT_NO_ENTRY
} QueuePutStatus;

/**
 * ==============================================================================================
 * These are the function prototypes for the Bytebeam UART interface. They include initialization,
 * configuration (baud rate, parameters), data transmission and reception (in bytes, data blocks, 
 * and single byte formats), registration of callbacks for data reception and transmission, 
 * de-initialization, setting the receive frame timeout, and sending AT commands.
 * 
 * The specific details of each function, including parameters and return types, are provided in 
 * their respective definitions in the source code.
 * ==============================================================================================
 */
int bytebeam_uart_init(bytebeam_uart_config_t config);
bool bytebeam_uart_set_baud(uint8_t uart_handle, uint32_t baud);
bool bytebeam_uart_get_baud(uint8_t uart_handle, uint32_t* baud);
bool bytebeam_uart_set_parameters(uint8_t uart_handle, uart_parity_t parity, uart_data_bits_t data_size, uart_stop_bits_t stop_size, bool flow_ctrl);
bool bytebeam_uart_get_parameters(uint8_t uart_handle, uart_parity_t* parity, uart_data_bits_t* data_size, uart_stop_bits_t* stop_size, bool* flow_ctrl);
int bytebeam_uart_send_data_in_bytes(uint8_t uart_handle, uint8_t* data_ptr, uint32_t length, uint32_t delay_ms);
int bytebeam_uart_send_data_block(uint8_t uart_handle, uint8_t* data_ptr, uint32_t length);
int bytebeam_uart_send_single_byte(uint8_t uart_handle, uint8_t data);
bool bytebeam_receive_in_bytes(bytebeam_uart_recv_callback_t recv_cb);
bool bytebeam_receive_data_block(bytebeam_uart_recv_callback_t recv_cb);
bool bytebeam_receive_single_byte(bytebeam_uart_recv_callback_t recv_cb);
bool bytebeam_uart_register_receive_callback(uint8_t uart_handle, bytebeam_uart_recv_callback_t recv_cb);
bool bytebeam_uart_register_transmit_callback(uint8_t uart_handle, bytebeam_uart_send_callback_t tx_cb);
bool bytebeam_uart_deinit(uint8_t uart_handle);
bool bytebeam_set_rx_frame_timeout(uint8_t uart_handle, int time);
bool bytebeam_at_uart_send(bytebeam_uart_port_t port, void* data, size_t size);
