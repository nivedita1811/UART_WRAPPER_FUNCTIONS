#include <neoway_api.h>

typedef enum 
{
    UART_PARITY_NONE,   // No parity
    UART_PARITY_EVEN,   // Even parity
    UART_PARITY_ODD     // Odd parity
} uart_parity_t;

typedef enum 
{
    UART_DATA_BITS_5,   // 5 data bits
    UART_DATA_BITS_6,   // 6 data bits
    UART_DATA_BITS_7,   // 7 data bits
    UART_DATA_BITS_8    // 8 data bits
} uart_data_bits_t;

typedef enum 
{
    UART_STOP_BITS_1,   // 1 stop bit
    UART_STOP_BITS_1_5, // 1.5 stop bits
    UART_STOP_BITS_2    // 2 stop bits
} uart_stop_bits_t;

typedef enum 
{
    UART_MODE_BLOCKING,       // UART interface will block during data transfer
    UART_MODE_NON_BLOCKING,   // UART interface will not block during data transfer
    UART_MODE_INTERRUPT,      // UART interface will use interrupts for data transfer
    UART_MODE_POLLING,        // UART interface will use polling for data transfer
    UART_MODE_FULL_DUPLEX,    // UART interface will support simultaneous sending and receiving
    UART_MODE_HALF_DUPLEX     // UART interface will support either sending or receiving, but not at the same time
} nwy_uart_mode_t;

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

int bytebeam_uart_init(bytebeam_uart_config_t config);
bool bytebeam_uart_set_baud(uint8_t hd, uint32_t baud);
bool bytebeam_uart_get_baud(uint8_t hd, uint32_t* baud);
bool bytebeam_uart_set_parameters(uint8_t hd, uart_parity_t parity, uart_data_bits_t data_size, uart_stop_bits_t stop_size, bool flow_ctrl);
bool bytebeam_uart_get_parameters(uint8_t hd, uart_parity_t* parity, uart_data_bits_t* data_size, uart_stop_bits_t* stop_size, bool* flow_ctrl);
int bytebeam_uart_send_data_byte_by_byte(uint8_t hd, uint8_t* data_ptr, uint32_t length, uint32_t delay_ms);
int bytebeam_uart_send_data_block(uint8_t hd, uint8_t* data_ptr, uint32_t length);
int bytebeam_uart_send_single_byte(uint8_t hd, uint8_t data);
bool bytebeam_receive_byte_by_byte(bytebeam_uart_recv_callback_t recv_cb);
bool bytebeam_receive_data_block(bytebeam_uart_recv_callback_t recv_cb);
bool bytebeam_receive_single_byte(bytebeam_uart_recv_callback_t recv_cb);
bool bytebeam_uart_register_receive_callback(uint8_t hd, bytebeam_uart_recv_callback_t recv_cb);
bool bytebeam_uart_register_transmit_callback(uint8_t hd, bytebeam_uart_send_callback_t tx_cb);
bool bytebeam_uart_deinit(uint8_t hd);
bool bytebeam_set_rx_frame_timeout(uint8_t hd, int time);
bool bytebeam_at_uart_send(bytebeam_uart_port_t port, void* data, size_t size);
