/**
 * @file bytebeam_uart.h
 * @brief ByteBeam UART API functions.
 */

#ifndef BYTEBEAM_UART_H
#define BYTEBEAM_UART_H

#include <stdint.h>
#include <stdbool.h>

/** UART parity options. */
typedef enum uart_parity {
    UART_PARITY_NONE,   /**< No parity */
    UART_PARITY_EVEN,   /**< Even parity */
    UART_PARITY_ODD     /**< Odd parity */
} uart_parity_t;

/** UART data bits options. */
typedef enum uart_data_bits {
    UART_DATA_BITS_5,   /**< 5 data bits */
    UART_DATA_BITS_6,   /**< 6 data bits */
    UART_DATA_BITS_7,   /**< 7 data bits */
    UART_DATA_BITS_8    /**< 8 data bits */
} uart_data_bits_t;

/** UART stop bits options. */
typedef enum uart_stop_bits {
    UART_STOP_BITS_1,   /**< 1 stop bit */
    UART_STOP_BITS_1_5, /**< 1.5 stop bits */
    UART_STOP_BITS_2    /**< 2 stop bits */
} uart_stop_bits_t;

/** UART mode options. */
typedef enum nwy_uart_mode {
    UART_MODE_BLOCKING,       /**< UART interface will block during data transfer */
    UART_MODE_NON_BLOCKING,   /**< UART interface will not block during data transfer */
    UART_MODE_INTERRUPT,      /**< UART interface will use interrupts for data transfer */
    UART_MODE_POLLING,        /**< UART interface will use polling for data transfer */
    UART_MODE_FULL_DUPLEX,    /**< UART interface will support simultaneous sending and receiving */
    UART_MODE_HALF_DUPLEX     /**< UART interface will support either sending or receiving, but not at the same time */
} nwy_uart_mode_t;

/** UART configuration structure. */
typedef struct bytebeam_uart_config {
    uint32_t name;                   /**< UART name. */
    uint32_t baud;                   /**< Baud rate. */
    uart_parity_t parity;            /**< Parity mode. */
    uart_data_bits_t data_size;      /**< Data bits size. */
    uart_stop_bits_t stop_size;      /**< Stop bits size. */
    bool flow_ctrl;                  /**< Flow control. */
    nwy_uart_mode_t mode;            /**< UART mode. */
    void (*recv_cb)(uint8_t* data, uint32_t length); /**< Receive callback function. */
    void (*tx_cb)(void);             /**< Transmit callback function. */
} bytebeam_uart_config_t;

/**
 * @brief Initialize the UART interface.
 * @param config The UART configuration.
 * @return The UART handle, or -1 if initialization failed.
 */
int bytebeam_uart_init(bytebeam_uart_config_t config);

/**
 * @brief Set the baud rate for the UART interface.
 * @param hd The UART handle.
 * @param baud The new baud rate.
 * @return True if the baud rate was successfully set, false otherwise.
 */
bool bytebeam_uart_set_baud(uint8_t hd, uint32_t baud);

/**
 * @brief Get the baud rate of the UART interface.
 * @param hd The UART handle.
 * @param baud Pointer to store the baud rate.
 * @return True if the baud rate was successfully retrieved, false otherwise.
 */
bool bytebeam_uart_get_baud(uint8_t hd, uint32_t* baud);

/**
 * @brief Set the parameters (parity, data bits, stop bits, flow control) of the UART interface.
 * @param hd The UART handle.
 * @param parity The parity mode.
 * @param data_size The data bits size.
 * @param stop_size The stop bits size.
 * @param flow_ctrl The flow control.
 * @return True if the parameters were successfully set, false otherwise.
 */
bool bytebeam_uart_set_parameters(uint8_t hd, uart_parity_t parity, uart_data_bits_t data_size, uart_stop_bits_t stop_size, bool flow_ctrl);

/**
 * @brief Get the parameters (parity, data bits, stop bits, flow control) of the UART interface.
 * @param hd The UART handle.
 * @param parity Pointer to store the parity mode.
 * @param data_size Pointer to store the data bits size.
 * @param stop_size Pointer to store the stop bits size.
 * @param flow_ctrl Pointer to store the flow control.
 * @return True if the parameters were successfully retrieved, false otherwise.
 */
bool bytebeam_uart_get_parameters(uint8_t hd, uart_parity_t* parity, uart_data_bits_t* data_size, uart_stop_bits_t* stop_size, bool* flow_ctrl);

/**
 * @brief Send data in bytes through the UART interface with a delay between each byte.
 * @param hd The UART handle.
 * @param data_ptr Pointer to the data to be sent.
 * @param length The length of the data to be sent.
 * @param delay_ms The delay between each byte in milliseconds.
 * @return The number of bytes sent, or -1 if the send operation failed.
 */
int bytebeam_uart_send_data_in_bytes(uint8_t hd, uint8_t* data_ptr, uint32_t length, uint32_t delay_ms);

/**
 * @brief Send datacontinued...

/**
 * @brief Send data block through the UART interface.
 * @param hd The UART handle.
 * @param data_ptr Pointer to the data to be sent.
 * @param length The length of the data to be sent.
 * @return The number of bytes sent, or -1 if the send operation failed.
 */
int bytebeam_uart_send_data_block(uint8_t hd, uint8_t* data_ptr, uint32_t length);

/**
 * @brief Send a single byte through the UART interface.
 * @param hd The UART handle.
 * @param data The byte to be sent.
 * @return The number of bytes sent, or -1 if the send operation failed.
 */
int bytebeam_uart_send_single_byte(uint8_t hd, uint8_t data);

/**
 * @brief Receive data in bytes using a callback function.
 * @param recv_cb The receive callback function.
 * @return True if the receive operation was successful, false otherwise.
 */
bool bytebeam_receive_in_bytes(bytebeam_uart_recv_callback_t recv_cb);

/**
 * @brief Receive data block using a callback function.
 * @param recv_cb The receive callback function.
 * @return True if the receive operation was successful, false otherwise.
 */
bool bytebeam_receive_data_block(bytebeam_uart_recv_callback_t recv_cb);

/**
 * @brief Receive a single byte using a callback function.
 * @param recv_cb The receive callback function.
 * @return True if the receive operation was successful, false otherwise.
 */
bool bytebeam_receive_single_byte(bytebeam_uart_recv_callback_t recv_cb);

/**
 * @brief Register a receive callback function for the UART interface.
 * @param hd The UART handle.
 * @param recv_cb The receive callback function.
 * @return True if the receive callback function was successfully registered, false otherwise.
 */
bool bytebeam_uart_register_receive_callback(uint8_t hd, bytebeam_uart_recv_callback_t recv_cb);

/**
 * @brief Register a transmit callback function for the UART interface.
 * @param hd The UART handle.
 * @param tx_cb The transmit callback function.
 * @return True if the transmit callback function was successfully registered, false otherwise.
 */
bool bytebeam_uart_register_transmit_callback(uint8_t hd, bytebeam_uart_send_callback_t tx_cb);

/**
 * @brief Deinitialize the UART interface.
 * @param hd The UART handle.
 * @return True if the UART interface was successfully deinitialized, false otherwise.
 */
bool bytebeam_uart_deinit(uint8_t hd);

/**
 * @brief Set the receive frame timeout for the UART interface.
 * @param hd The UART handle.
 * @param time The receive frame timeout value.
 * @return True if the receive frame timeout was successfully set, false otherwise.
 */
bool bytebeam_set_rx_frame_timeout(uint8_t hd, int time);

/**
 * @brief Send data through the AT UART port.
 * @param port The AT UART port.
 * @param data Pointer to the data to be sent.
 * @param size The size of the data to be sent.
 * @return True if the send operation was successful, false otherwise.
 */
bool bytebeam_at_uart_send(bytebeam_uart_port_t port, void* data, size_t size);

#endif /* BYTEBEAM_UART_H */
