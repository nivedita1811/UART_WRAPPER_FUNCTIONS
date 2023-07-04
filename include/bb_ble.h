#ifndef __NWY_BLE_H__
#define __NWY_BLE_H__

#include <stdint.h>

#define BT_ADDRESS_SIZE 6
#define DATA_SIZE (2 * 1024)
#define NWY_BLE_READ_FLAG 0
#define NWY_BLE_WRITE_FLAG 1
#define NWY_BLE_ADD_WRITE_FLAG 2

typedef struct
{
    uint8_t ser_index;       /**< Service index */
    uint8_t char_index;      /**< Characteristic index */
    uint8_t char_uuid[16];   /**< Characteristic UUID */
    uint8_t char_des;        /**< Characteristic description */
    uint8_t char_per;        /**< Characteristic permissions */
    uint16_t permisssion;    /**< Permissions for the characteristic */
    uint8_t char_cp;         /**< Characteristic control point */
} nwy_ble_char_info;

typedef struct
{
    uint32_t ser_index;                      /**< Service index */
    uint8_t ser_uuid[16];                    /**< Service UUID */
    uint8_t char_num;                        /**< Number of characteristics */
    uint8_t p;                               /**< Placeholder */
    nwy_ble_char_info ser_ble_char[OPEN_CREATE_CHAR_MAX_NUM];   /**< Array of characteristics */
} nwy_ble_service_info;

typedef struct
{
    uint8_t ser_id;                          /**< Service ID */
    uint8_t char_id;                         /**< Characteristic ID */
    uint8_t data[DATA_SIZE];                  /**< Data */
    uint16_t datalen;                         /**< Data length */
    uint8_t op;                              /**< Operation */
} nwy_ble_send_info;

typedef struct
{
    uint8_t ser_id;                          /**< Service ID */
    uint8_t char_id;                         /**< Characteristic ID */
    uint8_t data[244];                       /**< Data */
    uint16_t datalen;                         /**< Data length */
} nwy_ble_recv_info;

typedef struct
{
    uint16_t configurationBits;               /**< Configuration bits */
    uint16_t aclHandle;                       /**< ACL handle */
} nwy_gatt_chara_cb_o;

/**
 * @brief Enable BLE functionality.
 */
void bytebeam_ble_enable() 
{
    nwy_ble_enable();
}

/**
 * @brief Send data over BLE.
 * 
 * @param datalen Length of the data.
 * @param data Pointer to the data.
 */
void bytebeam_ble_send_data(uint16_t datalen, char *data) 
{
    nwy_ble_send_data(datalen, data);
}

/**
 * @brief Send indicate data over BLE.
 * 
 * @param datalen Length of the data.
 * @param data Pointer to the data.
 */
void bytebeam_ble_send_indify_data(uint16_t datalen, char *data) 
{
    nwy_ble_send_indify_data(datalen, data);
}

/**
 * @brief Disable BLE functionality.
 */
void bytebeam_ble_disable() 
{
    nwy_ble_disable();
}

/**
 * @brief Receive data from BLE.
 * 
 * @param sel Selection.
 * @return Pointer to the received data.
 */
char* bytebeam_ble_receive_data(int sel) 
{
    return nwy_ble_receive_data(sel);
}

/**
 * @brief Get the version of the BLE.
 * 
 * @return Pointer to the version string.
 */
char* bytebeam_ble_get_version() 
{
    return nwy_ble_get_version();
}

/**
 * @brief Set the device name for BLE.
 * 
 * @param local_name Device name string.
 * @return Status code.
 */
int bytebeam_ble_set_device_name(char *local_name) 
{
    return nwy_ble_set_device_name(local_name);
}

/**
 * @brief Get the device name for BLE.
 * 
 * @return Pointer to the device name string.
 */
char* bytebeam_ble_get_device_name() 
{
    return nwy_ble_get_device_name();
}

/**
 * @brief Update BLE connection parameters.
 * 
 * @param handle Handle.
 * @param intervalMin Minimum connection interval.
 * @param intervalMax Maximum connection interval.
 * @param slaveLatency Slave latency.
 * @param timeoutMulti Connection timeout multiplier.
 * @return Status code.
 */

int bytebeam_ble_update_conn(uint16_t handle, uint16_t intervalMin, uint16_t intervalMax, uint16_t slaveLatency, uint16_t timeoutMulti) 
{
    return nwy_ble_update_conn(handle, intervalMin, intervalMax, slaveLatency, timeoutMulti);
}

/**
 * @brief Set the BLE service.
 * 
 * @param srv_uuid Service UUID.
 * @return Status code.
 */
int bytebeam_ble_set_service(uint8_t srv_uuid[]) 
{
    return nwy_ble_set_service(srv_uuid);
}

/**
 * @brief Set the BLE characteristic.
 * 
 * @param char_index Characteristic index.
 * @param char_uuid Characteristic UUID.
 * @param prop Characteristic property.
 * @return Status code.
 */

int bytebeam_ble_set_character(uint8_t char_index, uint8_t char_uuid[], uint8_t prop) 
{
    return nwy_ble_set_character(char_index, char_uuid, prop);
}

/**
 * @brief Add receive data to BLE.
 * 
 * @param recv_data Pointer to the receive data structure.
 * @return Status code.
 */

int bytebeam_ble_add_recv_data(nwy_ble_recv_info* recv_data) 
{
    return nwy_ble_add_recv_data(recv_data);
}

/**
 * @brief Add send data to BLE.
 * 
 * @param send_data Pointer to the send data structure.
 * @return Status code.
 */
int bytebeam_ble_add_send_data(nwy_ble_send_info* send_data) 
{
    return nwy_ble_add_send_data(send_data);
}

/**
 * @brief Disconnect BLE.
 * 
 * @return Status code.
 */
int bytebeam_ble_disconnect() 
{
    return nwy_ble_disconnect();
}

/**
 * @brief Remove an address from the BLE white list.
 * 
 * @param addr Pointer to the address.
 * @param addr_type Address type.
 * @return Status code.
 */
int bytebeam_ble_add_white_list(uint8_t *addr, uint8_t addr_type) 
{
    return nwy_ble_add_white_list(addr, addr_type);
}

/**
 * @brief Remove an address from the BLE white list.
 * 
 * @param addr Pointer to the address.
 * @param addr_type Address type.
 * @return Status code.
 */
int bytebeam_ble_remove_white_list(uint8_t *addr, uint8_t addr_type) 
{
    return nwy_ble_remove_white_list(addr, addr_type);
}

/**
 * @brief Clean the BLE white list.
 * 
 * @return Status code.
 */
int bytebeam_ble_clean_white_list() 
{
    return nwy_ble_clean_white_list();
}
