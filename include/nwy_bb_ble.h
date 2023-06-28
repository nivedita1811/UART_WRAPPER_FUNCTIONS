// Declaration for the ByteBeam wrapper function
int bytebeam_ble_send_data(uint16 datalen, char *data);

// Definition for the ByteBeam wrapper function
int bytebeam_ble_send_data(uint16 datalen, char *data) {
    // Simply call the original function and return its result
    return nwy_ble_send_data(datalen, data);
}