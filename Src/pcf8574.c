#include "pcf8574.h"

PCF8574_RESULT pcf8574_init(pcf8574* handle) {
    handle->address &= 0x07;
    handle->address <<= 1;
    handle->address |= 0x40;
    return PCF8574_OK;
}

PCF8574_RESULT pcf8574_deinit(pcf8574* handle) { return PCF8574_OK; }

PCF8574_RESULT pcf8574_write(pcf8574* handle, uint8_t val) {
    if (HAL_I2C_Master_Transmit(handle->i2c, handle->address, &val, 1,
                                handle->timeout) != HAL_OK) {
        return PCF8574_ERROR;
    }
    return PCF8574_OK;
}

PCF8574_RESULT pcf8574_read(pcf8574* handle, uint8_t* val) {
    if (HAL_I2C_Master_Receive(handle->i2c, handle->address, val, 1,
                               handle->timeout) != HAL_OK) {
        return PCF8574_ERROR;
    }
    return PCF8574_OK;
}
