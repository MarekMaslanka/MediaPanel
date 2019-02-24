#ifndef PCF8574_H
#define PCF8574_H

#include "stm32f1xx_hal.h"

/**
 * Provides possible return values for the functions
 */
typedef enum {
    /**
     * Function call was successful
     */
    PCF8574_OK,

    /**
     * Function call was unsuccessful
     */
    PCF8574_ERROR
} PCF8574_RESULT;

/**
 * PCF8574 handle
 */
typedef struct {
    /**
     * Address of the PCA8574
     */
    uint8_t address;

    /**
     * Timeout in milliseconds for the I2C communication
     */
    uint32_t timeout;

    /**
     * I2C to use
     */
    I2C_HandleTypeDef* i2c;
} pcf8574;

/**
 * Initializes the PCF8574
 * @param	handle - a pointer to the PCF8574 handle
 * @return	whether the function was successful or not
 */
PCF8574_RESULT pcf8574_init(pcf8574* handle);

/**
 * Deinitializes the PCF8574
 * @param	handle - a pointer to the PCF8574 handle
 * @return	whether the function was successful or not
 */
PCF8574_RESULT pcf8574_deinit(pcf8574* handle);

/**
 * Writes a given value to the PCF8574 port
 * @param	handle - a pointer to the PCF8574 handle
 * @param	val - a value to be written to the port
 * @return	whether the function was successful or not
 */
PCF8574_RESULT pcf8574_write(pcf8574* handle, uint8_t val);

/**
 * Reads the current state of the PCF8574 port
 * @param	handle - a pointer to the PCF8574 handle
 * @param	val - a pointer to the variable that will be assigned a value
 * from the chip
 * @return	whether the function was successful or not
 */
PCF8574_RESULT pcf8574_read(pcf8574* handle, uint8_t* val);

#endif /* PCF8574_H */
