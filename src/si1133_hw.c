/*
 * si1133_hw.c
 *
 *  Created on: 31 aug. 2024
 *      Author: Ludo
 */

#include "si1133_hw.h"

#ifndef SI1133_DRIVER_DISABLE_FLAGS_FILE
#include "si1133_driver_flags.h"
#endif
#include "si1133.h"
#include "types.h"

#ifndef SI1133_DRIVER_DISABLE

/*** SI1133 HW functions ***/

/*******************************************************************/
SI1133_status_t __attribute__((weak)) SI1133_HW_init(void) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
SI1133_status_t __attribute__((weak)) SI1133_HW_de_init(void) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
SI1133_status_t __attribute__((weak)) SI1133_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    /* To be implemented */
    UNUSED(i2c_address);
    UNUSED(data);
    UNUSED(data_size_bytes);
    UNUSED(stop_flag);
    return status;
}

/*******************************************************************/
SI1133_status_t __attribute__((weak)) SI1133_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    /* To be implemented */
    UNUSED(i2c_address);
    UNUSED(data);
    UNUSED(data_size_bytes);
    return status;
}

/*******************************************************************/
SI1133_status_t __attribute__((weak)) SI1133_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    /* To be implemented */
    UNUSED(delay_ms);
    return status;
}

#endif /* SI1133_DRIVER_DISABLE */
