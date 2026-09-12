/*
 * si1133.h
 *
 *  Created on: 31 aug. 2024
 *      Author: Ludo
 */

#ifndef __SI1133_H__
#define __SI1133_H__

#ifndef SI1133_DRIVER_DISABLE_FLAGS_FILE
#include "si1133_driver_flags.h"
#endif
#include "error.h"
#include "types.h"

/*** SI1133 structures ***/

/*!******************************************************************
 * \enum SI1133_command_error_t
 * \brief SI1133 internal command error codes.
 *******************************************************************/
typedef enum {
    SI1133_COMMAND_ERROR_INVALID = 0x00,
    SI1133_COMMAND_ERROR_LOCATION = 0x01,
    SI1133_COMMAND_ERROR_SATURATION = 0x02,
    SI1133_COMMAND_ERROR_OVERFLOW = 0x03,
    SI1133_COMMAND_ERROR_LAST = 0x0F
} SI1133_command_error_t;

/*!******************************************************************
 * \enum SI1133_status_t
 * \brief SI1133 driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    SI1133_SUCCESS = 0,
    SI1133_ERROR_NULL_PARAMETER,
    SI1133_ERROR_REGISTER,
    SI1133_ERROR_WRITE_BUFFER_SIZE,
    SI1133_ERROR_READY,
    SI1133_ERROR_COMMAND_COMPLETION,
    SI1133_ERROR_PARAMETER_COMPLETION = (SI1133_ERROR_COMMAND_COMPLETION + SI1133_COMMAND_ERROR_LAST),
    SI1133_ERROR_COMMAND_COUNTER = (SI1133_ERROR_PARAMETER_COMPLETION + SI1133_COMMAND_ERROR_LAST),
    SI1133_ERROR_TIMEOUT,
    // Low level drivers errors.
    SI1133_ERROR_HW_FUNCTION_NOT_IMPLEMENTED,
    SI1133_ERROR_BASE_I2C = ERROR_BASE_STEP,
    SI1133_ERROR_BASE_DELAY = (SI1133_ERROR_BASE_I2C + SI1133_DRIVER_I2C_ERROR_BASE_LAST),
    // Last base value.
    SI1133_ERROR_BASE_LAST = (SI1133_ERROR_BASE_DELAY + SI1133_DRIVER_DELAY_ERROR_BASE_LAST)
} SI1133_status_t;

#ifndef SI1133_DRIVER_DISABLE

/*!******************************************************************
 * \enum SI1133_light_status_t
 * \brief SI1133 driver light data status.
 *******************************************************************/
typedef enum {
    SI1133_LIGHT_STATUS_AVAILABLE = 0,
    SI1133_LIGHT_STATUS_SENSOR_ERROR,
    SI1133_LIGHT_STATUS_SENSOR_SATURATION,
    SI1133_LIGHT_STATUS_SENSOR_OVERFLOW,
    SI1133_LIGHT_STATUS_LAST
} SI1133_light_status_t;

/*** SI1133 functions ***/

/*!******************************************************************
 * \fn SI1133_status_t SI1133_init(void)
 * \brief Init SI1133 driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SI1133_status_t SI1133_init(void);

/*!******************************************************************
 * \fn SI1133_status_t SI1133_de_init(void)
 * \brief Release SI1133 driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SI1133_status_t SI1133_de_init(void);

/*!******************************************************************
 * \fn SI1133_status_t SI1133_get_light_uv_index(uint8_t i2c_address, int32_t* light_mlux, int32_t* uv_index_duvi, SI1133_light_status_t* light_status)
 * \brief Perform ambient light and UV index measurements.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[out]  light_mlux: Pointer to integer that will contain the ambient light in mlux.
 * \param[out]  uv_index_duvi: Pointer to integer that will contain the UV index in dUVI.
 * \param[out]  light_status: Status of the output data.
 * \retval      Function execution status.
 *******************************************************************/
SI1133_status_t SI1133_get_light_uv_index(uint8_t i2c_address, int32_t* light_mlux, int32_t* uv_index_duvi, SI1133_light_status_t* light_status);

/*******************************************************************/
#define SI1133_exit_error(base) { ERROR_check_exit(si1133_status, SI1133_SUCCESS, base) }

/*******************************************************************/
#define SI1133_stack_error(base) { ERROR_check_stack(si1133_status, SI1133_SUCCESS, base) }

/*******************************************************************/
#define SI1133_stack_exit_error(base, code) { ERROR_check_stack_exit(si1133_status, SI1133_SUCCESS, base, code) }

#endif /* SI1133_DRIVER_DISABLE */

#endif /* __SI1133_H__ */
