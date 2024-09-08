/*
 * si1133_hw.h
 *
 *  Created on: 31 aug. 2024
 *      Author: Ludo
 */

#ifndef __SI1133_HW_H__
#define __SI1133_HW_H__

#ifndef SI1133_DRIVER_DISABLE_FLAGS_FILE
#include "si1133_driver_flags.h"
#endif
#include "si1133.h"
#include "types.h"

#ifndef SI1133_DRIVER_DISABLE

/*** SI1133 HW functions ***/

/*!******************************************************************
 * \fn SI1133_status_t SI1133_HW_init(void)
 * \brief Init SI1133 hardware interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SI1133_status_t SI1133_HW_init(void);

/*!******************************************************************
 * \fn SI1133_status_t SI1133_HW_de_init(void)
 * \brief Release SI1133 hardware interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SI1133_status_t SI1133_HW_de_init(void);

/*!******************************************************************
 * \fn SI1133_status_t SI1133_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag)
 * \brief Write data to sensor over I2C bus.
 * \param[in]  	i2c_address: 7-bits sensor address.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[in]	stop_flag: Generate stop condition at the end of the transfer if non zero.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SI1133_status_t SI1133_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag);

/*!******************************************************************
 * \fn SI1133_status_t SI1133_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes)
 * \brief Read data from sensor over I2C bus.
 * \param[in]  	i2c_address: 7-bits sensor address.
 * \param[in]	data_size_bytes: Number of bytes to read.
 * \param[out]	data: Byte array that will contain the read data.
 * \retval		Function execution status.
 *******************************************************************/
SI1133_status_t SI1133_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes);

/*!******************************************************************
 * \fn SI1133_status_t SI1133_HW_delay_milliseconds(uint32_t delay_ms)
 * \brief Delay function.
 * \param[in]  	delay_ms: Delay to wait in ms.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SI1133_status_t SI1133_HW_delay_milliseconds(uint32_t delay_ms);

#endif /* SI1133_DRIVER_DISABLE */

#endif /* __SI1133_HW_H__ */
