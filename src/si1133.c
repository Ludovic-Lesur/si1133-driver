/*
 * si1133.c
 *
 *  Created on: 31 aug. 2024
 *      Author: Ludo
 */

#include "si1133.h"

#ifndef SI1133_DRIVER_DISABLE_FLAGS_FILE
#include "si1133_driver_flags.h"
#endif
#include "si1133_hw.h"
#include "types.h"

#ifndef SI1133_DRIVER_DISABLE

/*** SI1133 local macros ***/

#define SI1133_BURST_WRITE_MAX_SIZE     10

#define SI1133_SUB_DELAY_MS             10
#define SI1133_TIMEOUT_MS               2000

/*** SI1133 local structures ***/

/*******************************************************************/
typedef enum {
    SI1133_REGISTER_PART_ID = 0x00,
    SI1133_REGISTER_HW_ID = 0x01,
    SI1133_REGISTER_REV_ID = 0x02,
    SI1133_REGISTER_HOSTIN0 = 0x0A,
    SI1133_REGISTER_COMMAND = 0x0B,
    SI1133_REGISTER_IRQ_ENABLE = 0x0F,
    SI1133_REGISTER_RESPONSE1 = 0x10,
    SI1133_REGISTER_RESPONSE0 = 0x11,
    SI1133_REGISTER_IRQ_STATUS = 0x12,
    SI1133_REGISTER_HOSTOUT0 = 0x13,
    SI1133_REGISTER_HOSTOUT1 = 0x14,
    SI1133_REGISTER_HOSTOUT2 = 0x15,
    SI1133_REGISTER_HOSTOUT3 = 0x16,
    SI1133_REGISTER_HOSTOUT4 = 0x17,
    SI1133_REGISTER_HOSTOUT5 = 0x18,
    SI1133_REGISTER_HOSTOUT6 = 0x19,
    SI1133_REGISTER_HOSTOUT7 = 0x1A,
    SI1133_REGISTER_HOSTOUT8 = 0x1B,
    SI1133_REGISTER_HOSTOUT9 = 0x1C,
    SI1133_REGISTER_HOSTOUT10 = 0x1D,
    SI1133_REGISTER_HOSTOUT11 = 0x1E,
    SI1133_REGISTER_HOSTOUT12 = 0x1F,
    SI1133_REGISTER_HOSTOUT13 = 0x20,
    SI1133_REGISTER_HOSTOUT14 = 0x21,
    SI1133_REGISTER_HOSTOUT15 = 0x22,
    SI1133_REGISTER_HOSTOUT16 = 0x23,
    SI1133_REGISTER_HOSTOUT17 = 0x24,
    SI1133_REGISTER_HOSTOUT18 = 0x25,
    SI1133_REGISTER_HOSTOUT19 = 0x26,
    SI1133_REGISTER_HOSTOUT20 = 0x27,
    SI1133_REGISTER_HOSTOUT21 = 0x28,
    SI1133_REGISTER_HOSTOUT22 = 0x29,
    SI1133_REGISTER_HOSTOUT23 = 0x2A,
    SI1133_REGISTER_HOSTOUT24 = 0x2B,
    SI1133_REGISTER_HOSTOUT25 = 0x2C,
    SI1133_REGISTER_LAST
} SI1133_register_t;

/*******************************************************************/
typedef enum {
    SI1133_PARAMETER_I2C_ADDR = 0x00,
    SI1133_PARAMETER_CH_LIST = 0x01,
    SI1133_PARAMETER_ADCCONFIG0 = 0x02,
    SI1133_PARAMETER_ADCSENS0 = 0x03,
    SI1133_PARAMETER_ADCPOST0 = 0x04,
    SI1133_PARAMETER_MEASCONFIG0 = 0x05,
    SI1133_PARAMETER_ADCCONFIG1 = 0x06,
    SI1133_PARAMETER_ADCSENS1 = 0x07,
    SI1133_PARAMETER_ADCPOST1 = 0x08,
    SI1133_PARAMETER_MEASCONFIG1 = 0x09,
    SI1133_PARAMETER_ADCCONFIG2 = 0x0A,
    SI1133_PARAMETER_ADCSENS2 = 0x0B,
    SI1133_PARAMETER_ADCPOST2 = 0x0C,
    SI1133_PARAMETER_MEASCONFIG2 = 0x0D,
    SI1133_PARAMETER_ADCCONFIG3 = 0x0E,
    SI1133_PARAMETER_ADCSENS3 = 0x0F,
    SI1133_PARAMETER_ADCPOST3 = 0x10,
    SI1133_PARAMETER_MEASCONFIG3 = 0x11,
    SI1133_PARAMETER_ADCCONFIG4 = 0x12,
    SI1133_PARAMETER_ADCSENS4 = 0x13,
    SI1133_PARAMETER_ADCPOST4 = 0x14,
    SI1133_PARAMETER_MEASCONFIG4 = 0x15,
    SI1133_PARAMETER_ADCCONFIG5 = 0x16,
    SI1133_PARAMETER_ADCSENS5 = 0x17,
    SI1133_PARAMETER_ADCPOST5 = 0x18,
    SI1133_PARAMETER_MEASCONFIG5 = 0x19,
    SI1133_PARAMETER_MEASRATE_H = 0x1A,
    SI1133_PARAMETER_MEASRATE_L = 0x1B,
    SI1133_PARAMETER_MEASCOUNT0 = 0x1C,
    SI1133_PARAMETER_MEASCOUNT1 = 0x1D,
    SI1133_PARAMETER_MEASCOUNT2 = 0x1E,
    SI1133_PARAMETER_THRESHOLD0_H = 0x25,
    SI1133_PARAMETER_THRESHOLD0_L = 0x26,
    SI1133_PARAMETER_THRESHOLD1_H = 0x27,
    SI1133_PARAMETER_THRESHOLD1_L = 0x28,
    SI1133_PARAMETER_THRESHOLD2_H = 0x29,
    SI1133_PARAMETER_THRESHOLD2_L = 0x2A,
    SI1133_PARAMETER_BURST = 0x2B,
    SI1133_PARAMETER_LAST
} SI1133_parameter_t;

/*******************************************************************/
typedef enum {
    SI1133_COMMAND_RESET_CMD_CTR = 0x00,
    SI1133_COMMAND_RESET = 0x01,
    SI1133_COMMAND_NEW_ADDR = 0x02,
    SI1133_COMMAND_FORCE_CH = 0x11,
    SI1133_COMMAND_PAUSE_CH = 0x12,
    SI1133_COMMAND_START = 0x13,
    SI1133_COMMAND_PARAM_QUERY = 0x40,
    SI1133_COMMAND_PARAM_SET = 0x80,
    SI1133_COMMAND_LAST
} SI1133_commmand_t;

/*** SI1133 local function declarations ***/

static SI1133_status_t _SI1133_send_command(uint8_t i2c_address, SI1133_commmand_t command);

/*** SI1133 local functions ***/

/*******************************************************************/
static SI1133_status_t _SI1133_write_register(uint8_t i2c_address, SI1133_register_t register_address, uint8_t* data, uint8_t data_size_bytes) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t register_write_command[SI1133_BURST_WRITE_MAX_SIZE];
    uint8_t tx_buf_length = (data_size_bytes + 1);
    uint8_t idx = 0;
    // Clamp buffer length.
    if (tx_buf_length >= SI1133_BURST_WRITE_MAX_SIZE) {
        tx_buf_length = (SI1133_BURST_WRITE_MAX_SIZE - 1);
    }
    // Build TX buffer.
    register_write_command[0] = register_address;
    for (idx = 1; idx < tx_buf_length; idx++) {
        register_write_command[idx] = data[idx - 1];
    }
    // I2C transfer.
    status = SI1133_HW_i2c_write(i2c_address, register_write_command, tx_buf_length, 1);
    if (status != SI1133_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_read_register(uint8_t i2c_address, SI1133_register_t register_address, uint8_t* value) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    // I2C transfer.
    status = SI1133_HW_i2c_write(i2c_address, &register_address, 1, 1);
    if (status != SI1133_SUCCESS) goto errors;
    status = SI1133_HW_i2c_read(i2c_address, value, 1);
    if (status != SI1133_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_wait_flag(uint8_t i2c_address, SI1133_register_t register_address, uint8_t bit_index, SI1133_status_t timeout_error) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t reg_value = 0;
    uint32_t loop_count_ms = 0;
    // Read register.
    status = _SI1133_read_register(i2c_address, register_address, &reg_value);
    if (status != SI1133_SUCCESS) goto errors;
    // Wait for flag to be set.
    while ((reg_value & (0b1 << bit_index)) == 0) {
        // Low power delay.
        status = SI1133_HW_delay_milliseconds(SI1133_SUB_DELAY_MS);
        if (status != SI1133_SUCCESS) goto errors;
        // Exit if timeout.
        loop_count_ms += SI1133_SUB_DELAY_MS;
        if (loop_count_ms > SI1133_TIMEOUT_MS) {
            status = timeout_error;
            goto errors;
        }
        // Read register.
        status = _SI1133_read_register(i2c_address, register_address, &reg_value);
        if (status != SI1133_SUCCESS) goto errors;
        
    }
errors:
    return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_get_status(uint8_t i2c_address, uint8_t* command_counter, uint8_t* error_flag) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t response0 = 0;
    // Get counter.
    status = _SI1133_read_register(i2c_address, SI1133_REGISTER_RESPONSE0, &response0);
    if (status != SI1133_SUCCESS) goto errors;
    // Extract command counter and flag.
    (*command_counter) = response0 & 0x0F;
    (*error_flag) = response0 & 0x10;
    // Reset counter when overflow.
    if ((*command_counter) >= 0x0F) {
        status = _SI1133_send_command(i2c_address, SI1133_COMMAND_RESET_CMD_CTR);
        if (status != SI1133_SUCCESS) goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_wait_for_command_completion(uint8_t i2c_address, uint8_t previous_counter, SI1133_status_t error_base) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t response0 = 0;
    uint8_t current_counter = 0;
    uint8_t error_flag = 0;
    uint32_t loop_count_ms = 0;
    // Wait for error or command counter change.
    do {
        // Read status register.
        status = _SI1133_get_status(i2c_address, &current_counter, &error_flag);
        if (status != SI1133_SUCCESS) goto errors;
        // Check flag.
        if (error_flag != 0) {
            status = error_base + (response0 & 0x0F);
            goto errors;
        }
        // Low power delay.
        status = SI1133_HW_delay_milliseconds(SI1133_SUB_DELAY_MS);
        if (status != SI1133_SUCCESS) goto errors;
        // Exit if timeout.
        loop_count_ms += SI1133_SUB_DELAY_MS;
        if (loop_count_ms > SI1133_TIMEOUT_MS) {
            status = SI1133_ERROR_COMMAND_COUNTER;
            goto errors;
        }
    }
    while ((error_flag == 0) && (current_counter == previous_counter));
errors:
    return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_send_command(uint8_t i2c_address, SI1133_commmand_t command) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t previous_counter = 0;
    uint8_t error_flag = 0;
    // Get current value of counter in RESPONSE0 register.
    if (command != SI1133_COMMAND_RESET_CMD_CTR) {
        status = _SI1133_get_status(i2c_address, &previous_counter, &error_flag);
        if (status != SI1133_SUCCESS) goto errors;
    }
    // Send command.
    status = _SI1133_write_register(i2c_address, SI1133_REGISTER_COMMAND, &command, 1);
    if (status != SI1133_SUCCESS) goto errors;
    // Wait for completion.
    if (command != SI1133_COMMAND_RESET_CMD_CTR) {
        status = _SI1133_wait_for_command_completion(i2c_address, previous_counter, SI1133_ERROR_COMMAND_COMPLETION);
        if (status != SI1133_SUCCESS) goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_set_parameter(uint8_t i2c_address, SI1133_parameter_t parameter, uint8_t value) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t parameter_write_command[2];
    uint8_t previous_counter = 0;
    uint8_t error_flag = 0;
    // Build command.
    parameter_write_command[0] = value;
    parameter_write_command[1] = 0x80 + (parameter & 0x3F);
    // Get current value of counter in RESPONSE0 register.
    status = _SI1133_get_status(i2c_address, &previous_counter, &error_flag);
    if (status != SI1133_SUCCESS) goto errors;
    // Send command.
    status = _SI1133_write_register(i2c_address, SI1133_REGISTER_HOSTIN0, parameter_write_command, 2);
    if (status != SI1133_SUCCESS) goto errors;
    // Wait for completion.
    status = _SI1133_wait_for_command_completion(i2c_address, previous_counter, SI1133_ERROR_PARAMETER_COMPLETION);
    if (status != SI1133_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_configure(uint8_t i2c_address) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t irq0_enable = 0x01;
    // Configure channel 0 to compute UV index.
    status = _SI1133_set_parameter(i2c_address, SI1133_PARAMETER_CH_LIST, 0x01); // Enable channel 0.
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_set_parameter(i2c_address, SI1133_PARAMETER_ADCCONFIG0, 0x18); // ADCMUX='11000' (UV index).
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_set_parameter(i2c_address, SI1133_PARAMETER_ADCSENS0, 0x71);
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_set_parameter(i2c_address, SI1133_PARAMETER_ADCPOST0, 0x00); // 16-bits results.
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_write_register(i2c_address, SI1133_REGISTER_IRQ_ENABLE, &irq0_enable, 1);
    if (status != SI1133_SUCCESS) goto errors;
errors:
    return status;
}

/*** SI1133 functions ***/

/*******************************************************************/
SI1133_status_t SI1133_init(void) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    // Init hardware interface.
    status = SI1133_HW_init();
    if (status != SI1133_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SI1133_status_t SI1133_de_init(void) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    // Release hardware interface.
    status = SI1133_HW_de_init();
    if (status != SI1133_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SI1133_status_t SI1133_get_uv_index(uint8_t i2c_address, int32_t* uv_index) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t response0 = 0;
    int32_t raw_uv = 0;
    // Check parameter.
    if (uv_index == NULL) {
        status = SI1133_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Wait for sensor to be ready.
    status = _SI1133_wait_flag(i2c_address, SI1133_REGISTER_RESPONSE0, 5, SI1133_ERROR_READY);
    if (status != SI1133_SUCCESS) goto errors;
    // Configure sensor.
    status = _SI1133_configure(i2c_address);
    if (status != SI1133_SUCCESS) goto errors;
    // Start conversion.
    status = _SI1133_send_command(i2c_address, SI1133_COMMAND_FORCE_CH);
    if (status != SI1133_SUCCESS) goto errors;
    // Wait for conversion to complete (IRQ0='1').
    status = _SI1133_wait_flag(i2c_address, SI1133_REGISTER_IRQ_STATUS, 0, SI1133_ERROR_TIMEOUT);
    if (status != SI1133_SUCCESS) goto errors;
    // Get result.
    status = _SI1133_read_register(i2c_address, SI1133_REGISTER_HOSTOUT0, &response0);
    if (status != SI1133_SUCCESS) goto errors;
    raw_uv |= (int32_t) (response0 << 8);
    status = _SI1133_read_register(i2c_address, SI1133_REGISTER_HOSTOUT1, &response0);
    if (status != SI1133_SUCCESS) goto errors;
    // Convert to UV index.
    // UV index = k * ((m * raw^2) + raw) where k = 0.008284 = 1 / 121 and m = -0.000231 = -1 / 4329.
    raw_uv |= (int32_t) response0;
    (*uv_index) = ((((-1) * raw_uv * raw_uv) / 4329) + raw_uv) / (121);
errors:
    return status;
}

#endif /* SI1133_DRIVER_DISABLE */
