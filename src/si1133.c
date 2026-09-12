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
#include "maths.h"
#include "types.h"

#ifndef SI1133_DRIVER_DISABLE

/*** SI1133 local macros ***/

#define SI1133_BURST_WRITE_MAX_SIZE                 10

#define SI1133_RESET_DELAY_MS                       50
#define SI1133_SUB_DELAY_MS                         10
#define SI1133_TIMEOUT_MS                           2000

#define SI1133_DATA_SIZE_BYTES                      3

#define SI1133_LIGHT_HIGH_COEFFICIENTS_LIST_SIZE    4
#define SI1133_LIGHT_HIGH_INPUT_FRACTION            7

#define SI1133_LIGHT_LOW_COEFFICIENTS_LIST_SIZE     9
#define SI1133_LIGHT_LOW_INPUT_FRACTION             15

#define SI1133_LIGHT_OUTPUT_FRACTION                12

#define SI1133_UV_INDEX_COEFFICIENTS_LIST_SIZE      2
#define SI1133_UV_INDEX_INPUT_FRACTION              15
#define SI1133_UV_INDEX_OUTPUT_FRACTION             12

#define SI1133_X_ORDER_MASK                         0x0070
#define SI1133_Y_ORDER_MASK                         0x0007
#define SI1133_SIGN_MASK                            0x0080
#define SI1133_GET_X_ORDER(m)                       ((m & SI1133_X_ORDER_MASK) >> 4)
#define SI1133_GET_Y_ORDER(m)                       ((m & SI1133_Y_ORDER_MASK))
#define SI1133_GET_SIGN(m)                          ((m & SI1133_SIGN_MASK) >> 7)

#define SI1133_HIGH_AMPLITUDE_THRESHOLD             16000

#define SI1133_SATURATION_VALUE_24BITS              0x7FFFFF
#define SI1133_SATURATION_VALUE_MLUX                128000000

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

/*******************************************************************/
typedef struct {
    int16_t info;
    uint16_t magnitude;
} SI1133_coefficient_t;

/*******************************************************************/
typedef struct {
    SI1133_coefficient_t coefficients_high[SI1133_LIGHT_HIGH_COEFFICIENTS_LIST_SIZE];
    SI1133_coefficient_t coefficients_low[SI1133_LIGHT_LOW_COEFFICIENTS_LIST_SIZE];
} SI1133_light_coefficients;

/*******************************************************************/
typedef struct {
    int32_t x;
    int32_t y;
    uint8_t input_fraction;
    uint8_t output_fraction;
    const SI1133_coefficient_t* coefficients_list;
    uint8_t coefficients_list_size;
} SI1133_polynomial_input_t;

/*** SI1133 local global variables ***/

static const SI1133_light_coefficients SI1133_LIGHT_COEFFICIENTS = {
    { { 0, 209 }, { 1665, 93 }, { 2064, 65 }, { -2671, 234 } },
    { { 0, 0 }, { 1921, 29053 }, { -1022, 36363 }, { 2320, 20789 }, { -367, 57909 }, { -1774, 38240 }, { -608, 46775 }, { -1503, 51831 }, { -1886, 58928 } }
};

static const SI1133_coefficient_t SI1133_UV_INDEX_COEFFICIENTS[SI1133_UV_INDEX_COEFFICIENTS_LIST_SIZE] = {
    { 1281, 30902 }, { -638, 46301 }
};

/*** SI1133 local functions ***/

/*******************************************************************/
static SI1133_status_t _SI1133_write_register(uint8_t i2c_address, SI1133_register_t reg_addr, uint8_t* data, uint8_t data_size_bytes) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t register_write_command[SI1133_BURST_WRITE_MAX_SIZE];
    uint8_t tx_buffer_size = (data_size_bytes + 1);
    uint8_t idx = 0;
    // Check parameters.
    if (reg_addr >= SI1133_REGISTER_LAST) {
        status = SI1133_ERROR_REGISTER;
        goto errors;
    }
    if (tx_buffer_size >= SI1133_BURST_WRITE_MAX_SIZE) {
        status = SI1133_ERROR_WRITE_BUFFER_SIZE;
        goto errors;
    }
    // Build TX buffer.
    register_write_command[0] = reg_addr;
    for (idx = 1; idx < tx_buffer_size; idx++) {
        register_write_command[idx] = data[idx - 1];
    }
    // I2C transfer.
    status = SI1133_HW_i2c_write(i2c_address, register_write_command, tx_buffer_size, 1);
    if (status != SI1133_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_read_register(uint8_t i2c_address, SI1133_register_t reg_addr, uint8_t* data, uint8_t data_size_bytes) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t local_addr = reg_addr;
    // Check parameters.
    if (reg_addr >= SI1133_REGISTER_LAST) {
        status = SI1133_ERROR_REGISTER;
        goto errors;
    }
    // I2C transfer.
    status = SI1133_HW_i2c_write(i2c_address, &local_addr, 1, 1);
    if (status != SI1133_SUCCESS) goto errors;
    status = SI1133_HW_i2c_read(i2c_address, data, data_size_bytes);
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
    // Wait for flag to be set.
    do {
        status = _SI1133_read_register(i2c_address, register_address, &reg_value, 1);
        if (status != SI1133_SUCCESS) goto errors;
        // Delay between reading.
        status = SI1133_HW_delay_milliseconds(SI1133_SUB_DELAY_MS);
        if (status != SI1133_SUCCESS) goto errors;
        // Exit if timeout.
        loop_count_ms += SI1133_SUB_DELAY_MS;
        if (loop_count_ms > SI1133_TIMEOUT_MS) {
            status = timeout_error;
            goto errors;
        }
    }
    while ((reg_value & (0b1 << bit_index)) == 0);
errors:
    return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_send_command(uint8_t i2c_address, SI1133_commmand_t command) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t local_command = command;
    // Send command.
    status = _SI1133_write_register(i2c_address, SI1133_REGISTER_COMMAND, &local_command, 1);
    if (status != SI1133_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_get_status(uint8_t i2c_address, uint8_t* command_counter, uint8_t* error_flag) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t response0 = 0;
    // Get counter.
    status = _SI1133_read_register(i2c_address, SI1133_REGISTER_RESPONSE0, &response0, 1);
    if (status != SI1133_SUCCESS) goto errors;
    // Extract command counter and flag.
    (*command_counter) = (response0 & 0x0F);
    (*error_flag) = (response0 & 0x10);
    // Reset counter when overflow.
    if ((*command_counter) >= 0x0F) {
        // Reset counter.
        status = _SI1133_send_command(i2c_address, SI1133_COMMAND_RESET_CMD_CTR);
        if (status != SI1133_SUCCESS) goto errors;
        // Update output.
        (*command_counter) = 0;
    }
errors:
    return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_wait_for_command_completion(uint8_t i2c_address, uint8_t previous_counter, SI1133_status_t error_base) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
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
            // Ignore saturation error.
            status = ((current_counter == SI1133_COMMAND_ERROR_SATURATION) ? SI1133_SUCCESS : (error_base + current_counter));
            goto errors;
        }
        // Delay between reading.
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
static SI1133_status_t _SI1133_send_command_with_completion(uint8_t i2c_address, SI1133_commmand_t command) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t previous_counter = 0;
    uint8_t error_flag = 0;
    // Wait for the chip to be ready.
    status = _SI1133_wait_flag(i2c_address, SI1133_REGISTER_RESPONSE0, 5, SI1133_ERROR_READY);
    if (status != SI1133_SUCCESS) goto errors;
    // Get current value of counter in RESPONSE0 register.
    status = _SI1133_get_status(i2c_address, &previous_counter, &error_flag);
    if (status != SI1133_SUCCESS) goto errors;
    // Send command.
    status = _SI1133_write_register(i2c_address, SI1133_REGISTER_COMMAND, &command, 1);
    if (status != SI1133_SUCCESS) goto errors;
    // Wait for completion.
    status = _SI1133_wait_for_command_completion(i2c_address, previous_counter, SI1133_ERROR_COMMAND_COMPLETION);
    if (status != SI1133_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
static SI1133_status_t _SI1133_set_parameter_with_completion(uint8_t i2c_address, SI1133_parameter_t parameter, uint8_t value) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t parameter_write_command[2];
    uint8_t previous_counter = 0;
    uint8_t error_flag = 0;
    // Build command.
    parameter_write_command[0] = value;
    parameter_write_command[1] = 0x80 + (parameter & 0x3F);
    // Wait for the chip to be ready.
    status = _SI1133_wait_flag(i2c_address, SI1133_REGISTER_RESPONSE0, 5, SI1133_ERROR_READY);
    if (status != SI1133_SUCCESS) goto errors;
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
static SI1133_status_t _SI1133_read_data(uint8_t i2c_address, SI1133_register_t reg_base, int32_t* data) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint32_t tmp_u32 = 0;
    uint8_t sample_buffer[SI1133_DATA_SIZE_BYTES] = { 0, 0, 0 };
    // Read data.
    status = _SI1133_read_register(i2c_address, reg_base, sample_buffer, SI1133_DATA_SIZE_BYTES);
    if (status != SI1133_SUCCESS) goto errors;
    // Compute data.
    tmp_u32 = (uint32_t) ((sample_buffer[0] << 16) | (sample_buffer[1] << 8) | (sample_buffer[2] << 0));
    // Sign extension.
    (*data) = (((tmp_u32 & 0x00800000) != 0) ? ((int32_t) (tmp_u32 | 0xFF000000)) : ((int32_t) tmp_u32));
errors:
    return status;
}

/*******************************************************************/
static int64_t _SI1133_compute_inner_polynomial(int32_t input, int8_t fraction, uint16_t magnitude, int8_t shift) {
    // Local variables.
    int64_t value = 0;
    int64_t scaled_input = 0;
    uint8_t s = 0;
    // Check magnitude.
    if (magnitude == 0) goto errors;
    // Compute value.
    scaled_input = ((int64_t) input << fraction);
    value = (scaled_input / (int64_t) magnitude);
    // Check shift direction.
    if (shift < 0) {
        s = ((uint8_t) (-shift));
        value >>= s;
    }
    else {
        s = ((uint8_t) shift);
        value <<= s;
    }
errors:
    return value;
}

/*******************************************************************/
static int32_t _SI1133_compute_evaluation_polynomial(SI1133_polynomial_input_t* input) {
    // Local variables.
    uint8_t info = 0;
    uint8_t x_order = 0;
    uint8_t y_order = 0;
    uint8_t counter = 0;
    int8_t sign = 0;
    int8_t shift = 0;
    uint16_t magnitude = 0;
    int64_t term = 0;
    int64_t output = 0;
    int64_t x1 = 0;
    int64_t x2 = 0;
    int64_t y1 = 0;
    int64_t y2 = 0;
    const SI1133_coefficient_t* coefficient_ptr = (input->coefficients_list);
    // Coefficients loop.
    for (counter = 0; counter < (input->coefficients_list_size); counter++) {
        // Store coefficient into local variables.
        info = (uint8_t) (coefficient_ptr->info);
        magnitude = (coefficient_ptr->magnitude);
        x_order = SI1133_GET_X_ORDER(info);
        y_order = SI1133_GET_Y_ORDER(info);
        shift = (int8_t) (((uint16_t) (coefficient_ptr->info) & 0xFF00) >> 8);
        shift = (int8_t) (~shift + 1);
        shift = (int8_t) (-shift);
        sign = ((SI1133_GET_SIGN(info) != 0) ? -1 : 1);
        if ((x_order == 0) && (y_order == 0)) {
            output += (((int64_t) sign) * (((int64_t) magnitude) << (input->output_fraction)));
        }
        else {
            if (x_order > 0) {
                x1 = _SI1133_compute_inner_polynomial((input->x), (int8_t) (input->input_fraction), magnitude, shift);
                x2 = (x_order > 1) ? _SI1133_compute_inner_polynomial((input->x), (int8_t) (input->input_fraction), magnitude, shift) : 1;
            }
            else {
                x1 = 1;
                x2 = 1;
            }
            if (y_order > 0) {
                y1 = _SI1133_compute_inner_polynomial((input->y), (int8_t) (input->input_fraction), magnitude, shift);
                y2 = (y_order > 1) ? _SI1133_compute_inner_polynomial((input->y), (int8_t) (input->input_fraction), magnitude, shift) : 1;
            }
            else {
                y1 = 1;
                y2 = 1;
            }
            term = (((int64_t) sign) * x1 * x2 * y1 * y2);
            output += term;
        }
        coefficient_ptr++;
    }
    // Keep absolute value.
    if (output < 0) {
        output = (-output);
    }
    // Clamp output.
    if (output > MATH_S32_MAX) {
        output = MATH_S32_MAX;
    }
    return ((int32_t) output);
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
SI1133_status_t SI1133_get_light_uv_index(uint8_t i2c_address, int32_t* light_mlux, int32_t* uv_index_duvi, SI1133_light_status_t* light_status) {
    // Local variables.
    SI1133_status_t status = SI1133_SUCCESS;
    uint8_t channel_mask = 0x0F;
    int32_t uv = 0;
    int32_t large_white_low = 0;
    int32_t medium_ir = 0;
    int32_t large_white_high = 0;
    SI1133_polynomial_input_t polynomial_input;
    int32_t tmp_s32 = 0;
    int64_t tmp_s64 = 0;
    // Check parameter.
    if ((light_mlux == NULL) || (uv_index_duvi == NULL) || (light_status == NULL)) {
        status = SI1133_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Reset status.
    (*light_status) = SI1133_LIGHT_STATUS_SENSOR_ERROR;
    // Reset chip.
    status = _SI1133_send_command(i2c_address, SI1133_COMMAND_RESET);
    if (status != SI1133_SUCCESS) goto errors;
    status = SI1133_HW_delay_milliseconds(SI1133_RESET_DELAY_MS);
    if (status != SI1133_SUCCESS) goto errors;
    // Disable burst mode.
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_BURST, 0x01);
    if (status != SI1133_SUCCESS) goto errors;
    // Enable channel 0 to 3.
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_CH_LIST, channel_mask);
    if (status != SI1133_SUCCESS) goto errors;
    // Configure channel 0 for UV index.
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_ADCCONFIG0, 0x78);
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_ADCSENS0, 0x71);
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_ADCPOST0, 0x40);
    if (status != SI1133_SUCCESS) goto errors;
    // Configure channel 1 for large white.
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_ADCCONFIG1, 0x4D);
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_ADCSENS1, 0xE1);
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_ADCPOST1, 0x40);
    if (status != SI1133_SUCCESS) goto errors;
    // Configure channel 2 for medium IR.
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_ADCCONFIG2, 0x41);
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_ADCSENS2, 0xE1);
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_ADCPOST2, 0x50);
    if (status != SI1133_SUCCESS) goto errors;
    // Configure channel 3 for large white.
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_ADCCONFIG3, 0x4D);
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_ADCSENS3, 0x87);
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_set_parameter_with_completion(i2c_address, SI1133_PARAMETER_ADCPOST3, 0x40);
    if (status != SI1133_SUCCESS) goto errors;
    // Enable interrupts.
    status = _SI1133_write_register(i2c_address, SI1133_REGISTER_IRQ_ENABLE, &channel_mask, 1);
    if (status != SI1133_SUCCESS) goto errors;
    // Start conversion.
    status = _SI1133_send_command_with_completion(i2c_address, SI1133_COMMAND_FORCE_CH);
    if (status != SI1133_SUCCESS) goto errors;
    // Wait for conversion to complete (IRQ0='1').
    status = _SI1133_wait_flag(i2c_address, SI1133_REGISTER_IRQ_STATUS, 0, SI1133_ERROR_TIMEOUT);
    if (status != SI1133_SUCCESS) goto errors;
    // Read raw data.
    status = _SI1133_read_data(i2c_address, SI1133_REGISTER_HOSTOUT0, &uv);
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_read_data(i2c_address, SI1133_REGISTER_HOSTOUT3, &large_white_high);
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_read_data(i2c_address, SI1133_REGISTER_HOSTOUT6, &medium_ir);
    if (status != SI1133_SUCCESS) goto errors;
    status = _SI1133_read_data(i2c_address, SI1133_REGISTER_HOSTOUT9, &large_white_low);
    if (status != SI1133_SUCCESS) goto errors;
    // Compute ambient light.
    polynomial_input.y = medium_ir;
    polynomial_input.output_fraction = SI1133_LIGHT_OUTPUT_FRACTION;
    // Check dynamic.
    if ((large_white_high >= SI1133_SATURATION_VALUE_24BITS) || (medium_ir >= SI1133_SATURATION_VALUE_24BITS)) {
        // Update status.
        (*light_mlux) = SI1133_SATURATION_VALUE_MLUX;
        (*light_status) = SI1133_LIGHT_STATUS_SENSOR_SATURATION;
    }
    else {
        // Check dynamic.
        if ((large_white_high > SI1133_HIGH_AMPLITUDE_THRESHOLD) || (medium_ir > SI1133_HIGH_AMPLITUDE_THRESHOLD)) {
            // High amplitude parameters.
            polynomial_input.x = large_white_high;
            polynomial_input.input_fraction = SI1133_LIGHT_HIGH_INPUT_FRACTION;
            polynomial_input.coefficients_list = &(SI1133_LIGHT_COEFFICIENTS.coefficients_high[0]);
            polynomial_input.coefficients_list_size = SI1133_LIGHT_HIGH_COEFFICIENTS_LIST_SIZE;
        }
        else {
            // Low amplitude parameters.
            polynomial_input.x = large_white_low;
            polynomial_input.input_fraction = SI1133_LIGHT_LOW_INPUT_FRACTION;
            polynomial_input.coefficients_list = &(SI1133_LIGHT_COEFFICIENTS.coefficients_low[0]);
            polynomial_input.coefficients_list_size = SI1133_LIGHT_LOW_COEFFICIENTS_LIST_SIZE;
        }
        // Compute lux.
        tmp_s32 = _SI1133_compute_evaluation_polynomial(&polynomial_input);
        tmp_s64 = ((((int64_t) tmp_s32 * (int64_t) 1000) + (int64_t) (1 << (SI1133_LIGHT_OUTPUT_FRACTION - 1))) >> SI1133_LIGHT_OUTPUT_FRACTION);
        (*light_mlux) = ((int32_t) tmp_s64);
        // Update status.
        (*light_status) = SI1133_LIGHT_STATUS_AVAILABLE;
    }
    // Compute UV index.
    polynomial_input.x = 0;
    polynomial_input.y = uv;
    polynomial_input.input_fraction = SI1133_UV_INDEX_INPUT_FRACTION;
    polynomial_input.output_fraction = SI1133_UV_INDEX_OUTPUT_FRACTION;
    polynomial_input.coefficients_list = &(SI1133_UV_INDEX_COEFFICIENTS[0]);
    polynomial_input.coefficients_list_size = SI1133_UV_INDEX_COEFFICIENTS_LIST_SIZE;
    tmp_s32 = _SI1133_compute_evaluation_polynomial(&polynomial_input);
    tmp_s64 = ((((int64_t) tmp_s32 * (int64_t) 10) + (int64_t) (1 << (SI1133_UV_INDEX_OUTPUT_FRACTION - 1))) >> SI1133_UV_INDEX_OUTPUT_FRACTION);
    (*uv_index_duvi) = ((int32_t) tmp_s64);
errors:
    return status;
}

#endif /* SI1133_DRIVER_DISABLE */
