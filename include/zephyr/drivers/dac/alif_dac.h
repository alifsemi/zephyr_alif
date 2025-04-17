/* Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DAC_ALIF_DAC_H_
#define ZEPHYR_DRIVERS_DAC_ALIF_DAC_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif


/* DAC Control Code */
#define DAC_RESET			(0x01UL)
#define DAC_INPUT_BYPASS_MODE		(0x02UL)
#define DAC_CAPACITANCE_HP_MODE		(0x03UL)
#define DAC_SELECT_IBIAS_OUTPUT		(0x04UL)

/* Select the DAC output current */
#define DAC_0UA_OUT_CUR			(0x0UL)
#define DAC_100UA_OUT_CUR		(0x1UL)
#define DAC_200UA_OUT_CUR		(0x2UL)
#define DAC_300UA_OUT_CUR		(0x3UL)
#define DAC_400UA_OUT_CUR		(0x4UL)
#define DAC_500UA_OUT_CUR		(0x5UL)
#define DAC_600UA_OUT_CUR		(0x6UL)
#define DAC_700UA_OUT_CUR		(0x7UL)
#define DAC_800UA_OUT_CUR		(0x8UL)
#define DAC_900UA_OUT_CUR		(0x9UL)
#define DAC_1000UA_OUT_CUR		(0xAUL)
#define DAC_1100UA_OUT_CUR		(0xBUL)
#define DAC_1200UA_OUT_CUR		(0xCUL)
#define DAC_1300UA_OUT_CUR		(0xDUL)
#define DAC_1400UA_OUT_CUR		(0xEUL)
#define DAC_1500UA_OUT_CUR		(0xFUL)

/* Select the DAC capacitance */
#define DAC_2PF_CAPACITANCE		(0x0UL)
#define DAC_4PF_CAPACITANCE		(0x1UL)
#define DAC_6PF_CAPACITANCE		(0x2UL)
#define DAC_8PF_CAPACITANCE		(0x3UL)


void dac_set_capacitance(const struct device *dev, uint8_t cap_val);

void dac_set_bypass_input(const struct device *dev, uint32_t bypass_val);

void dac_set_output_current(const struct device *dev, uint8_t cur_val);

void dac_reset_control(const struct device *dev, bool enable);

#ifdef __cplusplus
}
#endif

#endif  /* ZEPHYR_DRIVERS_DAC_ALIF_DAC_H_ */

