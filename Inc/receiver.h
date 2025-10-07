/*
 * hx1838.h
 *
 *  Created on: Aug 22, 2025
 *      Author: UTA1HYD
 */

#ifndef HX1838_H
#define HX1838_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* NEC timings (all in microseconds)
   We measure time BETWEEN FALLING EDGES = mark + space.
   Bit0  ~ 560 + 560   = 1120
   Bit1  ~ 560 + 1690  = 2250
   Header ~ 9000 + 4500 = 13500
   Repeat ~ 9000 + 2250 = 11250
*/
#define NEC_TOL_PCT           25      // +/- percentage tolerance
#define NEC_HEADER_US         13500u
#define NEC_REPEAT_US         11250u
#define NEC_BIT0_US           1120u
#define NEC_BIT1_US           2250u
#define NEC_BITS              32

#ifdef __cplusplus
extern "C" {
#endif

// Initialize with timer handle and IR input pin
void HX1838_Init(TIM_HandleTypeDef *htim, uint16_t ir_pin);

// Blocking receive: waits until code received or timeout (ms)
bool HX1838_GetCode(uint32_t *code, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
