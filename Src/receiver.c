/*
 * hx1838.c
 *
 *  Created on: Aug 22, 2025
 *      Author: UTA1HYD
 */

#include <receiver.h>

typedef enum {
    IR_IDLE = 0,
    IR_RECEIVING
} ir_state_t;

static TIM_HandleTypeDef *ir_htim = NULL;
static volatile uint32_t last_fall = 0;

static volatile ir_state_t state = IR_IDLE;
static volatile uint32_t ir_code = 0;
static volatile uint8_t  bit_count = 0;
static uint16_t ir_pin_g = GPIO_PIN_5; // default, set by Init()

// --- Simple ring buffer for received codes (instead of osMessageQueue) ---
#define IR_BUF_SIZE   5
static volatile uint32_t ir_buf[IR_BUF_SIZE];
static volatile uint8_t  ir_head = 0;
static volatile uint8_t  ir_tail = 0;

// ========== Helpers ==========
static inline uint32_t us_diff(uint32_t now, uint32_t prev) {
    return (now >= prev) ? (now - prev) : (0xFFFFFFFFu - prev + now);
}
static inline uint8_t in_range(uint32_t val, uint32_t target) {
    uint32_t tol = (target * NEC_TOL_PCT) / 100u;
    return (val >= (target - tol)) && (val <= (target + tol));
}
static inline void buf_push(uint32_t code) {
    uint8_t next = (ir_head + 1) % IR_BUF_SIZE;
    if (next != ir_tail) {   // only push if not full
        ir_buf[ir_head] = code;
        ir_head = next;
    }
}
static inline bool buf_pop(uint32_t *code) {
    if (ir_head == ir_tail) return false;  // empty
    *code = ir_buf[ir_tail];
    ir_tail = (ir_tail + 1) % IR_BUF_SIZE;
    return true;
}

// ========== API ==========
void HX1838_Init(TIM_HandleTypeDef *htim, uint16_t ir_pin)
{
    ir_htim = htim;
    ir_pin_g = ir_pin;

    // Start free-running microsecond timer
    HAL_TIM_Base_Start(ir_htim);

    // NOTE:
    // - GPIO EXTI must be configured in CubeMX (falling edge interrupt).
    // - stm32f4xx_it.c must call HAL_GPIO_EXTI_IRQHandler(ir_pin_g).
}

// HAL weak callback resolved here
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != ir_pin_g || ir_htim == NULL) return;

    uint32_t now = __HAL_TIM_GET_COUNTER(ir_htim);
    uint32_t dt  = us_diff(now, last_fall);
    last_fall = now;

    // First edge after boot has garbage dt; ignore once
    static uint8_t primed = 0;
    if (!primed) { primed = 1; return; }

    // Detect header or repeat
    if (in_range(dt, NEC_HEADER_US)) {
        state = IR_RECEIVING;
        bit_count = 0;
        ir_code = 0;
        return;
    }
    if (in_range(dt, NEC_REPEAT_US)) {
        // ignore repeats for now
        return;
    }

    if (state == IR_RECEIVING) {
        if (in_range(dt, NEC_BIT0_US)) {
            ir_code = (ir_code << 1) | 0;
            bit_count++;
        } else if (in_range(dt, NEC_BIT1_US)) {
            ir_code = (ir_code << 1) | 1;
            bit_count++;
        } else {
            // timing error -> reset state
            state = IR_IDLE;
            bit_count = 0;
            ir_code = 0;
            return;
        }

        if (bit_count >= NEC_BITS) {
            buf_push(ir_code);
            state = IR_IDLE;
        }
    }
}

bool HX1838_GetCode(uint32_t *code, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    do {
        if (buf_pop(code)) {
            return true;
        }
    } while ((HAL_GetTick() - start) < timeout_ms);

    return false; // timeout
}
