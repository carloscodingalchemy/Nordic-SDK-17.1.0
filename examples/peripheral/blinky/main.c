/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "nrf_timer.h"
#include "boards.h"

/**
 * @brief
 *
 */
void nrfx_timer_0_irq_handler(void)
{
    nrf_gpio_pin_toggle(LED_USER);
    nrf_timer_event_clear(NRF_TIMER0, nrf_timer_compare_event_get(NRF_TIMER_CC_CHANNEL0));
    nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_CLEAR);  
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{

    /* Configure USER LED as an output*/
    nrf_gpio_cfg_output(LED_USER);
    nrf_gpio_pin_write(LED_USER, 0);

    /* Configure the Timer 0 */
    nrf_timer_mode_set(NRF_TIMER0, NRF_TIMER_MODE_TIMER);
    nrf_timer_event_clear(NRF_TIMER0, nrf_timer_compare_event_get(NRF_TIMER_CC_CHANNEL0));
    nrf_timer_frequency_set(NRF_TIMER0, NRF_TIMER_FREQ_1MHz);
    nrf_timer_bit_width_set(NRF_TIMER0, NRF_TIMER_BIT_WIDTH_24);
    nrf_timer_cc_write(NRF_TIMER0, NRF_TIMER_CC_CHANNEL0, 0X7A120);

    /* */
    NVIC_ClearPendingIRQ(TIMER0_IRQn);
    NVIC_SetPriority(TIMER0_IRQn, 2);
    NVIC_EnableIRQ(TIMER0_IRQn);

    nrf_timer_int_enable(NRF_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);

    nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_START);   

    /* Loop waiting for interrupts from the timer 0 */
    while (true)
    {
        __WFI();
    }
}