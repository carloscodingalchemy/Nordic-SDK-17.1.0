/**
 * Copyright (c) 2015 - 2021, Nordic Semiconductor ASA
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
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include "boards.h"
#include "nrf_twi.h"
#include "nrf_uart.h"
#include "nrf_delay.h"



/* Common addresses definition for Lluminsity sensor. */
#define TSL2561_ADDR                (0x39U) // Could also be 0x29 or 0x49 depending on how the ADDR_SEL pin is connected.

#define TSL2561_REG_CONTROL         (0x00U)
#define TSL2561_REG_TIMING          (0x01U)
#define TSL2561_REG_THRESHLOWLOW    (0x02U)
#define TSL2561_REG_THRESHLOWHIGH   (0x03U)
#define TSL2561_REG_THRESHHIGHLOW   (0x04U)
#define TSL2561_REG_THRESHHIGHHIGH  (0x05U)
#define TSL2561_REG_INTERRUPT       (0x06U)
#define TSL2561_REG_RESERVED1       (0x07U)
#define TSL2561_REG_CRC             (0x08U)
#define TSL2561_REG_RESERVED2       (0x09U)
#define TSL2561_REG_ID              (0x0AU)
#define TSL2561_REG_RESERVED3       (0x0BU)
#define TSL2561_REG_DATA0LOW        (0x0CU)
#define TSL2561_REG_DATA0HIGH       (0x0DU)
#define TSL2561_REG_DATA1LOW        (0x0EU)
#define TSL2561_REG_DATA1HIGH       (0x0FU)

#define TSL2561_CMD                 (0x80)

void uart_write_byte(uint8_t value);

/**
 * @brief Initialise the TWI peripheral with the given parameters
 * 
 */
void twi_init (void)
{
    nrf_twi_address_set(NRF_TWI0, TSL2561_ADDR);
    nrf_twi_pins_set(NRF_TWI0, ARDUINO_SCL_PIN, ARDUINO_SDA_PIN);
    nrf_twi_frequency_set(NRF_TWI0, NRF_TWI_FREQ_400K);
    nrf_twi_enable(NRF_TWI0);
}

/**
 * @brief 
 * 
 */
void uart_init()
{

    nrf_gpio_pin_set(NRF_UART0->PSEL.TXD);
    nrf_gpio_cfg_output(NRF_UART0->PSEL.TXD);
    nrf_gpio_cfg_input(NRF_UART0->PSEL.RXD, NRF_GPIO_PIN_NOPULL);

    nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_115200);
    /* 1-stop bit is implicitly configured as it is the reset value and this function does not allow to modify the value */
    nrf_uart_configure(NRF_UART0, NRF_UART_PARITY_EXCLUDED, NRF_UART_HWFC_DISABLED); 
    nrf_uart_txrx_pins_set(NRF_UART0, TX_PIN_NUMBER, RX_PIN_NUMBER);
    nrf_uart_enable(NRF_UART0);
}

void uart_write_byte(uint8_t value)
{

    nrf_uart_txd_set(NRF_UART0, value);
    nrf_uart_event_clear(NRF_UART0, NRF_UART_EVENT_TXDRDY); // If you do not clear this event nobody will do it for you.
    nrf_uart_task_trigger(NRF_UART0, NRF_UART_TASK_STARTTX);

    
    while (nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_TXDRDY) == false)
    {
        /* code */
    }
    
    nrf_uart_task_trigger(NRF_UART0, NRF_UART_TASK_STOPTX);

}

void uart_write_string(uint8_t* message, uint32_t len)
{
    for (int i = 0; i < len; i++)
    {
        uart_write_byte(message[i]);
    }
    uart_write_byte('\r');
}

void tsl2561_write_byte(uint8_t address, uint8_t value)
{
    nrf_twi_txd_set(NRF_TWI0, (address & 0x0F) | TSL2561_CMD); // Send command byte
    nrf_twi_event_clear(NRF_TWI0, NRF_TWI_EVENT_TXDSENT);
    nrf_twi_task_trigger(NRF_TWI0, NRF_TWI_TASK_STARTTX);

    /* Wait until TASK_TXSEND event is rised and send a TWI_TASK_STOP */
    while (nrf_twi_event_check(NRF_TWI0, NRF_TWI_EVENT_TXDSENT) == false)
    {

    }

    nrf_twi_txd_set(NRF_TWI0, value); // Send value
    nrf_twi_event_clear(NRF_TWI0, NRF_TWI_EVENT_TXDSENT);

    /* Wait until TASK_TXSEND event is rised and send a TWI_TASK_STOP */
    while (nrf_twi_event_check(NRF_TWI0, NRF_TWI_EVENT_TXDSENT) == false)
    {

    }

    nrf_twi_task_trigger(NRF_TWI0, NRF_TWI_TASK_STOP);
}

/**
 * @brief initialise the tsl2561 by configuring the corresponding registers
 * 
 */
void tsl2561_init(void)
{
    tsl2561_write_byte(TSL2561_REG_CONTROL, 0x03);
}

/**
 * @brief Reads a value from the sensor 
 * 
 */
uint8_t tsl2561_read_byte(uint8_t address)
{

    uint8_t value;
    nrf_twi_event_clear(NRF_TWI0, NRF_TWI_EVENT_TXDSENT);
    nrf_twi_event_clear(NRF_TWI0, NRF_TWI_EVENT_RXDREADY);

    nrf_twi_txd_set(NRF_TWI0, (address & 0x0F) | TSL2561_CMD); // Send command byte

    nrf_twi_task_trigger(NRF_TWI0, NRF_TWI_TASK_STARTTX);

    /* Wait until TASK_TXSEND event is rised and send a TWI_TASK_STOP */
    while (nrf_twi_event_check(NRF_TWI0, NRF_TWI_EVENT_TXDSENT) == false)
    {

    }

    nrf_twi_task_trigger(NRF_TWI0, NRF_TWI_TASK_STARTRX);

    /* Wait until EVENT_RXDREADY event is rised and send a TWI_TASK_STOP */
    while (nrf_twi_event_check(NRF_TWI0, NRF_TWI_EVENT_RXDREADY) == false)
    {
    }

    nrf_twi_task_trigger(NRF_TWI0, NRF_TWI_TASK_STOP);
    
    value = nrf_twi_rxd_get(NRF_TWI0);


    return value;
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{

    uint8_t message[255];
    uint32_t len;

    /* Initialise both UART and I2C peripherals */
    uart_init();
    twi_init();
    tsl2561_init();

    uint8_t data0_low = 0XFF;
    uint8_t data0_high = 0XFF;

    /* Infinite loop when we read data from the sensor and send it through the UART */
    while (true)
    {
        data0_low = tsl2561_read_byte(TSL2561_REG_DATA0LOW);
        data0_high = tsl2561_read_byte(TSL2561_REG_DATA0HIGH);

        len = sprintf((char*)message, "Value read from TSL2561: %08x_%08x\n", data0_high, data0_low);

        uart_write_string(message, len);

        nrf_delay_ms(500);

    }
}

/** @} */
