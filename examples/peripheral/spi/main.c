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


#include <string.h>
#include <stdio.h>

#include "boards.h"
#include "nrf_spi.h"
#include "nrf_uart.h"
#include "nrf_delay.h"


// SPI pins
#define SPI_SCK             0x2F
#define SPI_MOSI            0x2D
#define SPI_MISO            0x2E
#define SPI_CS              0x22


// command definitions
#define CMD0                0
#define CMD0_ARG            0x00000000
#define CMD0_CRC            0x94
#define CMD8                8
#define CMD8_ARG            0x0000001AA
#define CMD8_CRC            0x86
#define CMD58               58
#define CMD58_ARG           0x00000000
#define CMD58_CRC           0x00

// R1 responses
#define PARAM_ERROR(X)      X & 0b01000000
#define ADDR_ERROR(X)       X & 0b00100000
#define ERASE_SEQ_ERROR(X)  X & 0b00010000
#define CRC_ERROR(X)        X & 0b00001000
#define ILLEGAL_CMD(X)      X & 0b00000100
#define ERASE_RESET(X)      X & 0b00000010
#define IN_IDLE(X)          X & 0b00000001

#define POWER_UP_STATUS(X)  X & 0x40
#define CCS_VAL(X)          X & 0x40
#define VDD_2728(X)         X & 0b10000000
#define VDD_2829(X)         X & 0b00000001
#define VDD_2930(X)         X & 0b00000010
#define VDD_3031(X)         X & 0b00000100
#define VDD_3132(X)         X & 0b00001000
#define VDD_3233(X)         X & 0b00010000
#define VDD_3334(X)         X & 0b00100000
#define VDD_3435(X)         X & 0b01000000
#define VDD_3536(X)         X & 0b10000000

// R7 responses
#define CMD_VER(X)          ((X >> 4) & 0xF0)
#define VOL_ACC(X)          (X & 0x1F)
#define VOLTAGE_ACC_27_33   0b00000001
#define VOLTAGE_ACC_LOW     0b00000010
#define VOLTAGE_ACC_RES1    0b00000100
#define VOLTAGE_ACC_RES2    0b00001000

// UART functions
void uart_init();
void uart_write_byte(uint8_t value);
void uart_write_string(char *message);

// SPI functions
void SPI_init(void);
uint8_t SPI_transfer(uint8_t data);

// SD functions
void SD_powerUpSeq(void);
void SD_command(uint8_t cmd, uint32_t arg, uint8_t crc);
uint8_t SD_readRes1(void);
void SD_readRes3_7(uint8_t *res);
uint8_t SD_goIdleState(void);
uint8_t *SD_sendIfCond(uint8_t *res);
uint8_t *SD_readOCR(uint8_t *res);
void SD_printR1(uint8_t res);
void SD_printR3(uint8_t *res);
void SD_printR7(uint8_t *res);

void cs_enable(void)
{
    /* Sequence to disable the Chip Select GPIO pin */
    nrf_gpio_pin_clear(SPI_CS);
    
}

void cs_disable(void)
{
    /* Sequence to enable the Chip Select GPIO pin */
    nrf_gpio_pin_set(SPI_CS);
}

void SPI_init()
{
    nrf_gpio_cfg_output(SPI_SCK);
    nrf_gpio_cfg_output(SPI_MOSI);
    nrf_gpio_cfg_output(SPI_CS);
    cs_disable();
    nrf_gpio_cfg_input(SPI_MISO, NRF_GPIO_PIN_NOPULL);

    nrf_spi_configure(NRF_SPI0, NRF_SPI_MODE_0, NRF_SPI_BIT_ORDER_MSB_FIRST);
    nrf_spi_pins_set(NRF_SPI0, SPI_SCK, SPI_MOSI, SPI_MISO);
    nrf_spi_frequency_set(NRF_SPI0, NRF_SPI_FREQ_1M);

    nrf_spi_enable(NRF_SPI0);


}


uint8_t SPI_transfer(uint8_t data)
{

    nrf_spi_event_clear(NRF_SPI0, NRF_SPI_EVENT_READY); // If you do not clear this event, nobody will do it for you.
    nrf_spi_txd_set(NRF_SPI0, data);
    
    while (nrf_spi_event_check(NRF_SPI0, NRF_SPI_EVENT_READY) == false)
    {
        /* code */
    }

    return nrf_spi_rxd_get(NRF_SPI0);
}

void SD_powerUpSeq()
{
    // make sure card is deselected
    cs_disable();

    // give SD card time to power up
    nrf_delay_ms(1000);

    // select SD card

    cs_enable();

    // send 74 clock cycles to synchronize
    for(uint8_t i = 0; i < 74; i++)
    

    // deselect SD card
    cs_disable();

}

void SD_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    // transmit command to sd card
    SPI_transfer(cmd|0x40);

    // transmit argument
    SPI_transfer((uint8_t)(arg >> 24));
    SPI_transfer((uint8_t)(arg >> 16));
    SPI_transfer((uint8_t)(arg >> 8));
    SPI_transfer((uint8_t)(arg));

    // transmit crc
    SPI_transfer(crc|0x01);
}

uint8_t SD_readRes1()
{
    uint8_t i = 0, res1;

    // keep polling until actual data received
    while((res1 = SPI_transfer(0xFF)) == 0xFF)
    {
        i++;

        // if no data received for 8 bytes, break
        if(i > 8) break;
    }

    return res1;
}

void SD_readRes3_7(uint8_t *res)
{
    // read response 1 in R7
    res[0] = SD_readRes1();

    // if error reading R1, return
    if(res[0] > 1) 
    {
        uart_write_string("Error reading R1\r\n");
        return;
    }

    // read remaining bytes
    res[1] = SPI_transfer(0xFF);
    res[2] = SPI_transfer(0xFF);
    res[3] = SPI_transfer(0xFF);
    res[4] = SPI_transfer(0xFF);
}

uint8_t SD_goIdleState()
{
    // assert chip select

    cs_enable();


    // send CMD0
    SD_command(CMD0, CMD0_ARG, CMD0_CRC);

    // read response
    uint8_t res1 = SD_readRes1();
    SD_printR1(res1);

    // deassert chip select

    cs_disable();


    return res1;
}

uint8_t *SD_sendIfCond(uint8_t *res)
{
    // assert chip select

    cs_enable();


    // send CMD8
    SD_command(CMD8, CMD8_ARG, CMD8_CRC);

    // read response
    SD_readRes3_7(res);

    // deassert chip select

    cs_disable();


    return res;
}

uint8_t *SD_readOCR(uint8_t *res)
{
    // assert chip select

    cs_enable();


    // send CMD58
    SD_command(CMD58, CMD58_ARG, CMD58_CRC);

    // read response
    SD_readRes3_7(res);

    // deassert chip select

    cs_disable();


    return res;
}

void SD_printR1(uint8_t res)
{
    if(res & 0b10000000)
        uart_write_string("\tError: MSB = 1\r\n");
    if(res == 0)
        uart_write_string("\tCard Ready\r\n");
    if(PARAM_ERROR(res))
        uart_write_string("\tParameter Error\r\n");
    if(ADDR_ERROR(res))
        uart_write_string("\tAddress Error\r\n");
    if(ERASE_SEQ_ERROR(res))
        uart_write_string("\tErase Sequence Error\r\n");
    if(CRC_ERROR(res))
        uart_write_string("\tCRC Error\r\n");
    if(ILLEGAL_CMD(res))
        uart_write_string("\tIllegal Command\r\n");
    if(ERASE_RESET(res))
        uart_write_string("\tErase Reset Error\r\n");
    if(IN_IDLE(res))
        uart_write_string("\tIn Idle State\r\n");
}

void SD_printR3(uint8_t *res)
{
    SD_printR1(res[0]);

    if(res[0] > 1) return;

    uart_write_string("\tCard Power Up Status: ");
    if(POWER_UP_STATUS(res[1]))
    {
        uart_write_string("READY\r\n");
        uart_write_string("\tCCS Status: ");
        if(CCS_VAL(res[1])){ uart_write_string("1\r\n"); }
        else uart_write_string("0\r\n");
    }
    else
    {
        uart_write_string("BUSY\r\n");
    }

    uart_write_string("\tVDD Window: ");
    if(VDD_2728(res[3])) uart_write_string("2.7-2.8, ");
    if(VDD_2829(res[2])) uart_write_string("2.8-2.9, ");
    if(VDD_2930(res[2])) uart_write_string("2.9-3.0, ");
    if(VDD_3031(res[2])) uart_write_string("3.0-3.1, ");
    if(VDD_3132(res[2])) uart_write_string("3.1-3.2, ");
    if(VDD_3233(res[2])) uart_write_string("3.2-3.3, ");
    if(VDD_3334(res[2])) uart_write_string("3.3-3.4, ");
    if(VDD_3435(res[2])) uart_write_string("3.4-3.5, ");
    if(VDD_3536(res[2])) uart_write_string("3.5-3.6");
    uart_write_string("\r\n");
}

void SD_printR7(uint8_t *res)
{
    char buffer[255];

    SD_printR1(res[0]);

    if(res[0] > 1) return;

    (void)sprintf(buffer, "\tCommand Version: %08x\r\n", res[1]);

    uart_write_string(buffer);

    uart_write_string("\tVoltage Accepted: ");
    if(VOL_ACC(res[3]) == VOLTAGE_ACC_27_33)
        uart_write_string("2.7-3.6V\r\n");
    else if(VOL_ACC(res[3]) == VOLTAGE_ACC_LOW)
        uart_write_string("LOW VOLTAGE\r\n");
    else if(VOL_ACC(res[3]) == VOLTAGE_ACC_RES1)
        uart_write_string("RESERVED\r\n");
    else if(VOL_ACC(res[3]) == VOLTAGE_ACC_RES2)
        uart_write_string("RESERVED\r\n");
    else uart_write_string("NOT DEFINED\r\n");

    (void)sprintf(buffer, "\tEcho: %08x\r\n", res[4]);

    uart_write_string(buffer);
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

void uart_write_string(char* message)
{
    for (int i = 0; i < strlen(message); i++)
    {
        uart_write_byte(message[i]);
    }
}





/**
 * @brief Function for main application entry.
 */
int main(void)
{

    uint8_t response[5];

    /* Initialise both UART and SPI peripherals */
    uart_init();
    SPI_init();
    // SDCard_init();
    SD_goIdleState();

    uart_write_string("Sending CMD58...\r\n");
    SD_readOCR(response);
    SD_printR3(response);

    nrf_delay_ms(500);
}

/** @} */


