/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"


#define I2C_TARGET_ADDRESS (0x40)
#define SI7021_ADDR      (0x40)   // I2C address of the Si7021 
#define TEMP_MEASURE_CMD (0xE3)   // Command to measure temperature
#define HUMI_MEASURE_CMD (0xE5)   // Command to measure humidity


uint8_t recv_buf[10];
uint8_t send_buf[1] = {0xE3};

uint16_t temp_raw, humi_raw;
float temperature, humidity;

 
 uint16_t si7021_read(uint8_t command)
{
    send_buf[0] = command;
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &send_buf[0], 1);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, I2C_TARGET_ADDRESS,
            DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) &DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    delay_cycles(1000);
    DL_I2C_startControllerTransfer(I2C_0_INST, I2C_TARGET_ADDRESS,
            DL_I2C_CONTROLLER_DIRECTION_RX, 2);
    for (uint8_t i = 0; i < 2; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST));
        recv_buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
    return (recv_buf[0]<<8) + recv_buf[1];
}


int main(void)
{
    SYSCFG_DL_init();

    /* Set LED to indicate start of transfer */
    DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

    while (1) {
        temp_raw = si7021_read(TEMP_MEASURE_CMD);
        humi_raw = si7021_read(HUMI_MEASURE_CMD);
        temperature = ((175.72 * temp_raw) / 65536.0) - 46.85;
        humidity = ((125.0 * humi_raw) / 65536.0) - 6.0;
        
        delay_cycles(3200000);
        
        if (temperature < 25)
            DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN | GPIO_LEDS_USER_TEST_PIN);
        else
            DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

    }
}

