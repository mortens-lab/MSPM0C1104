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

#define I2C_SI7021_ADDR  (0x40)   // I2C address of the Si7021 
#define TEMP_MEASURE_CMD (0xE3)   // Command to measure temperature
#define HUMI_MEASURE_CMD (0xE5)   // Command to measure humidity

#define I2C_VK16K33_ADDR (0x70)   // I2C address of the VK16K33

uint8_t recv_buf[10];
uint8_t send_buf[1] = {0xE3};
uint8_t send_bux[2] = {0x55,0x00};
uint8_t send_data[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t vk16k33_data[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint16_t num=0;

uint16_t temp_raw, humi_raw;
float temperature, humidity;


uint16_t si7021_read(uint8_t command)
{
	send_buf[0] = command;
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &send_buf[0], 1);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_0_INST, I2C_SI7021_ADDR,
            DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C_0_INST) &DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    delay_cycles(1000);
    DL_I2C_startControllerTransfer(I2C_0_INST, I2C_SI7021_ADDR,
            DL_I2C_CONTROLLER_DIRECTION_RX, 2);
    for (uint8_t i = 0; i < 2; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST));
        recv_buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
    return (recv_buf[0]<<8) + recv_buf[1];
}


void vk16k33_write(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    unsigned char I2Ctxbuff[8] = {0x00};

    I2Ctxbuff[0] = reg_addr;
    unsigned char i, j = 1;

    for(i=0; i<count; i++)
    {
        I2Ctxbuff[j] = reg_data[i];
        j++;
    }

    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &I2Ctxbuff[0], count+1);

    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_startControllerTransfer(I2C_0_INST, I2C_VK16K33_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, count+1);

    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    /* Wait for I2C to be Idle */
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

	DL_I2C_flushControllerTXFIFO(I2C_0_INST);
}


void calc_digit_reset(void)
{
	uint8_t i=0;
	for (i=0; i<16; i++)
		vk16k33_data[i] = 0;
}


void calc_digit_fill(uint8_t value, uint8_t pos)
{
	uint8_t p=0;

    if (pos==0) p=8;
    if (pos==1) p=4;
    if (pos==2) p=2;
    if (pos==3) p=1;
    
    if (value==0)
    {
        vk16k33_data[0x00] = vk16k33_data[0x00] | p;
        vk16k33_data[0x02] = vk16k33_data[0x02] | p;
        vk16k33_data[0x04] = vk16k33_data[0x04] | p;
        vk16k33_data[0x06] = vk16k33_data[0x06] | p;
        vk16k33_data[0x08] = vk16k33_data[0x08] | p;
        vk16k33_data[0x0A] = vk16k33_data[0x0A] | p;
    }   
    if (value==1)
    {
        vk16k33_data[0x02] = vk16k33_data[0x02] | p;
        vk16k33_data[0x04] = vk16k33_data[0x04] | p;
    }   
    if (value==2)
    {
        vk16k33_data[0x00] = vk16k33_data[0x00] | p;
        vk16k33_data[0x02] = vk16k33_data[0x02] | p;
        vk16k33_data[0x06] = vk16k33_data[0x06] | p;
        vk16k33_data[0x08] = vk16k33_data[0x08] | p;
        vk16k33_data[0x0C] = vk16k33_data[0x0C] | p;
        vk16k33_data[0x00] = vk16k33_data[0x00] | (p<<4);
    }   
    if (value==3)
    {
        vk16k33_data[0x00] = vk16k33_data[0x00] | p;
        vk16k33_data[0x02] = vk16k33_data[0x02] | p;
        vk16k33_data[0x04] = vk16k33_data[0x04] | p;
        vk16k33_data[0x06] = vk16k33_data[0x06] | p;
        vk16k33_data[0x0C] = vk16k33_data[0x0C] | p;
        vk16k33_data[0x00] = vk16k33_data[0x00] | (p<<4);
    }   
    if (value==4)
    {
        vk16k33_data[0x02] = vk16k33_data[0x02] | p;
        vk16k33_data[0x04] = vk16k33_data[0x04] | p;
        vk16k33_data[0x0A] = vk16k33_data[0x0A] | p; 
        vk16k33_data[0x0C] = vk16k33_data[0x0C] | p;  
        vk16k33_data[0x00] = vk16k33_data[0x00] | (p<<4);
    }   
    if (value==5)
    {
        vk16k33_data[0x00] = vk16k33_data[0x00] | p;
        vk16k33_data[0x04] = vk16k33_data[0x04] | p;
        vk16k33_data[0x06] = vk16k33_data[0x06] | p;
        vk16k33_data[0x0A] = vk16k33_data[0x0A] | p;
        vk16k33_data[0x0C] = vk16k33_data[0x0C] | p;  
        vk16k33_data[0x00] = vk16k33_data[0x00] | (p<<4);
    }   
    if (value==6)
    {
        vk16k33_data[0x00] = vk16k33_data[0x00] | p;
        vk16k33_data[0x04] = vk16k33_data[0x04] | p;
        vk16k33_data[0x06] = vk16k33_data[0x06] | p;
        vk16k33_data[0x08] = vk16k33_data[0x08] | p;
        vk16k33_data[0x0A] = vk16k33_data[0x0A] | p;
        vk16k33_data[0x0C] = vk16k33_data[0x0C] | p;  
        vk16k33_data[0x00] = vk16k33_data[0x00] | (p<<4);
    }   
    if (value==7)
    {
        vk16k33_data[0x00] = vk16k33_data[0x00] | p;
        vk16k33_data[0x02] = vk16k33_data[0x02] | p;   
        vk16k33_data[0x04] = vk16k33_data[0x04] | p;
    }   
    if (value==8)
    {
        vk16k33_data[0x00] = vk16k33_data[0x00] | p;
        vk16k33_data[0x02] = vk16k33_data[0x02] | p;   
        vk16k33_data[0x04] = vk16k33_data[0x04] | p;
        vk16k33_data[0x06] = vk16k33_data[0x06] | p;
        vk16k33_data[0x08] = vk16k33_data[0x08] | p;
        vk16k33_data[0x0A] = vk16k33_data[0x0A] | p;
        vk16k33_data[0x0C] = vk16k33_data[0x0C] | p;  
        vk16k33_data[0x00] = vk16k33_data[0x00] | (p<<4);
    }   
    if (value==9)
    {
        vk16k33_data[0x00] = vk16k33_data[0x00] | p;
        vk16k33_data[0x02] = vk16k33_data[0x02] | p;   
        vk16k33_data[0x04] = vk16k33_data[0x04] | p;
        vk16k33_data[0x0A] = vk16k33_data[0x0A] | p;
        vk16k33_data[0x0C] = vk16k33_data[0x0C] | p;  
        vk16k33_data[0x00] = vk16k33_data[0x00] | (p<<4);
    }  
}


void show_digit(void)
{
    send_data[0] = vk16k33_data[0x00];  vk16k33_write(0x00, &send_data[0], 1);  // 0
    send_data[0] = vk16k33_data[0x02];  vk16k33_write(0x02, &send_data[0], 1);  // 1
    send_data[0] = vk16k33_data[0x04];  vk16k33_write(0x04, &send_data[0], 1);  // 2
    send_data[0] = vk16k33_data[0x06];  vk16k33_write(0x06, &send_data[0], 1);  // 3
    send_data[0] = vk16k33_data[0x08];  vk16k33_write(0x08, &send_data[0], 1);  // 4
    send_data[0] = vk16k33_data[0x0A];  vk16k33_write(0x0A, &send_data[0], 1);  // 5
    send_data[0] = vk16k33_data[0x0C];  vk16k33_write(0x0C, &send_data[0], 1);  // 6

	send_data[0] = 0xFF;  vk16k33_write(0x03, &send_data[0], 1);  // 7

}


int main(void)
{
	uint16_t digit1=0;
	uint16_t digit2=0;
	uint16_t digit3=0;
	uint16_t digit4=0;
	uint16_t tmp=0;
    
    
    /* WAIT BEFORE DISBALE NRST */

    delay_cycles(100000000);
	SYSCFG_DL_init();

    /* Set LED to indicate start of transfer */
    DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

    delay_cycles(1000000);
 
    /* START oscillator */
 	vk16k33_write(0x21, send_bux, 0);

	/* Display setup */
    vk16k33_write(0x81, send_bux, 0);

	/* Display dimming */
    vk16k33_write(0xE1, send_bux, 0);

    calc_digit_reset();

    while (1) {

		temp_raw = si7021_read(TEMP_MEASURE_CMD);
        humi_raw = si7021_read(HUMI_MEASURE_CMD);
        temperature = ((175.72 * temp_raw) / 65536.0) - 46.85;
        humidity = ((125.0 * humi_raw) / 65536.0) - 6.0;
        
        delay_cycles(3200000);
        
        if (temperature < 30)
            //DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN | GPIO_LEDS_USER_TEST_PIN);
            DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

        else
            DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

        num=(temperature * 10);
        if (num>1258)
			num=0;

		digit4 = num/1000;
		digit3 = (num / 100) % 10;
		digit2 = (num / 10) % 10;
		digit1 = num % 10;

		calc_digit_reset();

        calc_digit_fill(digit4, 3);
        calc_digit_fill(digit3, 2);
        calc_digit_fill(digit2, 1);
        calc_digit_fill(digit1, 0);
        
        show_digit();
      
        delay_cycles(820000);

       // DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN | GPIO_LEDS_USER_TEST_PIN);
       // DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

    }
}

// reg 0x00    xxxx 0000    xxxx = digit 7
// reg 0x00    0000 xxxx    xxxx = digit 0

// reg 0x01    xxxxxxxxx    xxxx = :

// reg 0x02    xxxx 0000    xxxx = digit 8
// reg 0x02    0000 xxxx    xxxx = digit 1

// reg 0x03    xxxxxxxxx    xxxx = .

// reg 0x04    xxxx 0000    xxxx = digit 9
// reg 0x04    0000 xxxx    xxxx = digit 2

// reg 0x06    xxxx 0000    xxxx = digit 10
// reg 0x06    0000 xxxx    xxxx = digit 3

// reg 0x08    xxxx 0000    xxxx = digit 11
// reg 0x08    0000 xxxx    xxxx = digit 4

// reg 0x0A    xxxx 0000    xxxx = digit 12
// reg 0x0A    0000 xxxx    xxxx = digit 5

// reg 0x0C    xxxx 0000    xxxx = digit 13
// reg 0x0C    0000 xxxx    xxxx = digit 6
// reg 0x0D    xxxx xxxx    xxxx = no function
// reg 0x0E    xxxx xxxx    xxxx = no function
// reg 0x0F    xxxx xxxx    xxxx = no function

// glyph "0"   012345     0xA0      
// glyph "1"    12        0x60      
// glyph "2"   01 34 67   0xCB       
// glyph "3"   0123  67   0xF3
// glyph "4"    12  567   0x67
// glyph "5"   0 23  67   0xB3
// glyph "6"     234567   0x3F
// glyph "7"   012        0xE0 
// glyph "8"   01234567   0xFF
// glyph "9"   012  567   0xE7

//    # *************** DISPLAY SEGMENTS *************** #
//
//                0                    9
//                _
//            5 |   | 1            8 \ | / 10
//              |   |                 \|/
//                                 6  - -  7
//            4 |   | 2               /|\
//              | _ |             13 / | \ 11    . 14
//                3                   12
//
//    # *************** DISPLAY SEGMENTS *************** #
