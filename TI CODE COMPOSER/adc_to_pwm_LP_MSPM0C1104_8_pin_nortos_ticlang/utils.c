#include "ti_msp_dl_config.h"

typedef unsigned char byte;


void doubleToString(double num, char* str)
{
    int whole       = (int) num;
    double fraction = num - whole;

    int i = 0;
    if (whole == 0) {
        str[i++] = '0';
    } else {
        while (whole != 0) {
            str[i++] = '0' + whole % 10;
            whole    = whole / 10;
        }
    }
    int n = i - 1;
    for (int j = 0; j < i / 2; j++) {
        char temp = str[j];
        str[j]    = str[n];
        str[n]    = temp;
        n--;
    }

    str[i++] = '.';

    for (int k = 0; k < 5; k++) {
        fraction *= 10;
        int digit = (int) fraction;
        str[i++]  = '0' + digit;
        fraction -= digit;
    }
    return;
}


void byteToString(unsigned int num, char* str)
{
    int whole       = (int) num;
 /*   double fraction = num - whole;*/

    int i = 0;
    if (whole == 0) {
        str[i++] = '0';
    } else {
        while (whole != 0) {
            str[i++] = '0' + whole % 10;
            whole    = whole / 10;
        }
    }
    int n = i - 1;
    for (int j = 0; j < i / 2; j++) {
        char temp = str[j];
        str[j]    = str[n];
        str[n]    = temp;
        n--;
    }

   /* str[i++] = '.';

    for (int k = 0; k < 5; k++) {
        fraction *= 10;
        int digit = (int) fraction;
        str[i++]  = '0' + digit;
        fraction -= digit;
    }*/
    return;
}


void uartSend(double num)
{
    char resultbuf[20];
    doubleToString(num, resultbuf);
    for (int i = 0; i < 7; i++) {
        DL_UART_Main_transmitData(UART_0_INST, resultbuf[i]);
        delay_cycles(100000);
    }
    return;
}

void uartSend_VT(byte VT1_adc,byte VT1_pwm)
{
    char datastring[104] = "VT1_ADC: ___ | VT1_PWM1: ___ \r\n ";
    char resultbuffer[5];

   
    byteToString(VT1_adc, resultbuffer);

    if (VT1_adc < 10) { datastring[11] = resultbuffer[0]; }
    if ((VT1_adc > 9) && (VT1_adc<100)) { datastring[10] = resultbuffer[0]; datastring[11] = resultbuffer[1]; }
    if (VT1_adc > 99) { datastring[9] = resultbuffer[0]; datastring[10] = resultbuffer[1]; datastring[11] = resultbuffer[2]; }
    
  
    byteToString(VT1_pwm, resultbuffer);

    if (VT1_pwm < 10) { datastring[27] = resultbuffer[0]; }
    if ((VT1_pwm > 9) && (VT1_pwm<100)) { datastring[26] = resultbuffer[0]; datastring[27] = resultbuffer[1]; }
    if (VT1_pwm > 99) { datastring[25] = resultbuffer[0]; datastring[26] = resultbuffer[1]; datastring[27] = resultbuffer[2]; }
    
   
    for (int i = 0; i < 30; i++) {
        DL_UART_Main_transmitData(UART_0_INST, datastring[i]);
        delay_cycles(10000);
    }

    return;
}


