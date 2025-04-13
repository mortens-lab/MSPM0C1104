#ifndef UTILS_H_
#define UTILS_H_

typedef unsigned char byte;

// UART communication

void uartSend(double num);
/* void uartSend_VT(double num);*/
void uartSend_VT(byte VT1_adc, byte VT1_pwm);

void doubleToString(double num, char* str);
void byteToString(unsigned int num, char* str);

#endif /* UTILS_H_ */
