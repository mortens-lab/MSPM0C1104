
#include "utils.h"
#include "ti_msp_dl_config.h"

volatile bool gCheckADC;

#define RESULT_SIZE (1)
volatile uint16_t gAdcResult0[RESULT_SIZE]; /* PA24 */

typedef unsigned char byte;

byte VT1[128] = {
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,
    32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 60,
    60, 60, 60, 60, 60, 60, 60, 78, 80, 82, 84, 86, 88, 90, 92, 94,
    96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126,
    128, 130, 132, 134, 136, 138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158,
    160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182, 184, 186, 188, 190,
    192, 194, 196, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 218, 220, 222,
    224, 226, 228, 230, 232, 234, 236, 238, 240, 242, 244, 246, 248, 250, 252, 255
};

byte j=0;

int main(void)
{
    SYSCFG_DL_init();

    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    gCheckADC  = false;
    uint16_t i = 0;

    while (1)  //run measurement code repeatedly
    {
         DL_ADC12_startConversion(ADC12_0_INST);

        /* First pass. Wait until all data channels have been loaded. */
        while (gCheckADC == false) {
          __WFE();
        }  
       
        /* Store ADC Results into their respective buffer */
        gAdcResult0[i] =
           DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);

        DL_ADC12_enableConversions(ADC12_0_INST);
        delay_cycles(880000);

        uartSend_VT(gAdcResult0[i]>>2,VT1[gAdcResult0[i]>>3]);

        DL_TimerA_setCaptureCompareValue(PWM_0_INST , VT1[gAdcResult0[i]>>3], DL_TIMER_CC_0_INDEX);  
  
        delay_cycles(880000);
    }
}


/* Check for the last result to be loaded then change boolean */
void ADC12_0_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            gCheckADC = true;
            break;
        default:
            break;
    }
}


