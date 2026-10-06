#include "project.h"
#include <stdio.h>

int main(void)
{
    CyGlobalIntEnable;

    UART_Start();
    ADC_Start();

    for(;;)
    {
        ADC_StartConvert();

        if(ADC_IsEndConversion(ADC_WAIT_FOR_RESULT))
        {
            int16_t value = ADC_GetResult16();

            char buffer[32];
            sprintf(buffer, "ADC: %d\r\n", value);
            UART_PutString(buffer);

            CyDelay(100);
        }
    }
}