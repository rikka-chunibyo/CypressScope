#include "project.h"

CY_ISR(Timer_ISR)
{
    Timer_ReadStatusRegister();
    LED_Write(~LED_Read());
}

int main(void)
{
    CyGlobalIntEnable;

    Timer_ISR_StartEx(Timer_ISR);

    Timer_Start();

    for (;;)
    {
    }
}