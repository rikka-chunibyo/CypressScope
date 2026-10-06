#include "project.h"

int main(void)
{
    CyGlobalIntEnable;
    UART_Start();

    for (;;)
    {
        UART_PutChar(0x55);
    }
}