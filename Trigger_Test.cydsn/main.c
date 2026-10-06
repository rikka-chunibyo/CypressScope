#include "project.h"
#include <stdio.h>

#define SAMPLE_COUNT 1000
#define TRIGGER_LEVEL 100u

uint16_t samples[SAMPLE_COUNT];

int main(void)
{
    CyGlobalIntEnable;

    UART_Start();
    ADC_Start();

    TriggerDAC_Start();
    TriggerDAC_SetValue(TRIGGER_LEVEL);

    TriggerCMP_Start();

    uint8_t td;

    DMA_DmaInitialize(
        2,
        1,
        HI16(CYDEV_PERIPH_BASE),
        HI16(CYDEV_SRAM_BASE)
    );

    for(;;)
    {
        /* Wait for input to be below the trigger level */
        while(Trigger_Out_Read())
        {
        }

        /* Wait for the trigger crossing */
        while(!Trigger_Out_Read())
        {
        }

        td = CyDmaTdAllocate();

        CyDmaTdSetConfiguration(
            td,
            SAMPLE_COUNT * 2,
            CY_DMA_DISABLE_TD,
            CY_DMA_TD_INC_DST_ADR
        );

        CyDmaTdSetAddress(
            td,
            LO16((uint32)ADC_SAR_WRK0_PTR),
            LO16((uint32)samples)
        );

        CyDmaChSetInitialTd(DMA_DmaHandle, td);
        CyDmaChEnable(DMA_DmaHandle, 1);

        Timer_Start();

        CyDelay(1100);

        Timer_Stop();

        CyDmaChDisable(DMA_DmaHandle);

        UART_PutString("CAPTURE\r\n");

        for(uint16_t i = 0; i < SAMPLE_COUNT; i++)
        {
            char buffer[32];

            sprintf(buffer, "%u\r\n", samples[i]);
            UART_PutString(buffer);
        }

        UART_PutString("END\r\n\r\n");

        CyDmaTdFree(td);

        CyDelay(100);
    }
}