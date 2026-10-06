#include "project.h"
#include <stdio.h>

#define SAMPLE_COUNT 1000

uint16_t samples[SAMPLE_COUNT];

int main(void)
{
    CyGlobalIntEnable;

    UART_Start();
    ADC_Start();
    Timer_Start();

    uint8_t td;
    uint8_t currentTd;
    uint8_t dmaState;

    /* Initialize DMA */
    DMA_DmaInitialize(
        2,                              /* 2 bytes per burst */
        1,                              /* 1 request per burst */
        HI16(CYDEV_PERIPH_BASE),
        HI16(CYDEV_SRAM_BASE)
    );

    /* Allocate DMA descriptor */
    td = CyDmaTdAllocate();

    /*
     * Transfer 1000 uint16_t samples.
     * DMA transfer count is in bytes.
     *
     * The TD points back to itself so the buffer
     * continuously fills and wraps around.
     */
    CyDmaTdSetConfiguration(
        td,
        SAMPLE_COUNT * sizeof(uint16_t),
        td,
        CY_DMA_TD_INC_DST_ADR
    );

    CyDmaTdSetAddress(
        td,
        LO16((uint32)ADC_SAR_WRK0_PTR),
        LO16((uint32)samples)
    );

    CyDmaChSetInitialTd(DMA_DmaHandle, td);
    CyDmaChEnable(DMA_DmaHandle, 1);

    for(;;)
    {
        /*
         * Wait until this pass through the TD finishes.
         *
         * Because the TD loops back to itself, DMA continues
         * sampling while we process the previous buffer.
         */
        do
        {
            CyDmaChStatus(
                DMA_DmaHandle,
                &currentTd,
                &dmaState
            );
        }
        while(dmaState & CY_DMA_STATUS_TD_ACTIVE);

        /*
         * Dump the current contents of the buffer.
         *
         * DMA continues running and will refill the buffer.
         */
        for(uint16_t i = 0; i < SAMPLE_COUNT; i++)
        {
            char buffer[32];

            sprintf(buffer, "%u\r\n", samples[i]);
            UART_PutString(buffer);
        }
    }
}