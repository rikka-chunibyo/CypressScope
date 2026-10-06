#include "project.h"
#include <stdio.h>

/* ---------------- Configuration ---------------- */

#define SNAPSHOT_SIZE       1000u
#define PRE_TRIGGER         250u
#define POST_TRIGGER        750u

#define RING_SIZE           1200u   /* must be > SNAPSHOT_SIZE + SETTLE_TICKS */
#define SETTLE_TICKS        4u      /* extra ticks so DMA finishes the last sample */

#define TRIG_CH             0       /* 0 = TriggerCMP0, 1 = TriggerCMP1 */
#define TRIGGER0_LEVEL      100u
#define TRIGGER1_LEVEL      100u

/* Shift the trigger position in the output by N samples.
 * With the hardware-recorded edge this should stay 0 unless a
 * constant offset shows up when testing with the PWM edge. */
#define TRIGGER_OFFSET      0

#define ARM_TIMEOUT_TICKS      20000u   /* ~20 s at 1 kHz */
#define TRIGGER_TIMEOUT_TICKS  20000u

/* The software notices the trigger slightly AFTER the hardware bit was
 * recorded. Search this many ring entries before / after the software
 * trigger position for the real 0->1 edge. */
#define EDGE_SEARCH_BEFORE  8
#define EDGE_SEARCH_AFTER   2       /* keep small: post-trigger margin is
                                       SETTLE_TICKS + a few */

/* Bring-up helpers. Set both to 0 when the PC app expects the original
 * two-column protocol. */
#define DEBUG_TRIG_COLUMN   1   /* adds comparator bit as 3rd CSV column
                                   and an EDGE line before the data    */
#define DEBUG_CMP_PRINT     1   /* prints the raw TrigBits value
                                   each time before ARMING              */

/* Trigger comparator output, read from the Status Register (the
 * internal signal), not from the output pin. */
#define TRIGBIT(v)          (((v) >> TRIG_CH) & 1u)
#define TRIG_READ()         TRIGBIT(TrigBits_Read())



/* ---------------- Buffers ---------------- */

static uint16_t ring0[RING_SIZE];
static uint16_t ring1[RING_SIZE];
static uint8_t  ringT[RING_SIZE];   /* comparator bits, one per sample */

static uint16_t channel0[SNAPSHOT_SIZE];
static uint16_t channel1[SNAPSHOT_SIZE];
static uint8_t  trigbits[SNAPSHOT_SIZE];

static uint16_t ring_pos;       /* ticks since start, modulo RING_SIZE */
static uint32_t total_ticks;    /* ticks since start */

static uint8_t td0, td1, tdT;

static uint8_t edge_found;      /* diagnostics from the last capture */
static int8_t  edge_delta;


/* ---------------- Sample tick ---------------- */

/* Polls the Timer terminal count. Must be called often (well under
 * 1 ms) and nothing blocking may sit between calls during capture. */
static inline void tick(void)
{
    if (Timer_ReadStatusRegister() & Timer_STATUS_TC)
    {
        total_ticks++;
        if (++ring_pos >= RING_SIZE)
        {
            ring_pos = 0;
        }
    }
}

static void usb_setup(void) {
    UARTUSB_
    
    UARTUSB_Start();
}


/* ---------------- DMA ---------------- */

static void dma_setup_once(void)
{
    DMA0_DmaInitialize(2, 1, HI16(CYDEV_PERIPH_BASE), HI16(CYDEV_SRAM_BASE));
    DMA1_DmaInitialize(2, 1, HI16(CYDEV_PERIPH_BASE), HI16(CYDEV_SRAM_BASE));
    DMA2_DmaInitialize(1, 1, HI16(CYDEV_PERIPH_BASE), HI16(CYDEV_SRAM_BASE));

    td0 = CyDmaTdAllocate();
    td1 = CyDmaTdAllocate();
    tdT = CyDmaTdAllocate();
}

/* (Re)start all rings at element 0. */
static void dma_arm(void)
{
    CyDmaChDisable(DMA0_DmaHandle);
    CyDmaChDisable(DMA1_DmaHandle);
    CyDmaChDisable(DMA2_DmaHandle);

    CyDmaTdSetConfiguration(td0, RING_SIZE * sizeof(uint16_t), td0,
                            CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(td0, LO16((uint32)ADC0_SAR_WRK0_PTR),
                           LO16((uint32)ring0));
    CyDmaChSetInitialTd(DMA0_DmaHandle, td0);

    CyDmaTdSetConfiguration(td1, RING_SIZE * sizeof(uint16_t), td1,
                            CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(td1, LO16((uint32)ADC1_SAR_WRK0_PTR),
                           LO16((uint32)ring1));
    CyDmaChSetInitialTd(DMA1_DmaHandle, td1);

    /* Comparator bits: 1 byte per sample from the status register. */
    CyDmaTdSetConfiguration(tdT, RING_SIZE * sizeof(uint8_t), tdT,
                            CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(tdT, LO16((uint32)TrigBits_Status_PTR),
                           LO16((uint32)ringT));
    CyDmaChSetInitialTd(DMA2_DmaHandle, tdT);

    CyDmaChEnable(DMA0_DmaHandle, 1);
    CyDmaChEnable(DMA1_DmaHandle, 1);
    CyDmaChEnable(DMA2_DmaHandle, 1);
}

static void dma_stop(void)
{
    CyDmaChDisable(DMA0_DmaHandle);
    CyDmaChDisable(DMA1_DmaHandle);
    CyDmaChDisable(DMA2_DmaHandle);
}


/* ---------------- Edge search ---------------- */

/* Look near 'around' for the first sample whose comparator bit is 1
 * while the previous sample's bit is 0. That sample is the first one
 * taken after the comparator fired. */
static uint8_t find_edge(uint16_t around, uint16_t *edge, int8_t *delta)
{
    for (int8_t d = -EDGE_SEARCH_BEFORE; d <= EDGE_SEARCH_AFTER; d++)
    {
        uint16_t cur  = (uint16_t)(((int32_t)around + d + (int32_t)RING_SIZE)
                                   % (int32_t)RING_SIZE);
        uint16_t prev = (cur == 0u) ? (uint16_t)(RING_SIZE - 1u)
                                    : (uint16_t)(cur - 1u);

        if (TRIGBIT(ringT[cur]) && !TRIGBIT(ringT[prev]))
        {
            *edge  = cur;
            *delta = d;
            return 1;
        }
    }
    return 0;
}


/* ---------------- Capture ---------------- */

/* Returns 1 on success, 0 on timeout. On success, fills channel0/1 and
 * sets *trigger_out to the trigger position within the snapshot. */
static uint8_t capture(uint16_t *trigger_out)
{
    uint32_t limit;

    ring_pos    = 0;
    total_ticks = 0;

    dma_arm();

    (void)Timer_ReadStatusRegister();   /* clear stale TC */
    Timer_Start();

    /* ---- No UART output from here until Timer_Stop() ---- */

    /* 1. Fill enough history that pre-trigger data is real. */
    while (total_ticks < (PRE_TRIGGER + SETTLE_TICKS))
    {
        tick();
    }

    /* 2. Wait for the comparator to be low (armed). */
    limit = total_ticks + ARM_TIMEOUT_TICKS;
    while (TRIG_READ())
    {
        tick();
        if (total_ticks > limit)
        {
            Timer_Stop();
            dma_stop();
            return 0;
        }
    }

    /* 3. Wait for the rising trigger. */
    limit = total_ticks + TRIGGER_TIMEOUT_TICKS;
    while (!TRIG_READ())
    {
        tick();
        if (total_ticks > limit)
        {
            Timer_Stop();
            dma_stop();
            return 0;
        }
    }

    /* Trigger noticed by software: remember roughly where in the ring. */
    uint16_t trig_pos = ring_pos;

    /* 4. Collect post-trigger samples plus settle margin. */
    uint32_t target = total_ticks + POST_TRIGGER + SETTLE_TICKS;
    while (total_ticks < target)
    {
        tick();
    }

    Timer_Stop();
    dma_stop();

    /* ---- Timing-critical part is over; UART is safe again ---- */

    /* Use the hardware-recorded edge when found; otherwise fall back to
     * the software position. */
    uint16_t edge = trig_pos;
    edge_found = find_edge(trig_pos, &edge, &edge_delta);
    if (!edge_found)
    {
        edge       = trig_pos;
        edge_delta = 0;
    }

    /* Unroll the ring: PRE_TRIGGER samples before the edge, then the rest. */
    int32_t start = ((int32_t)edge - (int32_t)PRE_TRIGGER
                     + TRIGGER_OFFSET + 2 * (int32_t)RING_SIZE)
                    % (int32_t)RING_SIZE;

    for (uint16_t i = 0; i < SNAPSHOT_SIZE; i++)
    {
        uint16_t src = (uint16_t)((start + i) % RING_SIZE);
        channel0[i] = ring0[src];
        channel1[i] = ring1[src];
        trigbits[i] = (uint8_t)TRIGBIT(ringT[src]);
    }

    *trigger_out = PRE_TRIGGER;
    return 1;
}


/* ---------------- Output ---------------- */

static void send_capture(uint16_t trigger_index)
{
    char buf[48];

    UART_PutString("CAPTURE\r\n");

    sprintf(buf, "TRIGGER_INDEX,%u\r\n", (unsigned)trigger_index);
    UART_PutString(buf);

#if DEBUG_TRIG_COLUMN
    /* edge found by hardware bit? and how far (in samples) it was from
     * where software noticed the trigger. */
    sprintf(buf, "EDGE,%u,%d\r\n", (unsigned)edge_found, (int)edge_delta);
    UART_PutString(buf);
#endif

    for (uint16_t i = 0; i < SNAPSHOT_SIZE; i++)
    {
#if DEBUG_TRIG_COLUMN
        sprintf(buf, "%u,%u,%u\r\n", (unsigned)channel0[i],
                (unsigned)channel1[i], (unsigned)trigbits[i]);
#else
        sprintf(buf, "%u,%u\r\n", (unsigned)channel0[i], (unsigned)channel1[i]);
#endif
        UART_PutString(buf);
    }

    UART_PutString("END\r\n");
}


/* ---------------- Main ---------------- */

int main(void)
{
    CyGlobalIntEnable;

    UART_Start();
    UART_PutString("START\r\n");

    PWM_1_Start();      /* test edge generator -> TestEdge pin; remove
                           once the alignment test is done */

    ADC0_Start();
    ADC1_Start();

    TriggerDAC0_Start();
    TriggerDAC1_Start();
    TriggerDAC0_SetValue(TRIGGER0_LEVEL);
    TriggerDAC1_SetValue(TRIGGER1_LEVEL);
    TriggerCMP0_Start();
    TriggerCMP1_Start();

    dma_setup_once();

    UART_PutString("READY\r\n");

    for (;;)
    {
        uint16_t trig_index = 0;

#if DEBUG_CMP_PRINT
        {
            /* Raw TrigBits value: bit0 = CMP0, bit1 = CMP1. With the input
             * grounded and a sane threshold, the selected bit should be 0. */
            char dbg[40];
            sprintf(dbg, "DBG,sr=0x%02X\r\n", (unsigned)TrigBits_Read());
            UART_PutString(dbg);
        }
#endif

        UART_PutString("ARMING\r\n");   /* safe: timer is stopped here */

        if (capture(&trig_index))
        {
            send_capture(trig_index);
        }
        else
        {
            UART_PutString("TIMEOUT\r\n");
        }

        CyDelay(2000);
    }
}