#include "project.h"
#include <stdio.h>
#include "app_comm.h"

/* Hardware-gated capture.
 *
 * Schematic instance names this file expects:
 *   Timer (free-running sample tick -> ADC soc), ADC0, ADC1, DMA0, DMA1,
 *   TriggerDAC0/1, TriggerCMP0/1, TrigEnable (Control Reg),
 *   TrigBits (Status Reg), PWM_Sample_Buffer, DebugUART (debug UART),
 *   USBUART (USB CDC app link),
 *   PWM_1 (optional test-edge generator).
 *
 * Control bits (TrigEnable):  control_6 = arm, control_7 = clear all
 * Status bits  (TrigBits):    0 = CMP0 level, 1 = CMP1 level,
 *                             2 = Triggered,  3 = CaptureDone
 *
 * How it works: DMA0/DMA1 write each ADC result into a circular buffer.
 * When armed, the next comparator rising edge starts the PWM counting
 * POST_TRIGGER samples; its terminal count sets CaptureDone, which gates
 * both DMA requests off in hardware. The write position of the DMA is then
 * read back, and the buffer is rotated so the trigger lands at a known row.
 */

/* ---------------- Configuration ---------------- */

#define BUFFER_SIZE         1024u   /* samples per channel (ring and output) */
#define POST_TRIGGER        750u    /* samples kept after the trigger        */
#define PRE_TRIGGER         (BUFFER_SIZE - POST_TRIGGER)

#define SAMPLE_CLOCK_HZ     ((uint32)BCLK__BUS_CLK__HZ)
#define TIMER_PERIOD_CYCLES ((uint32)(Timer_INIT_PERIOD + 1u))
#define SAMPLE_RATE_HZ      (SAMPLE_CLOCK_HZ / TIMER_PERIOD_CYCLES)
#define SAMPLE_PERIOD_NS    ((TIMER_PERIOD_CYCLES * 1000UL) / BCLK__BUS_CLK__MHZ)

#define TRIGGER0_LEVEL      156u    /* VDAC code, 16 mV per code (156 = 2.5 V) */
#define TRIGGER1_LEVEL      156u
#define ADC_BITS            12u

/* If the edge shows up one row late (row 275 instead of 274) use +1,
 * one row early use -1. Calibrate once with the PWM test edge. */
#define TRIGGER_OFFSET      0

#define ARM_TIMEOUT_MS      20000u

#define USE_TEST_PWM        1       /* start PWM_1 (test edge generator)    */
#define BRINGUP_TEST        1       /* run the staged hardware check first  */
#define TIMING_DIAGNOSTICS  1
#define RATE_TEST_WINDOW_MS 10u
#define USB_TEST_MODE       1       /* send a fixed protocol frame over USB */
#define USB_TEST_INTERVAL_MS 1000u

/* TrigEnable control bits */
#define CTL_ARM             0x40u
#define CTL_CLEAR           0x80u

/* TrigBits status bits */
#define ST_CMP0             0x01u
#define ST_CMP1             0x02u
#define ST_TRIGGERED        0x04u
#define ST_DONE             0x08u

#define PREFILL_MS          ((PRE_TRIGGER * 1000u) / SAMPLE_RATE_HZ + 10u)


/* ---------------- Buffers ---------------- */

static uint16_t ring0[BUFFER_SIZE];
static uint16_t ring1[BUFFER_SIZE];

static uint8_t ch0, ch1;        /* DMA channel handles */
static uint8_t td0, td1;


/* ---------------- Diagnostics ---------------- */

static void debug_status(const char *stage, uint32_t elapsed_ms, uint8_t status)
{
    char b[256];

    sprintf(b, "DBG,STATUS,stage=%s,ms=%lu,ctl=0x%02X,sr=0x%02X,cmp0=%u,cmp1=%u,trig=%u,done=%u,cmp0_direct=%u,cmp1_direct=%u,testedge_pin=%u,adc0_raw=%d,adc1_raw=%d,pwm_period=%u,pwm_compare=%u\r\n",
            stage,
            (unsigned long)elapsed_ms,
            (unsigned)TrigEnable_Read(),
            (unsigned)status,
            (unsigned)((status & ST_CMP0) != 0u),
            (unsigned)((status & ST_CMP1) != 0u),
            (unsigned)((status & ST_TRIGGERED) != 0u),
            (unsigned)((status & ST_DONE) != 0u),
            (unsigned)(TriggerCMP0_GetCompare() != 0u),
            (unsigned)(TriggerCMP1_GetCompare() != 0u),
            (unsigned)TestEdge_Read(),
            (int)ADC0_GetResult16(),
            (int)ADC1_GetResult16(),
            (unsigned)PWM_1_ReadPeriod(),
            (unsigned)PWM_1_ReadCompare());
    AppComm_DebugString(b);
}

static void debug_config(void)
{
    char b[256];

    sprintf(b, "DBG,CONFIG,buffer=%u,post=%u,pre=%u,clock_hz=%lu,timer_period=%lu,rate_hz=%lu,period_ns=%lu,prefill_ms=%lu,timeout_ms=%lu,offset=%d,trigger0=%u,trigger1=%u,test_pwm=%u,bringup=%u,usb_test=%u,usb_test_interval_ms=%u\r\n",
            (unsigned)BUFFER_SIZE,
            (unsigned)POST_TRIGGER,
            (unsigned)PRE_TRIGGER,
            (unsigned long)SAMPLE_CLOCK_HZ,
            (unsigned long)TIMER_PERIOD_CYCLES,
            (unsigned long)SAMPLE_RATE_HZ,
            (unsigned long)SAMPLE_PERIOD_NS,
            (unsigned long)PREFILL_MS,
            (unsigned long)ARM_TIMEOUT_MS,
            TRIGGER_OFFSET,
            (unsigned)TRIGGER0_LEVEL,
            (unsigned)TRIGGER1_LEVEL,
            (unsigned)USE_TEST_PWM,
            (unsigned)BRINGUP_TEST,
            (unsigned)USB_TEST_MODE,
            (unsigned)USB_TEST_INTERVAL_MS);
    AppComm_DebugString(b);
}


/* ---------------- DMA ---------------- */

static void dma_setup(void)
{
    char b[128];

    AppComm_DebugString("DBG,DMA,begin\r\n");

    ch0 = DMA0_DmaInitialize(2, 1, HI16(CYDEV_PERIPH_BASE), HI16(CYDEV_SRAM_BASE));
    ch1 = DMA1_DmaInitialize(2, 1, HI16(CYDEV_PERIPH_BASE), HI16(CYDEV_SRAM_BASE));

    td0 = CyDmaTdAllocate();
    td1 = CyDmaTdAllocate();

    /* Each TD loops to itself: a circular buffer. */
    CyDmaTdSetConfiguration(td0, BUFFER_SIZE * sizeof(uint16_t), td0,
                            CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(td0, LO16((uint32)ADC0_SAR_WRK0_PTR), LO16((uint32)ring0));
    CyDmaChSetInitialTd(ch0, td0);

    CyDmaTdSetConfiguration(td1, BUFFER_SIZE * sizeof(uint16_t), td1,
                            CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(td1, LO16((uint32)ADC1_SAR_WRK0_PTR), LO16((uint32)ring1));
    CyDmaChSetInitialTd(ch1, td1);

    /* Second argument 1 = "work separate": the DMA keeps its working copy of
     * the TD apart, which is what lets us read the write position back. */
    CyDmaChEnable(ch0, 1);
    CyDmaChEnable(ch1, 1);

    sprintf(b, "DBG,DMA,configured,ch0=%u,ch1=%u,td0=%u,td1=%u,bytes=%u,ring0=0x%04X,ring1=0x%04X\r\n",
            (unsigned)ch0,
            (unsigned)ch1,
            (unsigned)td0,
            (unsigned)td1,
            (unsigned)(BUFFER_SIZE * sizeof(uint16_t)),
            (unsigned)LO16((uint32)ring0),
            (unsigned)LO16((uint32)ring1));
    AppComm_DebugString(b);
}

static uint16_t dma_destination(uint8_t ch)
{
    return (uint16_t)(((uint16_t)CY_DMA_TDMEM_STRUCT_PTR[ch].TD1[3] << 8)
                      | CY_DMA_TDMEM_STRUCT_PTR[ch].TD1[2]);
}

static uint16_t dma_last_index(uint8_t ch, const uint16_t *ring)
{
    uint16_t dest = dma_destination(ch);
    uint16_t bytes = (uint16_t)(dest - LO16((uint32)ring));
    uint16_t next  = (uint16_t)((bytes / sizeof(uint16_t)) % BUFFER_SIZE);
    return (uint16_t)((next + BUFFER_SIZE - 1u) % BUFFER_SIZE);
}

static uint16_t ring_distance(uint16_t from, uint16_t to)
{
    return (uint16_t)((to + BUFFER_SIZE - from) % BUFFER_SIZE);
}

/* Index (0..BUFFER_SIZE-1) of the most recently written sample. Same
 * readback method as the reference design. Call after the DMA has stopped. */
static uint16_t last_sample_index(uint8_t ch, const uint16_t *ring)
{
    uint16_t dest = dma_destination(ch);
    uint16_t bytes = (uint16_t)(dest - LO16((uint32)ring));
    uint16_t next  = (uint16_t)((bytes / sizeof(uint16_t)) % BUFFER_SIZE);
    uint16_t last  = dma_last_index(ch, ring);
    char b[128];

    sprintf(b, "DBG,DMA_INDEX,ch=%u,dest=0x%04X,base=0x%04X,bytes=%u,next=%u,last=%u\r\n",
            (unsigned)ch,
            (unsigned)dest,
            (unsigned)LO16((uint32)ring),
            (unsigned)bytes,
            (unsigned)next,
            (unsigned)last);
    AppComm_DebugString(b);

    return last;
}

#if TIMING_DIAGNOSTICS
static void timing_diagnostics(void)
{
    char b[192];
    uint16_t start0 = dma_last_index(ch0, ring0);
    uint16_t start1 = dma_last_index(ch1, ring1);
    uint16_t end0;
    uint16_t end1;
    uint16_t delta0;
    uint16_t delta1;

    CyDelay(RATE_TEST_WINDOW_MS);

    end0 = dma_last_index(ch0, ring0);
    end1 = dma_last_index(ch1, ring1);
    delta0 = ring_distance(start0, end0);
    delta1 = ring_distance(start1, end1);

    sprintf(b, "DBG,RATE,window_ms=%u,ch0_start=%u,ch0_end=%u,ch0_samples=%u,ch1_start=%u,ch1_end=%u,ch1_samples=%u,expected=%lu\r\n",
            (unsigned)RATE_TEST_WINDOW_MS,
            (unsigned)start0,
            (unsigned)end0,
            (unsigned)delta0,
            (unsigned)start1,
            (unsigned)end1,
            (unsigned)delta1,
            (unsigned long)((SAMPLE_RATE_HZ * RATE_TEST_WINDOW_MS) / 1000u));
    AppComm_DebugString(b);
}

#endif


/* ---------------- Trigger control ---------------- */

/* Clears both latches and the PWM, and restarts the DMA gate. */
static void clear_latches(void)
{
    debug_status("clear-before", 0u, TrigBits_Read());

    TrigEnable_Write(CTL_CLEAR);
    debug_status("clear-asserted", 0u, TrigBits_Read());
    CyDelayUs(5);                   /* hardware needs a few BUS_CLK cycles */
    TrigEnable_Write(0u);
    PWM_Sample_Buffer_WriteCounter(POST_TRIGGER);

    debug_status("clear-complete", 0u, TrigBits_Read());
}

/* Returns 1 on success, 0 on timeout. */
static uint8_t capture(uint16_t *last0, uint16_t *last1, uint8_t *final_status)
{
    uint32_t ms;
    uint8_t status;
    uint8_t previous_event_status;
#if TIMING_DIAGNOSTICS
    uint16_t trigger_dma_index = 0u;
    uint16_t done_dma_index = 0u;
    uint8_t trigger_dma_valid = 0u;
    uint8_t done_dma_valid = 0u;
#endif

    clear_latches();
    debug_status("capture-start", 0u, TrigBits_Read());

    /* Let the ring fill so the pre-trigger data is real. */
    CyDelay(PREFILL_MS);
    debug_status("capture-prefill-complete", PREFILL_MS, TrigBits_Read());

    /* Arm. A rising comparator edge from here on triggers. */
    TrigEnable_Write(CTL_ARM);
    status = TrigBits_Read();
    previous_event_status = status & (ST_TRIGGERED | ST_DONE);
    debug_status("capture-armed", 0u, status);

    for (ms = 0; ms < ARM_TIMEOUT_MS; ms++)
    {
        status = TrigBits_Read();

#if TIMING_DIAGNOSTICS
        if (((status & ST_TRIGGERED) != 0u) &&
            ((previous_event_status & ST_TRIGGERED) == 0u))
        {
            trigger_dma_index = dma_last_index(ch0, ring0);
            trigger_dma_valid = 1u;
        }

        if (((status & ST_DONE) != 0u) &&
            ((previous_event_status & ST_DONE) == 0u))
        {
            done_dma_index = dma_last_index(ch0, ring0);
            done_dma_valid = 1u;
        }
#endif

        if ((status & (ST_TRIGGERED | ST_DONE)) != previous_event_status)
        {
            debug_status("capture-status-change", ms, status);
            previous_event_status = status & (ST_TRIGGERED | ST_DONE);
        }

        if ((ms != 0u) && ((ms % 1000u) == 0u))
        {
            debug_status("capture-wait", ms, status);
        }

        if (status & ST_DONE)
        {
            break;
        }
        CyDelay(1);
    }

    status = TrigBits_Read();
    TrigEnable_Write(0u);
    *final_status = status;
    debug_status((ms >= ARM_TIMEOUT_MS) ? "capture-timeout" : "capture-complete",
                 ms,
                 status);

#if TIMING_DIAGNOSTICS
    if (trigger_dma_valid && done_dma_valid)
    {
        char b[128];
        sprintf(b, "DBG,POST_MEASURE,trigger_last=%u,done_last=%u,samples_mod_buffer=%u,expected=%u\r\n",
                (unsigned)trigger_dma_index,
                (unsigned)done_dma_index,
                (unsigned)ring_distance(trigger_dma_index, done_dma_index),
                (unsigned)POST_TRIGGER);
        AppComm_DebugString(b);
    }
#endif

    if (ms >= ARM_TIMEOUT_MS)
    {
        return 0;
    }

    /* The DMA has been gated off by CaptureDone: positions are stable. */
    *last0 = last_sample_index(ch0, ring0);
    *last1 = last_sample_index(ch1, ring1);
    return 1;
}


/* ---------------- Output ---------------- */

static uint8_t send_capture(uint32_t capture_number, uint16_t last0, uint16_t last1, uint8_t status)
{
    AppCommCaptureMetadata metadata;

    metadata.capture_number = capture_number;
    metadata.sample_count = BUFFER_SIZE;
    metadata.pre_trigger = PRE_TRIGGER;
    metadata.post_trigger = POST_TRIGGER;
    metadata.trigger_offset = TRIGGER_OFFSET;
    metadata.trigger_index = PRE_TRIGGER;
    metadata.data_start_index = (uint16_t)((last0 + 1u + BUFFER_SIZE + TRIGGER_OFFSET)
                                            % BUFFER_SIZE);
    metadata.end_index0 = last0;
    metadata.end_index1 = last1;
    metadata.status = status;
    metadata.adc_bits = ADC_BITS;
    metadata.sample_clock_hz = SAMPLE_CLOCK_HZ;
    metadata.timer_period_cycles = TIMER_PERIOD_CYCLES;
    metadata.sample_rate_hz = SAMPLE_RATE_HZ;
    metadata.sample_period_ns = SAMPLE_PERIOD_NS;
    metadata.trigger0_level = TRIGGER0_LEVEL;
    metadata.trigger1_level = TRIGGER1_LEVEL;

    return AppComm_SendCapture(&metadata, ring0, ring1);
}


/* ---------------- Bring-up check ---------------- */

#if BRINGUP_TEST
/* Stage test for the wiring. Expected output with the PWM test edge on ADC0:
 *   BU clear  sr=0x00 or 0x01 (bits 2,3 must be 0)
 *   BU trig   after a few ms to ~150 ms, bit 2 set
 *   BU done   about POST_TRIGGER ms after the trigger, bit 3 set */
static void bringup(void)
{
    char b[64];
    uint32_t ms;
    uint32_t t_trig = 0;
    uint8_t status;
    uint8_t previous_event_status;

    clear_latches();
    CyDelay(50);
    status = TrigBits_Read();
    debug_status("bringup-clear", 50u, status);
    sprintf(b, "BU clear sr=0x%02X\r\n", (unsigned)status);
    AppComm_DebugString(b);

    TrigEnable_Write(CTL_ARM);
    status = TrigBits_Read();
    previous_event_status = status & (ST_TRIGGERED | ST_DONE);
    debug_status("bringup-armed", 0u, status);

    for (ms = 0; ms < 5000u; ms++)
    {
        status = TrigBits_Read();

        if ((status & (ST_TRIGGERED | ST_DONE)) != previous_event_status)
        {
            debug_status("bringup-status-change", ms, status);
            previous_event_status = status & (ST_TRIGGERED | ST_DONE);
        }

        if ((ms != 0u) && ((ms % 1000u) == 0u))
        {
            debug_status("bringup-wait", ms, status);
        }

        if (status & ST_TRIGGERED)
        {
            break;
        }
        CyDelay(1);
    }
    t_trig = ms;
    status = TrigBits_Read();
    debug_status("bringup-trigger-result", t_trig, status);
    sprintf(b, "BU trig after %lu ms sr=0x%02X\r\n",
            (unsigned long)t_trig, (unsigned)status);
    AppComm_DebugString(b);

    for (ms = 0; ms < 5000u; ms++)
    {
        status = TrigBits_Read();

        if ((status & (ST_TRIGGERED | ST_DONE)) != previous_event_status)
        {
            debug_status("bringup-status-change", t_trig + ms, status);
            previous_event_status = status & (ST_TRIGGERED | ST_DONE);
        }

        if ((ms != 0u) && ((ms % 1000u) == 0u))
        {
            debug_status("bringup-wait", t_trig + ms, status);
        }

        if (status & ST_DONE)
        {
            break;
        }
        CyDelay(1);
    }
    status = TrigBits_Read();
    debug_status("bringup-done-result", t_trig + ms, status);
    sprintf(b, "BU done after %lu ms more sr=0x%02X\r\n",
            (unsigned long)ms, (unsigned)status);
    AppComm_DebugString(b);

    TrigEnable_Write(0u);
    debug_status("bringup-disarmed", t_trig + ms, TrigBits_Read());
}
#endif


/* ---------------- Main ---------------- */

int main(void)
{
    char dbg[160];
    uint32_t capture_number = 0u;

    CyGlobalIntEnable;

    AppComm_Init();
    debug_config();

#if USB_TEST_MODE
    AppComm_DebugString("DBG,USBTEST,mode=constant_frame\r\n");
    for (;;)
    {
        AppComm_Service();
        AppComm_SendTestFrame();
        CyDelay(USB_TEST_INTERVAL_MS);
    }
#else
#if USE_TEST_PWM
    PWM_1_Start();
    AppComm_DebugString("DBG,INIT,PWM_1=started\r\n");
#endif

    ADC0_Start();
    ADC1_Start();
    AppComm_DebugString("DBG,INIT,ADCs=started\r\n");

    TriggerDAC0_Start();
    TriggerDAC1_Start();
    TriggerDAC0_SetValue(TRIGGER0_LEVEL);
    TriggerDAC1_SetValue(TRIGGER1_LEVEL);
    TriggerCMP0_Start();
    TriggerCMP1_Start();
    sprintf(dbg, "DBG,INIT,trigger_path=started,levels=%u,%u\r\n",
            (unsigned)TRIGGER0_LEVEL,
            (unsigned)TRIGGER1_LEVEL);
    AppComm_DebugString(dbg);

    PWM_Sample_Buffer_Start();
    PWM_Sample_Buffer_WritePeriod(POST_TRIGGER);
    PWM_Sample_Buffer_WriteCounter(POST_TRIGGER);
    sprintf(dbg, "DBG,INIT,post_trigger_pwm=started,period=%u,counter=%u\r\n",
            (unsigned)POST_TRIGGER,
            (unsigned)POST_TRIGGER);
    AppComm_DebugString(dbg);

    dma_setup();

    Timer_Start();      /* starts the sampling */
    AppComm_DebugString("DBG,INIT,Timer=started\r\n");
#if TIMING_DIAGNOSTICS
    timing_diagnostics();
#endif
    debug_status("init-ready", 0u, TrigBits_Read());

    AppComm_DebugString("READY\r\n");

#if BRINGUP_TEST
    bringup();
#endif

    for (;;)
    {
        uint16_t last0 = 0, last1 = 0;
        uint8_t  st = 0;

        AppComm_Service();
        capture_number++;
        sprintf(dbg, "DBG,CYCLE,n=%lu,sr=0x%02X,ctl=0x%02X\r\n",
                (unsigned long)capture_number,
                (unsigned)TrigBits_Read(),
                (unsigned)TrigEnable_Read());
        AppComm_DebugString(dbg);
        AppComm_DebugString("ARMING\r\n");

        if (capture(&last0, &last1, &st))
        {
            sprintf(dbg, "DBG,CYCLE,n=%lu,result=success,status=0x%02X\r\n",
                    (unsigned long)capture_number,
                    (unsigned)st);
            AppComm_DebugString(dbg);
            if (send_capture(capture_number, last0, last1, st) == 0u)
            {
                AppComm_DebugString("DBG,APP,capture_send=failed\r\n");
            }
        }
        else
        {
            sprintf(dbg, "DBG,CYCLE,n=%lu,result=timeout,status=0x%02X\r\n",
                    (unsigned long)capture_number,
                    (unsigned)st);
            AppComm_DebugString(dbg);
            sprintf(dbg, "TIMEOUT,sr=0x%02X\r\n", (unsigned)st);
            AppComm_DebugString(dbg);
        }

        CyDelay(500);
    }
#endif
}
