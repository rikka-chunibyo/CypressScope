#include "project.h"
#include <stdio.h>

#include "../../app_comm.h"
#include "../feature_config.h"
#include "oscilloscope.h"

#define OSCILLOSCOPE_BUFFER_SIZE         1024u
#define OSCILLOSCOPE_POST_TRIGGER        750u
#define OSCILLOSCOPE_PRE_TRIGGER         (OSCILLOSCOPE_BUFFER_SIZE - OSCILLOSCOPE_POST_TRIGGER)

#define OSCILLOSCOPE_SAMPLE_CLOCK_HZ     ((uint32)BCLK__BUS_CLK__HZ)
#define OSCILLOSCOPE_TIMER_PERIOD_CYCLES ((uint32)(Timer_INIT_PERIOD + 1u))
#define OSCILLOSCOPE_SAMPLE_RATE_HZ      (OSCILLOSCOPE_SAMPLE_CLOCK_HZ / OSCILLOSCOPE_TIMER_PERIOD_CYCLES)
#define OSCILLOSCOPE_SAMPLE_PERIOD_NS    ((OSCILLOSCOPE_TIMER_PERIOD_CYCLES * 1000UL) / BCLK__BUS_CLK__MHZ)

#define OSCILLOSCOPE_TRIGGER0_LEVEL      156u
#define OSCILLOSCOPE_TRIGGER1_LEVEL      156u
#define OSCILLOSCOPE_ADC_BITS            12u
#define OSCILLOSCOPE_TRIGGER_OFFSET      0
#define OSCILLOSCOPE_ARM_TIMEOUT_MS      20000u

#define OSCILLOSCOPE_USE_TEST_PWM        1
#define OSCILLOSCOPE_BRINGUP_TEST        1
#define OSCILLOSCOPE_TIMING_DIAGNOSTICS  1
#define OSCILLOSCOPE_RATE_TEST_WINDOW_MS 10u

#define OSCILLOSCOPE_CTL_ARM             0x40u
#define OSCILLOSCOPE_CTL_CLEAR           0x80u

#define OSCILLOSCOPE_ST_CMP0             0x01u
#define OSCILLOSCOPE_ST_CMP1             0x02u
#define OSCILLOSCOPE_ST_TRIGGERED        0x04u
#define OSCILLOSCOPE_ST_DONE             0x08u

#define OSCILLOSCOPE_PREFILL_MS          ((OSCILLOSCOPE_PRE_TRIGGER * 1000u) / OSCILLOSCOPE_SAMPLE_RATE_HZ + 10u)

static uint16_t oscilloscope_ring0[OSCILLOSCOPE_BUFFER_SIZE];
static uint16_t oscilloscope_ring1[OSCILLOSCOPE_BUFFER_SIZE];

static uint8_t oscilloscope_dma_ch0;
static uint8_t oscilloscope_dma_ch1;
static uint8_t oscilloscope_dma_td0;
static uint8_t oscilloscope_dma_td1;
static uint32_t oscilloscope_capture_number;

static void oscilloscope_debug_status(const char *stage, uint32_t elapsed_ms, uint8_t status)
{
    char b[256];

    sprintf(b, "DBG,STATUS,stage=%s,ms=%lu,ctl=0x%02X,sr=0x%02X,cmp0=%u,cmp1=%u,trig=%u,done=%u,cmp0_direct=%u,cmp1_direct=%u,testedge_pin=%u,adc0_raw=%d,adc1_raw=%d,pwm_period=%u,pwm_compare=%u\r\n",
            stage,
            (unsigned long)elapsed_ms,
            (unsigned)TrigEnable_Read(),
            (unsigned)status,
            (unsigned)((status & OSCILLOSCOPE_ST_CMP0) != 0u),
            (unsigned)((status & OSCILLOSCOPE_ST_CMP1) != 0u),
            (unsigned)((status & OSCILLOSCOPE_ST_TRIGGERED) != 0u),
            (unsigned)((status & OSCILLOSCOPE_ST_DONE) != 0u),
            (unsigned)(TriggerCMP0_GetCompare() != 0u),
            (unsigned)(TriggerCMP1_GetCompare() != 0u),
            (unsigned)TestEdge_Read(),
            (int)ADC0_GetResult16(),
            (int)ADC1_GetResult16(),
            (unsigned)PWM_1_ReadPeriod(),
            (unsigned)PWM_1_ReadCompare());
    AppComm_DebugString(b);
}

void Oscilloscope_ReportConfig(void)
{
    char b[256];

    sprintf(b, "DBG,CONFIG,feature=oscilloscope,buffer=%u,post=%u,pre=%u,clock_hz=%lu,timer_period=%lu,rate_hz=%lu,period_ns=%lu,prefill_ms=%lu,timeout_ms=%lu,offset=%d,trigger0=%u,trigger1=%u,test_pwm=%u,bringup=%u,usb_test=%u,usb_test_interval_ms=%u\r\n",
            (unsigned)OSCILLOSCOPE_BUFFER_SIZE,
            (unsigned)OSCILLOSCOPE_POST_TRIGGER,
            (unsigned)OSCILLOSCOPE_PRE_TRIGGER,
            (unsigned long)OSCILLOSCOPE_SAMPLE_CLOCK_HZ,
            (unsigned long)OSCILLOSCOPE_TIMER_PERIOD_CYCLES,
            (unsigned long)OSCILLOSCOPE_SAMPLE_RATE_HZ,
            (unsigned long)OSCILLOSCOPE_SAMPLE_PERIOD_NS,
            (unsigned long)OSCILLOSCOPE_PREFILL_MS,
            (unsigned long)OSCILLOSCOPE_ARM_TIMEOUT_MS,
            OSCILLOSCOPE_TRIGGER_OFFSET,
            (unsigned)OSCILLOSCOPE_TRIGGER0_LEVEL,
            (unsigned)OSCILLOSCOPE_TRIGGER1_LEVEL,
            (unsigned)OSCILLOSCOPE_USE_TEST_PWM,
            (unsigned)OSCILLOSCOPE_BRINGUP_TEST,
            (unsigned)(ACTIVE_FEATURE_MODE == FEATURE_MODE_USB_TEST),
            (unsigned)FEATURE_USB_TEST_INTERVAL_MS);
    AppComm_DebugString(b);
}

static void oscilloscope_dma_setup(void)
{
    char b[128];

    AppComm_DebugString("DBG,DMA,begin\r\n");

    oscilloscope_dma_ch0 = DMA0_DmaInitialize(2, 1, HI16(CYDEV_PERIPH_BASE), HI16(CYDEV_SRAM_BASE));
    oscilloscope_dma_ch1 = DMA1_DmaInitialize(2, 1, HI16(CYDEV_PERIPH_BASE), HI16(CYDEV_SRAM_BASE));

    oscilloscope_dma_td0 = CyDmaTdAllocate();
    oscilloscope_dma_td1 = CyDmaTdAllocate();

    CyDmaTdSetConfiguration(oscilloscope_dma_td0,
                            OSCILLOSCOPE_BUFFER_SIZE * sizeof(uint16_t),
                            oscilloscope_dma_td0,
                            CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(oscilloscope_dma_td0,
                      LO16((uint32)ADC0_SAR_WRK0_PTR),
                      LO16((uint32)oscilloscope_ring0));
    CyDmaChSetInitialTd(oscilloscope_dma_ch0, oscilloscope_dma_td0);

    CyDmaTdSetConfiguration(oscilloscope_dma_td1,
                            OSCILLOSCOPE_BUFFER_SIZE * sizeof(uint16_t),
                            oscilloscope_dma_td1,
                            CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(oscilloscope_dma_td1,
                      LO16((uint32)ADC1_SAR_WRK0_PTR),
                      LO16((uint32)oscilloscope_ring1));
    CyDmaChSetInitialTd(oscilloscope_dma_ch1, oscilloscope_dma_td1);

    CyDmaChEnable(oscilloscope_dma_ch0, 1);
    CyDmaChEnable(oscilloscope_dma_ch1, 1);

    sprintf(b, "DBG,DMA,configured,ch0=%u,ch1=%u,td0=%u,td1=%u,bytes=%u,ring0=0x%04X,ring1=0x%04X\r\n",
            (unsigned)oscilloscope_dma_ch0,
            (unsigned)oscilloscope_dma_ch1,
            (unsigned)oscilloscope_dma_td0,
            (unsigned)oscilloscope_dma_td1,
            (unsigned)(OSCILLOSCOPE_BUFFER_SIZE * sizeof(uint16_t)),
            (unsigned)LO16((uint32)oscilloscope_ring0),
            (unsigned)LO16((uint32)oscilloscope_ring1));
    AppComm_DebugString(b);
}

static uint16_t oscilloscope_dma_destination(uint8_t ch)
{
    return (uint16_t)(((uint16_t)CY_DMA_TDMEM_STRUCT_PTR[ch].TD1[3] << 8)
                      | CY_DMA_TDMEM_STRUCT_PTR[ch].TD1[2]);
}

static uint16_t oscilloscope_dma_last_index(uint8_t ch, const uint16_t *ring)
{
    uint16_t dest = oscilloscope_dma_destination(ch);
    uint16_t bytes = (uint16_t)(dest - LO16((uint32)ring));
    uint16_t next = (uint16_t)((bytes / sizeof(uint16_t)) % OSCILLOSCOPE_BUFFER_SIZE);

    return (uint16_t)((next + OSCILLOSCOPE_BUFFER_SIZE - 1u) % OSCILLOSCOPE_BUFFER_SIZE);
}

static uint16_t oscilloscope_ring_distance(uint16_t from, uint16_t to)
{
    return (uint16_t)((to + OSCILLOSCOPE_BUFFER_SIZE - from) % OSCILLOSCOPE_BUFFER_SIZE);
}

static uint16_t oscilloscope_last_sample_index(uint8_t ch, const uint16_t *ring)
{
    uint16_t dest = oscilloscope_dma_destination(ch);
    uint16_t bytes = (uint16_t)(dest - LO16((uint32)ring));
    uint16_t next = (uint16_t)((bytes / sizeof(uint16_t)) % OSCILLOSCOPE_BUFFER_SIZE);
    uint16_t last = oscilloscope_dma_last_index(ch, ring);
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

#if OSCILLOSCOPE_TIMING_DIAGNOSTICS
static void oscilloscope_timing_diagnostics(void)
{
    char b[192];
    uint16_t start0 = oscilloscope_dma_last_index(oscilloscope_dma_ch0, oscilloscope_ring0);
    uint16_t start1 = oscilloscope_dma_last_index(oscilloscope_dma_ch1, oscilloscope_ring1);
    uint16_t end0;
    uint16_t end1;
    uint16_t delta0;
    uint16_t delta1;

    CyDelay(OSCILLOSCOPE_RATE_TEST_WINDOW_MS);

    end0 = oscilloscope_dma_last_index(oscilloscope_dma_ch0, oscilloscope_ring0);
    end1 = oscilloscope_dma_last_index(oscilloscope_dma_ch1, oscilloscope_ring1);
    delta0 = oscilloscope_ring_distance(start0, end0);
    delta1 = oscilloscope_ring_distance(start1, end1);

    sprintf(b, "DBG,RATE,window_ms=%u,ch0_start=%u,ch0_end=%u,ch0_samples=%u,ch1_start=%u,ch1_end=%u,ch1_samples=%u,expected=%lu\r\n",
            (unsigned)OSCILLOSCOPE_RATE_TEST_WINDOW_MS,
            (unsigned)start0,
            (unsigned)end0,
            (unsigned)delta0,
            (unsigned)start1,
            (unsigned)end1,
            (unsigned)delta1,
            (unsigned long)((OSCILLOSCOPE_SAMPLE_RATE_HZ * OSCILLOSCOPE_RATE_TEST_WINDOW_MS) / 1000u));
    AppComm_DebugString(b);
}
#endif

static void oscilloscope_clear_latches(void)
{
    oscilloscope_debug_status("clear-before", 0u, TrigBits_Read());

    TrigEnable_Write(OSCILLOSCOPE_CTL_CLEAR);
    oscilloscope_debug_status("clear-asserted", 0u, TrigBits_Read());
    CyDelayUs(5);
    TrigEnable_Write(0u);
    PWM_Sample_Buffer_WriteCounter(OSCILLOSCOPE_POST_TRIGGER);

    oscilloscope_debug_status("clear-complete", 0u, TrigBits_Read());
}

static uint8_t oscilloscope_capture(uint16_t *last0, uint16_t *last1, uint8_t *final_status)
{
    uint32_t ms;
    uint8_t status;
    uint8_t previous_event_status;
#if OSCILLOSCOPE_TIMING_DIAGNOSTICS
    uint16_t trigger_dma_index = 0u;
    uint16_t done_dma_index = 0u;
    uint8_t trigger_dma_valid = 0u;
    uint8_t done_dma_valid = 0u;
#endif

    oscilloscope_clear_latches();
    oscilloscope_debug_status("capture-start", 0u, TrigBits_Read());

    CyDelay(OSCILLOSCOPE_PREFILL_MS);
    oscilloscope_debug_status("capture-prefill-complete", OSCILLOSCOPE_PREFILL_MS, TrigBits_Read());

    TrigEnable_Write(OSCILLOSCOPE_CTL_ARM);
    status = TrigBits_Read();
    previous_event_status = status & (OSCILLOSCOPE_ST_TRIGGERED | OSCILLOSCOPE_ST_DONE);
    oscilloscope_debug_status("capture-armed", 0u, status);

    for (ms = 0; ms < OSCILLOSCOPE_ARM_TIMEOUT_MS; ms++)
    {
        status = TrigBits_Read();

#if OSCILLOSCOPE_TIMING_DIAGNOSTICS
        if (((status & OSCILLOSCOPE_ST_TRIGGERED) != 0u) &&
            ((previous_event_status & OSCILLOSCOPE_ST_TRIGGERED) == 0u))
        {
            trigger_dma_index = oscilloscope_dma_last_index(oscilloscope_dma_ch0, oscilloscope_ring0);
            trigger_dma_valid = 1u;
        }

        if (((status & OSCILLOSCOPE_ST_DONE) != 0u) &&
            ((previous_event_status & OSCILLOSCOPE_ST_DONE) == 0u))
        {
            done_dma_index = oscilloscope_dma_last_index(oscilloscope_dma_ch0, oscilloscope_ring0);
            done_dma_valid = 1u;
        }
#endif

        if ((status & (OSCILLOSCOPE_ST_TRIGGERED | OSCILLOSCOPE_ST_DONE)) != previous_event_status)
        {
            oscilloscope_debug_status("capture-status-change", ms, status);
            previous_event_status = status & (OSCILLOSCOPE_ST_TRIGGERED | OSCILLOSCOPE_ST_DONE);
        }

        if ((ms != 0u) && ((ms % 1000u) == 0u))
        {
            oscilloscope_debug_status("capture-wait", ms, status);
        }

        if ((status & OSCILLOSCOPE_ST_DONE) != 0u)
        {
            break;
        }
        CyDelay(1);
    }

    status = TrigBits_Read();
    TrigEnable_Write(0u);
    *final_status = status;
    oscilloscope_debug_status((ms >= OSCILLOSCOPE_ARM_TIMEOUT_MS) ? "capture-timeout" : "capture-complete",
                              ms,
                              status);

#if OSCILLOSCOPE_TIMING_DIAGNOSTICS
    if (trigger_dma_valid && done_dma_valid)
    {
        char b[128];
        sprintf(b, "DBG,POST_MEASURE,trigger_last=%u,done_last=%u,samples_mod_buffer=%u,expected=%u\r\n",
                (unsigned)trigger_dma_index,
                (unsigned)done_dma_index,
                (unsigned)oscilloscope_ring_distance(trigger_dma_index, done_dma_index),
                (unsigned)OSCILLOSCOPE_POST_TRIGGER);
        AppComm_DebugString(b);
    }
#endif

    if (ms >= OSCILLOSCOPE_ARM_TIMEOUT_MS)
    {
        return 0u;
    }

    *last0 = oscilloscope_last_sample_index(oscilloscope_dma_ch0, oscilloscope_ring0);
    *last1 = oscilloscope_last_sample_index(oscilloscope_dma_ch1, oscilloscope_ring1);
    return 1u;
}

static uint8_t oscilloscope_send_capture(uint16_t last0, uint16_t last1, uint8_t status)
{
    AppCommCaptureMetadata metadata;

    metadata.capture_number = oscilloscope_capture_number;
    metadata.sample_count = OSCILLOSCOPE_BUFFER_SIZE;
    metadata.pre_trigger = OSCILLOSCOPE_PRE_TRIGGER;
    metadata.post_trigger = OSCILLOSCOPE_POST_TRIGGER;
    metadata.trigger_offset = OSCILLOSCOPE_TRIGGER_OFFSET;
    metadata.trigger_index = OSCILLOSCOPE_PRE_TRIGGER;
    metadata.data_start_index = (uint16_t)((last0 + 1u + OSCILLOSCOPE_BUFFER_SIZE + OSCILLOSCOPE_TRIGGER_OFFSET)
                                           % OSCILLOSCOPE_BUFFER_SIZE);
    metadata.end_index0 = last0;
    metadata.end_index1 = last1;
    metadata.status = status;
    metadata.adc_bits = OSCILLOSCOPE_ADC_BITS;
    metadata.sample_clock_hz = OSCILLOSCOPE_SAMPLE_CLOCK_HZ;
    metadata.timer_period_cycles = OSCILLOSCOPE_TIMER_PERIOD_CYCLES;
    metadata.sample_rate_hz = OSCILLOSCOPE_SAMPLE_RATE_HZ;
    metadata.sample_period_ns = OSCILLOSCOPE_SAMPLE_PERIOD_NS;
    metadata.trigger0_level = OSCILLOSCOPE_TRIGGER0_LEVEL;
    metadata.trigger1_level = OSCILLOSCOPE_TRIGGER1_LEVEL;

    return AppComm_SendCapture(&metadata, oscilloscope_ring0, oscilloscope_ring1);
}

#if OSCILLOSCOPE_BRINGUP_TEST
static void oscilloscope_bringup(void)
{
    char b[64];
    uint32_t ms;
    uint32_t trigger_ms = 0u;
    uint8_t status;
    uint8_t previous_event_status;

    oscilloscope_clear_latches();
    CyDelay(50);
    status = TrigBits_Read();
    oscilloscope_debug_status("bringup-clear", 50u, status);
    sprintf(b, "BU clear sr=0x%02X\r\n", (unsigned)status);
    AppComm_DebugString(b);

    TrigEnable_Write(OSCILLOSCOPE_CTL_ARM);
    status = TrigBits_Read();
    previous_event_status = status & (OSCILLOSCOPE_ST_TRIGGERED | OSCILLOSCOPE_ST_DONE);
    oscilloscope_debug_status("bringup-armed", 0u, status);

    for (ms = 0; ms < 5000u; ms++)
    {
        status = TrigBits_Read();

        if ((status & (OSCILLOSCOPE_ST_TRIGGERED | OSCILLOSCOPE_ST_DONE)) != previous_event_status)
        {
            oscilloscope_debug_status("bringup-status-change", ms, status);
            previous_event_status = status & (OSCILLOSCOPE_ST_TRIGGERED | OSCILLOSCOPE_ST_DONE);
        }

        if ((ms != 0u) && ((ms % 1000u) == 0u))
        {
            oscilloscope_debug_status("bringup-wait", ms, status);
        }

        if ((status & OSCILLOSCOPE_ST_TRIGGERED) != 0u)
        {
            break;
        }
        CyDelay(1);
    }
    trigger_ms = ms;
    status = TrigBits_Read();
    oscilloscope_debug_status("bringup-trigger-result", trigger_ms, status);
    sprintf(b, "BU trig after %lu ms sr=0x%02X\r\n",
            (unsigned long)trigger_ms,
            (unsigned)status);
    AppComm_DebugString(b);

    for (ms = 0; ms < 5000u; ms++)
    {
        status = TrigBits_Read();

        if ((status & (OSCILLOSCOPE_ST_TRIGGERED | OSCILLOSCOPE_ST_DONE)) != previous_event_status)
        {
            oscilloscope_debug_status("bringup-status-change", trigger_ms + ms, status);
            previous_event_status = status & (OSCILLOSCOPE_ST_TRIGGERED | OSCILLOSCOPE_ST_DONE);
        }

        if ((ms != 0u) && ((ms % 1000u) == 0u))
        {
            oscilloscope_debug_status("bringup-wait", trigger_ms + ms, status);
        }

        if ((status & OSCILLOSCOPE_ST_DONE) != 0u)
        {
            break;
        }
        CyDelay(1);
    }
    status = TrigBits_Read();
    oscilloscope_debug_status("bringup-done-result", trigger_ms + ms, status);
    sprintf(b, "BU done after %lu ms more sr=0x%02X\r\n",
            (unsigned long)ms,
            (unsigned)status);
    AppComm_DebugString(b);

    TrigEnable_Write(0u);
    oscilloscope_debug_status("bringup-disarmed", trigger_ms + ms, TrigBits_Read());
}
#endif

static void oscilloscope_init(void)
{
    char dbg[160];

    oscilloscope_capture_number = 0u;
    Oscilloscope_ReportConfig();

#if OSCILLOSCOPE_USE_TEST_PWM
    PWM_1_Start();
    AppComm_DebugString("DBG,INIT,PWM_1=started\r\n");
#endif

    ADC0_Start();
    ADC1_Start();
    AppComm_DebugString("DBG,INIT,ADCs=started\r\n");

    TriggerDAC0_Start();
    TriggerDAC1_Start();
    TriggerDAC0_SetValue(OSCILLOSCOPE_TRIGGER0_LEVEL);
    TriggerDAC1_SetValue(OSCILLOSCOPE_TRIGGER1_LEVEL);
    TriggerCMP0_Start();
    TriggerCMP1_Start();
    sprintf(dbg, "DBG,INIT,trigger_path=started,levels=%u,%u\r\n",
            (unsigned)OSCILLOSCOPE_TRIGGER0_LEVEL,
            (unsigned)OSCILLOSCOPE_TRIGGER1_LEVEL);
    AppComm_DebugString(dbg);

    PWM_Sample_Buffer_Start();
    PWM_Sample_Buffer_WritePeriod(OSCILLOSCOPE_POST_TRIGGER);
    PWM_Sample_Buffer_WriteCounter(OSCILLOSCOPE_POST_TRIGGER);
    sprintf(dbg, "DBG,INIT,post_trigger_pwm=started,period=%u,counter=%u\r\n",
            (unsigned)OSCILLOSCOPE_POST_TRIGGER,
            (unsigned)OSCILLOSCOPE_POST_TRIGGER);
    AppComm_DebugString(dbg);

    oscilloscope_dma_setup();

    Timer_Start();
    AppComm_DebugString("DBG,INIT,Timer=started\r\n");

#if OSCILLOSCOPE_TIMING_DIAGNOSTICS
    oscilloscope_timing_diagnostics();
#endif

    oscilloscope_debug_status("init-ready", 0u, TrigBits_Read());
    AppComm_DebugString("READY\r\n");

#if OSCILLOSCOPE_BRINGUP_TEST
    oscilloscope_bringup();
#endif
}

static void oscilloscope_service(void)
{
    uint16_t last0 = 0u;
    uint16_t last1 = 0u;
    uint8_t status = 0u;
    char dbg[160];

    oscilloscope_capture_number++;
    sprintf(dbg, "DBG,CYCLE,n=%lu,sr=0x%02X,ctl=0x%02X\r\n",
            (unsigned long)oscilloscope_capture_number,
            (unsigned)TrigBits_Read(),
            (unsigned)TrigEnable_Read());
    AppComm_DebugString(dbg);
    AppComm_DebugString("ARMING\r\n");

    if (oscilloscope_capture(&last0, &last1, &status))
    {
        sprintf(dbg, "DBG,CYCLE,n=%lu,result=success,status=0x%02X\r\n",
                (unsigned long)oscilloscope_capture_number,
                (unsigned)status);
        AppComm_DebugString(dbg);
        if (oscilloscope_send_capture(last0, last1, status) == 0u)
        {
            AppComm_DebugString("DBG,APP,capture_send=failed\r\n");
        }
    }
    else
    {
        sprintf(dbg, "DBG,CYCLE,n=%lu,result=timeout,status=0x%02X\r\n",
                (unsigned long)oscilloscope_capture_number,
                (unsigned)status);
        AppComm_DebugString(dbg);
        sprintf(dbg, "TIMEOUT,sr=0x%02X\r\n", (unsigned)status);
        AppComm_DebugString(dbg);
    }

    CyDelay(500);
}

const FeatureModule *Oscilloscope_GetModule(void)
{
    static const FeatureModule module =
    {
        "oscilloscope",
        oscilloscope_init,
        oscilloscope_service
    };

    return &module;
}
