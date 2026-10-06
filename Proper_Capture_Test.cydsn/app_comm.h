#ifndef APP_COMM_H
#define APP_COMM_H

#include <stdint.h>

typedef struct
{
    uint32_t capture_number;
    uint16_t sample_count;
    uint16_t pre_trigger;
    uint16_t post_trigger;
    int16_t trigger_offset;
    uint16_t trigger_index;
    uint16_t data_start_index;
    uint16_t end_index0;
    uint16_t end_index1;
    uint8_t status;
    uint8_t adc_bits;
    uint32_t sample_clock_hz;
    uint32_t timer_period_cycles;
    uint32_t sample_rate_hz;
    uint32_t sample_period_ns;
    uint16_t trigger0_level;
    uint16_t trigger1_level;
} AppCommCaptureMetadata;

void AppComm_Init(void);
void AppComm_Service(void);
void AppComm_DebugString(const char *text);
uint8_t AppComm_SendCapture(const AppCommCaptureMetadata *metadata,
                            const uint16_t *channel0,
                            const uint16_t *channel1);

#endif
