#include "project.h"
#include "app_comm.h"
#include <stdio.h>
#include <string.h>

#define APP_PROTOCOL_VERSION   1u
#define APP_USB_TX_CHUNK_SIZE 64u
#define APP_USB_TX_TIMEOUT_MS 1000u
#define APP_LIVE_FRAME_HEADER_SIZE 20u
#define APP_LIVE_MAX_SAMPLES 64u

#define APP_USB_ERROR_NONE       0u
#define APP_USB_ERROR_NO_CDC     1u
#define APP_USB_ERROR_TX_TIMEOUT 2u

#if defined(UARTDebug_TX_BUFFER_SIZE) && !defined(DebugUART_TX_BUFFER_SIZE)
#define DebugUART_Start       UARTDebug_Start
#define DebugUART_PutString   UARTDebug_PutString
#endif

#if defined(UARTUSB_ENABLE_CDC_CLASS_API) && !defined(USBUART_ENABLE_CDC_CLASS_API)
#define USBUART_5V_OPERATION      UARTUSB_5V_OPERATION
#define USBUART_Start              UARTUSB_Start
#define USBUART_GetConfiguration   UARTUSB_GetConfiguration
#define USBUART_CDC_Init           UARTUSB_CDC_Init
#define USBUART_CDCIsReady         UARTUSB_CDCIsReady
#define USBUART_PutData            UARTUSB_PutData
#endif

static uint8_t usb_cdc_initialized;
static uint8_t usb_waiting_logged;
static uint8_t usb_cdc_init_failed_logged;
static uint8_t usb_last_error;

static const char app_usb_frame[] =
    "CAPTURE,BEGIN\r\n"
    "META,VERSION,1\r\n"
    "META,CAPTURE_NUMBER,0\r\n"
    "META,FORMAT,CSV16_ADC_COUNTS\r\n"
    "META,CHANNEL_COUNT,2\r\n"
    "META,SAMPLE_COUNT,16\r\n"
    "META,ADC_BITS,12\r\n"
    "META,SAMPLE_CLOCK_HZ,24000000\r\n"
    "META,TIMER_PERIOD_CYCLES,999\r\n"
    "META,SAMPLE_RATE_HZ,24024\r\n"
    "META,SAMPLE_PERIOD_NS,41625\r\n"
    "META,PRE_TRIGGER,8\r\n"
    "META,POST_TRIGGER,8\r\n"
    "META,TRIGGER_INDEX,8\r\n"
    "META,TRIGGER_OFFSET,0\r\n"
    "META,TRIGGER0_LEVEL,156\r\n"
    "META,TRIGGER1_LEVEL,156\r\n"
    "META,STATUS,13\r\n"
    "META,END_INDEX0,15\r\n"
    "META,END_INDEX1,15\r\n"
    "META,DATA_START_INDEX,0\r\n"
    "META,DATA_ROWS,16\r\n"
    "TRIGGER_INDEX,8\r\n"
    "IDX,15,15\r\n"
    "DATA\r\n"
    "0,100\r\n"
    "1,101\r\n"
    "2,102\r\n"
    "3,103\r\n"
    "4,104\r\n"
    "5,105\r\n"
    "6,106\r\n"
    "7,107\r\n"
    "8,108\r\n"
    "9,109\r\n"
    "10,110\r\n"
    "11,111\r\n"
    "12,112\r\n"
    "13,113\r\n"
    "14,114\r\n"
    "15,115\r\n"
    "CAPTURE,END\r\n";

static uint8_t app_usb_wait_ready(void)
{
    uint32_t waited_ms = 0u;

    AppComm_Service();
    if (usb_cdc_initialized == 0u)
    {
        usb_last_error = APP_USB_ERROR_NO_CDC;
        return 0u;
    }

    while (USBUART_CDCIsReady() == 0u)
    {
        if (waited_ms >= APP_USB_TX_TIMEOUT_MS)
        {
            usb_last_error = APP_USB_ERROR_TX_TIMEOUT;
            return 0u;
        }

        CyDelay(1u);
        waited_ms++;

        AppComm_Service();
        if (usb_cdc_initialized == 0u)
        {
            usb_last_error = APP_USB_ERROR_NO_CDC;
            return 0u;
        }
    }

    return 1u;
}

static uint8_t app_usb_send_bytes(const uint8_t *data, uint16_t length)
{
    uint16_t chunk;

    while (length != 0u)
    {
        if (app_usb_wait_ready() == 0u)
        {
            return 0u;
        }

        chunk = (length > APP_USB_TX_CHUNK_SIZE) ? APP_USB_TX_CHUNK_SIZE : length;
        USBUART_PutData(data, chunk);
        data += chunk;
        length -= chunk;
    }

    return 1u;
}

static uint8_t app_usb_send_text(const char *text)
{
    return app_usb_send_bytes((const uint8_t *)text, (uint16_t)strlen(text));
}

void AppComm_Init(void)
{
    DebugUART_Start();
    DebugUART_PutString("START\r\n");
    DebugUART_PutString("DBG,INIT,DebugUART=started\r\n");

    USBUART_Start(0, USBUART_5V_OPERATION);
    DebugUART_PutString("DBG,INIT,USBUART=started\r\n");
    AppComm_Service();
}

void AppComm_Service(void)
{
    uint8_t configuration = USBUART_GetConfiguration();

    if (configuration == 0u)
    {
        if (usb_cdc_initialized != 0u)
        {
            DebugUART_PutString("DBG,USB,disconnected\r\n");
        }
        else if (usb_waiting_logged == 0u)
        {
            DebugUART_PutString("DBG,USB,state=waiting_for_enumeration,configuration=0\r\n");
            usb_waiting_logged = 1u;
        }
        usb_cdc_initialized = 0u;
        usb_cdc_init_failed_logged = 0u;
        return;
    }

    usb_waiting_logged = 0u;
    if (usb_cdc_initialized == 0u)
    {
        if (USBUART_CDC_Init() == USBUART_SUCCESS)
        {
            usb_cdc_initialized = 1u;
            DebugUART_PutString("DBG,USB,cdc=ready\r\n");
        }
        else if (usb_cdc_init_failed_logged == 0u)
        {
            DebugUART_PutString("DBG,USB,state=configuration_seen,cdc_init=failed\r\n");
            usb_cdc_init_failed_logged = 1u;
        }
    }
}

void AppComm_DebugString(const char *text)
{
    DebugUART_PutString(text);
}

uint8_t AppComm_SendUsbFrame(void)
{
    uint8_t result = app_usb_send_text(app_usb_frame);

    if (result != 0u)
    {
        usb_last_error = APP_USB_ERROR_NONE;
        DebugUART_PutString("DBG,USB,send=ok\r\n");
    }
    else if (usb_last_error == APP_USB_ERROR_NO_CDC)
    {
        DebugUART_PutString("DBG,USB,send=failed,reason=cdc_not_ready\r\n");
    }
    else if (usb_last_error == APP_USB_ERROR_TX_TIMEOUT)
    {
        DebugUART_PutString("DBG,USB,send=failed,reason=tx_timeout\r\n");
    }
    else
    {
        DebugUART_PutString("DBG,USB,send=failed\r\n");
    }

    return result;
}

uint8_t AppComm_SendLiveBlock(uint32_t sequence,
                              uint16_t start_index,
                              uint16_t sample_count,
                              uint32_t sample_rate_hz,
                              const uint16_t *channel0,
                              const uint16_t *channel1)
{
    uint8_t header[APP_LIVE_FRAME_HEADER_SIZE];
    uint8_t payload[APP_LIVE_MAX_SAMPLES * 4u];
    uint16_t payload_length;
    uint16_t i;
    uint16_t payload_offset = 0u;

    if ((channel0 == 0) || (channel1 == 0) ||
        (sample_count == 0u) || (sample_count > APP_LIVE_MAX_SAMPLES))
    {
        return 0u;
    }

    payload_length = (uint16_t)(sample_count * 4u);

    header[0] = 'L';
    header[1] = 'I';
    header[2] = 'V';
    header[3] = 'E';
    header[4] = 1u;
    header[5] = 2u;
    header[6] = (uint8_t)(sample_count & 0x00FFu);
    header[7] = (uint8_t)((sample_count >> 8) & 0x00FFu);
    header[8] = (uint8_t)(start_index & 0x00FFu);
    header[9] = (uint8_t)((start_index >> 8) & 0x00FFu);
    header[10] = (uint8_t)(sequence & 0x000000FFUL);
    header[11] = (uint8_t)((sequence >> 8) & 0x000000FFUL);
    header[12] = (uint8_t)((sequence >> 16) & 0x000000FFUL);
    header[13] = (uint8_t)((sequence >> 24) & 0x000000FFUL);
    header[14] = (uint8_t)(sample_rate_hz & 0x000000FFUL);
    header[15] = (uint8_t)((sample_rate_hz >> 8) & 0x000000FFUL);
    header[16] = (uint8_t)((sample_rate_hz >> 16) & 0x000000FFUL);
    header[17] = (uint8_t)((sample_rate_hz >> 24) & 0x000000FFUL);
    header[18] = (uint8_t)(payload_length & 0x00FFu);
    header[19] = (uint8_t)((payload_length >> 8) & 0x00FFu);

    for (i = 0u; i < sample_count; i++)
    {
        payload[payload_offset++] = (uint8_t)(channel0[i] & 0x00FFu);
        payload[payload_offset++] = (uint8_t)((channel0[i] >> 8) & 0x00FFu);
        payload[payload_offset++] = (uint8_t)(channel1[i] & 0x00FFu);
        payload[payload_offset++] = (uint8_t)((channel1[i] >> 8) & 0x00FFu);
    }

    if (app_usb_send_bytes(header, APP_LIVE_FRAME_HEADER_SIZE) == 0u)
    {
        return 0u;
    }

    return app_usb_send_bytes(payload, payload_length);
}

uint8_t AppComm_SendCapture(const AppCommCaptureMetadata *metadata,
                            const uint16_t *channel0,
                            const uint16_t *channel1)
{
    char buf[112];
    uint16_t i;
    uint16_t src;

    if ((metadata == 0) || (channel0 == 0) || (channel1 == 0) ||
        (metadata->sample_count == 0u) ||
        (metadata->data_start_index >= metadata->sample_count))
    {
        return 0u;
    }

    sprintf(buf, "DBG,OUTPUT,start=%u,last0=%u,last1=%u,trigger_index=%u,offset=%d\r\n",
            (unsigned)metadata->data_start_index,
            (unsigned)metadata->end_index0,
            (unsigned)metadata->end_index1,
            (unsigned)metadata->trigger_index,
            (int)metadata->trigger_offset);
    DebugUART_PutString(buf);

    if (metadata->end_index0 != metadata->end_index1)
    {
        DebugUART_PutString("DBG,WARNING,channel_indices_differ\r\n");
    }

    if (app_usb_send_text("CAPTURE,BEGIN\r\n") == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,VERSION,%u\r\n", (unsigned)APP_PROTOCOL_VERSION);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,CAPTURE_NUMBER,%lu\r\n", (unsigned long)metadata->capture_number);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    if (app_usb_send_text("META,FORMAT,CSV16_ADC_COUNTS\r\n") == 0u)
    {
        return 0u;
    }

    if (app_usb_send_text("META,CHANNEL_COUNT,2\r\n") == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,SAMPLE_COUNT,%u\r\n", (unsigned)metadata->sample_count);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,ADC_BITS,%u\r\n", (unsigned)metadata->adc_bits);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,SAMPLE_CLOCK_HZ,%lu\r\n", (unsigned long)metadata->sample_clock_hz);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,TIMER_PERIOD_CYCLES,%lu\r\n",
            (unsigned long)metadata->timer_period_cycles);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,SAMPLE_RATE_HZ,%lu\r\n", (unsigned long)metadata->sample_rate_hz);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,SAMPLE_PERIOD_NS,%lu\r\n",
            (unsigned long)metadata->sample_period_ns);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,PRE_TRIGGER,%u\r\n", (unsigned)metadata->pre_trigger);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,POST_TRIGGER,%u\r\n", (unsigned)metadata->post_trigger);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,TRIGGER_INDEX,%u\r\n", (unsigned)metadata->trigger_index);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,TRIGGER_OFFSET,%d\r\n", (int)metadata->trigger_offset);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,TRIGGER0_LEVEL,%u\r\n", (unsigned)metadata->trigger0_level);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,TRIGGER1_LEVEL,%u\r\n", (unsigned)metadata->trigger1_level);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,STATUS,%u\r\n", (unsigned)metadata->status);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,END_INDEX0,%u\r\n", (unsigned)metadata->end_index0);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,END_INDEX1,%u\r\n", (unsigned)metadata->end_index1);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,DATA_START_INDEX,%u\r\n",
            (unsigned)metadata->data_start_index);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "META,DATA_ROWS,%u\r\n", (unsigned)metadata->sample_count);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "TRIGGER_INDEX,%u\r\n", (unsigned)metadata->trigger_index);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    sprintf(buf, "IDX,%u,%u\r\n",
            (unsigned)metadata->end_index0,
            (unsigned)metadata->end_index1);
    if (app_usb_send_text(buf) == 0u)
    {
        return 0u;
    }

    if (app_usb_send_text("DATA\r\n") == 0u)
    {
        return 0u;
    }

    for (i = 0u; i < metadata->sample_count; i++)
    {
        src = (uint16_t)((metadata->data_start_index + i) % metadata->sample_count);
        sprintf(buf, "%u,%u\r\n", (unsigned)channel0[src], (unsigned)channel1[src]);
        if (app_usb_send_text(buf) == 0u)
        {
            return 0u;
        }
    }

    if (app_usb_send_text("CAPTURE,END\r\n") == 0u)
    {
        return 0u;
    }

    DebugUART_PutString("DBG,OUTPUT,complete\r\n");
    return 1u;
}
