#include "project.h"

#include "../../app_comm.h"
#include "../feature_config.h"
#include "../oscilloscope/oscilloscope.h"
#include "usb.h"

static void usb_init(void)
{
    Oscilloscope_ReportConfig();
    AppComm_DebugString("DBG,USB,mode=constant_frame\r\n");
}

static void usb_service(void)
{
    AppComm_SendUsbFrame();
    CyDelay(FEATURE_USB_INTERVAL_MS);
}

static void usb_stop(void)
{
}

const FeatureModule *Usb_GetModule(void)
{
    static const FeatureModule module =
    {
        "usb",
        usb_init,
        usb_stop,
        usb_service
    };

    return &module;
}
