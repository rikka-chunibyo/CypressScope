#include "project.h"

#include "../../app_comm.h"
#include "../feature_config.h"
#include "../oscilloscope/oscilloscope.h"
#include "usb_test.h"

static void usb_test_init(void)
{
    Oscilloscope_ReportConfig();
    AppComm_DebugString("DBG,USBTEST,mode=constant_frame\r\n");
}

static void usb_test_service(void)
{
    AppComm_SendTestFrame();
    CyDelay(FEATURE_USB_TEST_INTERVAL_MS);
}

const FeatureModule *UsbTest_GetModule(void)
{
    static const FeatureModule module =
    {
        "usb_test",
        usb_test_init,
        usb_test_service
    };

    return &module;
}
