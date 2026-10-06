#include "feature_registry.h"
#include "feature_config.h"

#if ACTIVE_FEATURE_MODE == FEATURE_MODE_USB_TEST
#include "usb_test/usb_test.h"
#elif ACTIVE_FEATURE_MODE == FEATURE_MODE_OSCILLOSCOPE
#include "oscilloscope/oscilloscope.h"
#endif

const FeatureModule *FeatureRegistry_GetActive(void)
{
#if ACTIVE_FEATURE_MODE == FEATURE_MODE_USB_TEST
    return UsbTest_GetModule();
#elif ACTIVE_FEATURE_MODE == FEATURE_MODE_OSCILLOSCOPE
    return Oscilloscope_GetModule();
#else
#error "ACTIVE_FEATURE_MODE has no registered feature module"
#endif
}
