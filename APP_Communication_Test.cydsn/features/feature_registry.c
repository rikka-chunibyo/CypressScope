#include "feature_registry.h"
#include "usb/usb.h"
#include "oscilloscope/oscilloscope.h"

const FeatureModule *FeatureRegistry_Get(FeatureMode mode)
{
    switch (mode)
    {
        case FEATURE_MODE_USB:
            return Usb_GetModule();

        case FEATURE_MODE_OSCILLOSCOPE:
            return Oscilloscope_GetModule();

        case FEATURE_MODE_LIVE_FEED:
            return Oscilloscope_GetLiveModule();

        default:
            return 0;
    }
}
