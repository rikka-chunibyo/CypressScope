#include "feature_registry.h"
#include "feature_config.h"

#if ACTIVE_FEATURE_MODE == FEATURE_MODE_USB
#include "usb/usb.h"
#elif (ACTIVE_FEATURE_MODE == FEATURE_MODE_OSCILLOSCOPE) || \
      (ACTIVE_FEATURE_MODE == FEATURE_MODE_LIVE_FEED)
#include "oscilloscope/oscilloscope.h"
#endif

const FeatureModule *FeatureRegistry_GetActive(void)
{
#if ACTIVE_FEATURE_MODE == FEATURE_MODE_USB
    return Usb_GetModule();
#elif ACTIVE_FEATURE_MODE == FEATURE_MODE_OSCILLOSCOPE
    return Oscilloscope_GetModule();
#elif ACTIVE_FEATURE_MODE == FEATURE_MODE_LIVE_FEED
    return Oscilloscope_GetLiveModule();
#else
#error "ACTIVE_FEATURE_MODE has no registered feature module"
#endif
}
