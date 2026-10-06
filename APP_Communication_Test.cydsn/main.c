#include "project.h"

#include "app_comm.h"
#include "features/feature_registry.h"

int main(void)
{
    const FeatureModule *feature;
    FeatureMode active_mode = FEATURE_MODE_USB;
    FeatureMode requested_mode;

    CyGlobalIntEnable;

    AppComm_Init();
    feature = FeatureRegistry_Get(active_mode);
    feature->init();

    for (;;)
    {
        AppComm_Service();

        if (AppComm_TakeModeRequest(&requested_mode) != 0u &&
            requested_mode != active_mode)
        {
            const FeatureModule *next_feature = FeatureRegistry_Get(requested_mode);

            if (next_feature == 0)
            {
                AppComm_DebugString("DBG,FEATURE,mode=unsupported\r\n");
            }
            else
            {
                feature->stop();
                feature = next_feature;
                active_mode = requested_mode;
                feature->init();
                AppComm_DebugString("DBG,FEATURE,mode=switched\r\n");
            }
        }

        feature->service();
    }
}
