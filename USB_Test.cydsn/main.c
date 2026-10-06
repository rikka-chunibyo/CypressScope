#include "project.h"

#include "app_comm.h"
#include "features/feature_registry.h"

int main(void)
{
    const FeatureModule *feature;

    CyGlobalIntEnable;

    AppComm_Init();
    feature = FeatureRegistry_GetActive();
    feature->init();

    for (;;)
    {
        AppComm_Service();
        feature->service();
    }
}
