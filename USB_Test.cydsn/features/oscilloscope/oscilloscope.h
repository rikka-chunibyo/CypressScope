#ifndef OSCILLOSCOPE_H
#define OSCILLOSCOPE_H

#include "../feature.h"

const FeatureModule *Oscilloscope_GetModule(void);
const FeatureModule *Oscilloscope_GetLiveModule(void);
void Oscilloscope_ReportConfig(void);

#endif
