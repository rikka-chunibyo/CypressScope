#ifndef OSCILLOSCOPE_H
#define OSCILLOSCOPE_H

#include <stdint.h>

#include "../feature.h"

const FeatureModule *Oscilloscope_GetModule(void);
const FeatureModule *Oscilloscope_GetLiveModule(void);
void Oscilloscope_ReportConfig(void);
void Oscilloscope_SetTriggerLevel(uint8_t channel, uint8_t level);
uint8_t Oscilloscope_GetTriggerLevel(uint8_t channel);

#endif
