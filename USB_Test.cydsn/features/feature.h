#ifndef FEATURE_H
#define FEATURE_H

typedef enum
{
    FEATURE_MODE_USB = 1u,
    FEATURE_MODE_OSCILLOSCOPE = 2u,
    FEATURE_MODE_LOGIC_ANALYZER = 3u,
    FEATURE_MODE_LIVE_FEED = 4u
} FeatureMode;

typedef struct
{
    const char *name;
    void (*init)(void);
    void (*stop)(void);
    void (*service)(void);
} FeatureModule;

#endif
