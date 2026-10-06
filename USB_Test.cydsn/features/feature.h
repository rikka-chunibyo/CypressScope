#ifndef FEATURE_H
#define FEATURE_H

typedef struct
{
    const char *name;
    void (*init)(void);
    void (*service)(void);
} FeatureModule;

#endif
