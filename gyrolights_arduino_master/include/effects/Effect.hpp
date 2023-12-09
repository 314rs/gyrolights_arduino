#include "stdint.h"
#include "../projectConfig.h"

class Effect
{
protected:
    const static uint8_t maxBrightness = 10;
public:
    virtual void init() = 0;
    virtual void loop() = 0;
};

