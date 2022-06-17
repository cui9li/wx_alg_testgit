#ifndef PPG_ANALYSE_H
#define PPG_ANALYSE_H

#include <stdint.h>

#define BASE_PR          60.0

typedef struct _SPO2Parameter
{
    int32_t sensorValue;
    int32_t xPosition;
    int32_t yPosition;
    int32_t zPosition;
} SPO2Parameter;



#endif // PPG_ANAlYSE_H
