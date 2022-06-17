#ifndef DETECT_PR_SLOP_H
#define DETECT_PR_SLOP_H

#include <stdint.h>
#include "ppg_analyse.h"

#define PPG_SAMPLE_RATE    50


void initDetectPulseBySlop(int32_t sampleRate);
int16_t detectPulseBySlop(SPO2Parameter *parameter);

int32_t calulatePr(const int32_t *rawPr, int32_t dataSize);
int32_t judgeHrQuality(int32_t hr);

#endif //DETECT_PR_SLOP_H




