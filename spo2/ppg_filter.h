#ifndef PPG_FILTER_H
#define PPG_FILTER_H

#include "ppg_analyse.h"
#include "filter.h"

void initPpgFilter(void);
int32_t  ppgFilter(SPO2Parameter *parameter);
int32_t  judgePpgDataQuality(int32_t value);

#endif //PPG_FILTER_H
