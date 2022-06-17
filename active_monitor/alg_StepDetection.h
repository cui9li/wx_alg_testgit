/*******************************************************************************
Copyright (C), 2022, ANDON HEALTH Co., Ltd.
FileName     : alg_StepDetection.h
Author       :         
Version      : REV1.0
CreatDate    : 2022-05-07
Description  :             
History      :         
<Author>  <Modify date>   <Version >    <Modify content>

*******************************************************************************/
#ifndef ALG_STEPDETECTION_H
#define ALG_STEPDETECTION_H

#include <stdint.h>

extern uint32_t steps_display;

void Alg_Clear_RAM(void);
void alg_AM(int16_t x, int16_t y, int16_t z, uint32_t *current_steps);


#endif /* alg_AM */
