
#ifndef ALG_SPO2_H
#define ALG_SPO2_H

#include <stdint.h>

void spo2_init(void);
void cal_spo2(int32_t red_data, int32_t ir_data, int32_t green_data, uint8_t* spo2_value, uint8_t *bpm_value);

#endif // !ADPD_SPO2_H
