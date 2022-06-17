
#ifndef ADPD_SPO2_H
#define ADPD_SPO2_H
#include <stdint.h>

void adpd_spo2_data_deal(uint16_t size, int32_t * pack_data, uint8_t *spo2, uint8_t *bpm);
void adpd_spo2_init(uint8_t times, uint8_t raw_flag);

#endif // !ADPD_SPO2_H
