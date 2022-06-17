#ifndef JUDGE_ISWEARING_H
#define JUDGE_ISWEARING_H

#include <stdint.h>

void wear_init(uint8_t wear_init_flag,uint8_t version_num);
uint8_t judge_Iswearing(int32_t ir_data,int32_t red_data,int32_t green_data);
uint8_t judge_wear_status(void);

#endif
