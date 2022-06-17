#ifndef FILTER_H
#define FILTER_H
#include <stdint.h>

#define FIR_FILTER_LENGTH    33     // fir

void init_filter(void);
int32_t filter(int32_t data);

int32_t median_s32(int32_t *arr, uint8_t arr_len);
int32_t mean_s32(int32_t *arr, uint8_t start, uint8_t end);
int32_t truncated_mean_s32(int32_t *arr, uint8_t arr_len, uint8_t start, uint8_t end);

#endif // FILTER_H
