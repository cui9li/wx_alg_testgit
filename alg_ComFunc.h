
#ifndef ALG_COMFUNC_H
#define ALG_COMFUNC_H

#include <stdint.h>

#define 	ABS(x)			( (x)<0?( -(x) ) : (x) )

#define DECLARE_MEAN_FUN(type)\
	extern type type##_mean(type Data[], type NL, type NU);
DECLARE_MEAN_FUN(int16_t);   //声明类型
DECLARE_MEAN_FUN(uint16_t);
DECLARE_MEAN_FUN(uint8_t);

#define DECLARE_STD_FUN(type)\
	extern type type##_std(type Data[], type NL, type NU);
DECLARE_STD_FUN(int16_t);   //声明
DECLARE_STD_FUN(uint16_t);

/* 降序排列 函数模板 实例化时采用两种方式 其中之一 */
#define DECLARE_DESCEND_FUN( type )\
	extern type* type##desfun( type Data[] );
#define call_desfun(type) type##desfun
DECLARE_DESCEND_FUN(uint8_t);  //声明类型
DECLARE_DESCEND_FUN(uint16_t);


void quick_sort(int16_t s[], uint8_t l, uint8_t r);

void fifopush_u8(uint8_t data, uint8_t s[], uint16_t size);
void fifopush_s16(int16_t data, int16_t s[], uint8_t size);
void fifopush_f(float data, float s[], uint8_t size);
void fifopush_int32(int32_t data, int32_t s[], uint8_t size);

uint8_t victor_filer_u8(uint8_t s[], uint8_t size, uint8_t r, float *pscatter);
int16_t victor_filer_s16(int16_t s[], uint8_t size, uint8_t r, float *pscatter);
float scatter_avg_s16(int16_t s[], uint8_t size);


#endif // !ALG_COMFUNC_H
