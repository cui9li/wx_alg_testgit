#ifndef DEBUG_H
#define DEBUG_H

void debug_init(void);
void close_file(void);
int startup_file(char *data_name);

//#define output_flag
#ifdef output_flag
#include <stdio.h>
extern FILE *pfile_grnfilter;
#define debug_printf(...)  do{fprintf(pfile_grnfilter, ##__VA_ARGS__);}while(0)
#else
#define debug_printf(...)
#endif

////==================STARTS========================
////#define OUT_HR
//#ifdef OUT_HR
//#include <stdio.h>
//extern FILE *pfile_hr;
//#define hr_printf(...)  do{fprintf(pfile_hr, ##__VA_ARGS__);}while(0)
//#endif
////===================END==========================

//==================START===========================
//#define HR_STATS
#ifdef HR_STATS
#include "../alg_ComFunc.h"

#define ARR_LEN      500

extern uint8_t HR_N[ARR_LEN];
extern uint8_t hr_median;
extern uint8_t hr_mean ;
extern float hr_std;
extern uint8_t hr_len;
extern uint16_t file_len;
extern uint8_t len_hrAbnormal;

extern float RR_N[ARR_LEN] ;
extern float rr_mean ;
extern float rr_std ;
extern uint8_t len_rrIn10Mean ;   // rr值在均值+-10%以内的个数
extern uint8_t len_rrIn20Mean ;   // rr值在均值+-20%以内的个数
extern uint8_t len_rrIn30Mean ;   // rr值在均值+-30%以内的个数

extern uint8_t SPO_N[ARR_LEN];
extern uint8_t spo_len;
extern uint8_t spo_median;
extern uint8_t spo_mean;
extern float spo_std;
extern float spo_IQR;            // 四分位数间距IQR  = Q3-Q1
extern uint8_t len_spoAbnormal;  // 血氧值异常(<95)的个数


void init_parameters(void);
void cal_hr_statistics(void);
void cal_spo_statistics(void);

#define debug_push(a,arr) do{fifopush_u8(a,arr,ARR_LEN);}while(0)
#else
#define debug_push(...)
#endif // HR_STATS

void init_parameters(void);
void cal_hr_statistics(void);

//====================END=========================

#endif // !DEBUG_MY_H
