
#ifndef VIEW_MAIN_H
#define VIEW_MAIN_H

#include <stdint.h>

#define  Fs				    50                  //采样率
#define  ARR_LEN			5			        //算法调用数组长度
#define  DIS_LEN_THREE_AXIS			9000		//画图 数据长度 三轴之和
#define  DIS_LEN_SINGLE			    3000		//单轴长度

#define  LEN_ARR_UPFLAG	     (DIS_LEN_SINGLE/ARR_LEN)    //Up_flag标志位存储数组长度 只限DISLEN_O长度内
#define  LEN_ARR_BEATFLAG	 LEN_ARR_UPFLAG	 //Beat_flag标志位存储数组长度
#define  LEN_ARR_AFTERFLAG	 LEN_ARR_UPFLAG	 //After_flag

extern int16_t  Cnt01s;
extern int16_t  CntNs;

void view_main(void);

#endif // VIEW_MAIN_H
