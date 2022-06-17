
#ifndef ALGSMARTVIEW_H
#define ALGSMARTVIEW_H

#include <stdint.h>

extern uint8_t Beat_flag; /* 软件置位Beat_flag： 当切换屏幕的时候 Beat_flag = 1 */
extern uint8_t Up_flag;   

void upclc(void);
void clcbeat(void);    /* Up_Hand()函数相关变量清零 -- 初始化 */
void clcSWatch(void);  /* shakeWatch()函数相关变量清零  -- 初始化 */ 
void Up_Hand(int16_t *data_x, int16_t *data_y, int16_t *data_z, uint8_t time); /* 接口函数 */

#endif 
