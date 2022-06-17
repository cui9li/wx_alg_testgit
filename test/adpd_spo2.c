/*****************************************************************************
* @file         adpd_spo2.c
* @author       sky
* @version      V0.1
* @date         2020-12-09
* @brief
*--------------------------------------------------------------------*
* Change History :
* <Data>      |<Version> |<Author>       | <Description>
*--------------------------------------------------------------------*

*****************************************************************************/

#include "adpd_spo2.h"
#include <math.h>
#include "../spo2/ppg_analyse.h"
#include "../spo2/alg_spo2.h"
#include "../spo2/ppg_filter.h" // judgePpgDataQuality() need

#define LED_NUMS                3    //3路光 red ir grn
#define SPO2_SAMPLE_TIME        213  // 采样时间？213s后停止采集？
#define SPO2_SEND_START_VALUE   33

//static int16_t sum_bpm = 0;        //经过后期处理后 每个点处不为0的raw_bpm的累加，用bpm_cnt计数，每包（13.25个点）清零
//static int16_t bpm_cnt = 0;

//static int16_t sum_spo2 = 0;
//static uint8_t beat_cnt = 0;       //每个detectPulseBySlop计算的周期计数

static int16_t send_cnt = 0;
static uint8_t save_ctr;
static uint8_t send_raw_data_flag = 0;

#define SPO2_DATA_BYTES (4 * LED_NUMS)
static uint8_t byte_cnt;
//static uint8_t byte_buf[SPO2_DATA_BYTES];
static int32_t spo2_data[LED_NUMS];
//static uint8_t check_times;

/********************************************************************************
* 函    数：  adpd_spo2_init()
* 功能描述:   初始化
* 参数说明：
* 版    本：
* 返回说明：
* 说    明：
***********************************************************************************/
void adpd_spo2_init(uint8_t times, uint8_t raw_flag)
{

	//sample_timeing_int();
	spo2_init();

	byte_cnt = 0;
	send_cnt = 0;
	save_ctr = 0;
	send_raw_data_flag = raw_flag;


	//if (IS_WEAR_CHECK_ON)
	//	check_times = times;
	//else
	//	check_times = 0;

}

/******************************************************************************
* 函    数：  adpd_spo2_data_deal()
* 功能描述:   计算小包（约14个采样点）的spo2
* 参数说明：  size：包内采样次数  pack_data[3*size]:red ir green的数据 rlt一包输出一个结果，距离传输点或者保存点最近的结果
* 版    本：
* 返回说明：
* 说    明：  软件与算法接口
***********************************************************************************/

void adpd_spo2_data_deal(uint16_t size,int32_t * pack_data, uint8_t *spo2_ratio,uint8_t *mean_bpm)
{

	uint8_t spo2_val;
	uint8_t bpm_val;
	int32_t *p;
	p = pack_data;

	*spo2_ratio = 0;
	*mean_bpm = 0;

	spo2_val = 0;
	bpm_val = 0;

	while (size) {
		// 每个采样点的red ir green,验证通过
		spo2_data[0] = *p;
		spo2_data[1] = *(p+1);
		spo2_data[2] = *(p+2);
		p+=3;

		//*****每个采样点调用一次算法计算******
		cal_spo2(spo2_data[0], spo2_data[1], spo2_data[2],&spo2_val,&bpm_val);// red ir green
		// 输出最近的有效值
		if (spo2_val) {
			*spo2_ratio = spo2_val;
		}
		if (bpm_val) {
			*mean_bpm = bpm_val;
		}
		//***************************
		size -= 1;

	}

	//// 全局变量sum_spo beat_cnt sum_bpm bpm_cnt 都是在apdp_cal_spo2()计算的
	//if (beat_cnt != 0)                       //beat_cnt 在 函数apdp_cal_spo2()计算
	//	spo2_ratio = sum_spo2 / beat_cnt;    //一包 均值
	//else
	//	spo2_ratio = 0;
	//if (bpm_cnt != 0)
	//	mean_bpm = sum_bpm / bpm_cnt;   //一包中
	//else
	//	mean_bpm = 0;

}
