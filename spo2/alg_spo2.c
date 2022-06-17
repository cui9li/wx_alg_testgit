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
*  2021-12-13 |1.0       |wx_John         |
*  2022-04-14 |1.1       |wx_John         |1.green_ppg 先低通，然后去飘移（均值滤波）
* ----------------------------------------------------------------
*****************************************************************************/


//#include"adpd_spo2.h"

#include "alg_spo2.h"
#include <math.h>

#include "ppg_filter.h"
#include "../alg_ComFunc.h"
#include "detect_pr_slop.h"

//////////debug////////////////////
//#include <stdio.h>
//#include "main.h"
//#include "debug.h"
///////end debug////////////////

/******** SpO2 parameters ***********/

//for void dc_removal()
#define ALPHA (0.95f)  //dc filter alpha value

//for void cal_spo2()
#define SPO2_SAMPLE_RATE              50      // 采样率
#define VALUE_ARR_LEN                 4       // 缓存结果的数组大小 对应数组bpm_val_arr[] spo_val_arr[]
#define RESET_SPO2_EVERY_N_PULSES     4

// for balance_red_ir_dc()
#define RED_LED_CURRENT_ADJUSTMENT_MS 25      // 500ms sample dot
#define MAGIC_DIFF                    50000

// for cal_spo2()
#define SMOOTH_N                      51     // 均值滤波器的阶数
int32_t smooth_buf[SMOOTH_N] = { 0 };
uint32_t sum_smooth_buf = 0;                 // sum(buf)

uint16_t step_cnt = 0;                       // 采样点计数（每个采样点调用一次算法）
int32_t detect_pulse_cnt = 0;                // bpm>0 的心拍个数

static float sum_of_squares_red_ac = 0;      //平方和
static float sum_of_squares_ir_ac = 0;
static int32_t sum_cnt = 0;

//dc_removal()
static float red_dc;
static float red_value;
static float ir_dc;
static float ir_value;

uint8_t spo2_val_arr[VALUE_ARR_LEN] = { 0 };     //存储瞬时spo值
int16_t bpm_val_arr[VALUE_ARR_LEN] = { 0 };      //存储瞬时bpm*100值

//void balance_red_ir_dc()
static int32_t balance_dc_cnt = 0;
static int32_t red_led_current;

//static int32_t sum_bpm = 0;    //经过后期处理后 每个点处不为0的raw_bpm的累加，用bpm_cnt计数，每包（13.25个点）清零
//static int32_t bpm_cnt = 0;
//static int32_t sum_spo2 = 0;
//static uint8_t beat_cnt = 0;   //每个detectPulseBySlop计算的周期计数


/**********************************************************************************
* 函数名称：dc_removal()
* 功能描述：
* 参数说明：
* 返回说明：
* 说    明：
**********************************************************************************/
void dc_removal(float raw_data, float *prev_dc, float *prev_value)
{
	float pre_data = *prev_dc;
	*prev_dc = (raw_data + (ALPHA)* pre_data);
	*prev_value = (*prev_dc) - pre_data;
}


/**********************************************************************************
* 函数名称：data_sum_init()
* 功能描述：初始化算法里面的变量
* 参数说明：
* 返回说明：
* 说    明：
**********************************************************************************/
void data_sum_init(void)
{
	sum_of_squares_red_ac = 0;
	sum_of_squares_ir_ac = 0;
	sum_cnt = 0;
}


/**********************************************************************************
* 函数名称：spo2_init()
* 功能描述：初始化算法里面的变量
* 参数说明：
* 返回说明：
* 说    明：
**********************************************************************************/
void spo2_init(void)
{
	uint8_t i;

	initDetectPulseBySlop(PPG_SAMPLE_RATE);
	initPpgFilter();
	red_dc = 0;
	red_value = 0;
	ir_dc = 0;
	ir_value = 0;
	data_sum_init();
	detect_pulse_cnt = 0;
	step_cnt = 0;

	for (i = 0; i < VALUE_ARR_LEN; i++) {
		spo2_val_arr[i] = 0;
		bpm_val_arr[i] = 0;
	}

	sum_smooth_buf = 0;
	for (i = 0; i < SMOOTH_N; i++)
	{
		smooth_buf[i] = 0;
	}

}

/**********************************************************************************
* 函数名称：balance_red_ir_dc()
* 功能描述：
* 参数说明：
* 返回说明：
* 说    明：red_led_current 并没有用到 该函数用意是什么？
**********************************************************************************/

void balance_red_ir_dc(void)
{
	balance_dc_cnt++;

	if (balance_dc_cnt > RED_LED_CURRENT_ADJUSTMENT_MS)
	{
		if ((red_dc - ir_dc) < MAGIC_DIFF)
		{
			red_led_current--;
		}
		else if ((ir_dc - red_dc) > MAGIC_DIFF)
		{
			red_led_current++;
		}

		balance_dc_cnt = 0;
	}
}


/**********************************************************************************
* 函数名称：cal_spo2()
* 功能描述：计算spo2
* 参数说明：每个采样点的red ir green
* 返回说明：spo2_value:血氧值
* 说    明：
**********************************************************************************/
void cal_spo2(int32_t red_data, int32_t ir_data, int32_t green_data, uint8_t *spo_value,uint8_t *bpm_value)
{

	SPO2Parameter parameter;
	uint8_t spo2_value = 0;
	int16_t raw_bpm = 0;              // raw_bpm  = bpm*100
	//uint16_t delay_n = 0;
	float spo_scatter = 0;
	float bpm_scatter = 0;

	*spo_value = 0;
	*bpm_value = 0;

	step_cnt++;
	/*---1-lowpass*---------*/
	parameter.sensorValue = green_data;
	ppgFilter(&parameter); // green_ppg

	/* 2- 去掉直流分量-----*/
	if (step_cnt > FIR_FILTER_LENGTH-1 && step_cnt<FIR_FILTER_LENGTH+SMOOTH_N-2)
	{
		fifopush_int32(parameter.sensorValue, smooth_buf, SMOOTH_N);
	}
	if (step_cnt == FIR_FILTER_LENGTH + SMOOTH_N - 2)
	{
		fifopush_int32(parameter.sensorValue, smooth_buf, SMOOTH_N);
		for (uint8_t i = 0; i < SMOOTH_N; i++)
		{
			sum_smooth_buf += smooth_buf[i];
		}
	}
	if (step_cnt > FIR_FILTER_LENGTH + SMOOTH_N - 2)
	{
		sum_smooth_buf = sum_smooth_buf + parameter.sensorValue - smooth_buf[SMOOTH_N - 1];
		fifopush_int32(parameter.sensorValue, smooth_buf, SMOOTH_N);
		uint8_t smooth_delay = SMOOTH_N / 2;
		parameter.sensorValue = smooth_buf[smooth_delay] - sum_smooth_buf / SMOOTH_N;      //去掉低频

		//debug_printf("%d\n", parameter.sensorValue);
	}

	if (step_cnt > FIR_FILTER_LENGTH + SMOOTH_N - 2)     //开始计算hr spo
	{
		raw_bpm = detectPulseBySlop(&parameter);          // raw_bpm  = bpm*100

		//问题一：作用？分离直流和交流？未解决
		dc_removal((float)red_data, &red_dc, &red_value); //???
		dc_removal((float)ir_data, &ir_dc, &ir_value);

		//每4个周期清零重新计算，
		sum_of_squares_red_ac += (red_value) * (red_value);
		sum_of_squares_ir_ac += (ir_value) * (ir_value);
		sum_cnt++;    // 4个周期内的点数计算

		// 检测到周期性的峰值点之后5个点，该周期有效能计算出raw_bpm，此时才计算spo2_value
		if ( raw_bpm > 0 && sum_cnt > 0)
		{
			//beat_cnt++;             //每个detectPulseBySlop计算的周期计数，每包（13.25个点）开始时清零
			detect_pulse_cnt++;       // 心拍个数，测量过程中不清零

			//问题2：r的计算方式，参考依据？拟合曲线用什么数据拟合的？每次用的点数是不固定的，我认为固定窗长的更好，待修改
			float ratio = log(sqrt(sum_of_squares_red_ac / sum_cnt)) / log(sqrt(sum_of_squares_ir_ac / sum_cnt));
			spo2_value = (uint8_t)(118.0f - (18.0f) * ratio);

			if (spo2_value >= 100)
				spo2_value = 98;
			if (spo2_value < 70)
				spo2_value = 0;      //测量范围：[70,99]

			fifopush_s16(raw_bpm, bpm_val_arr, VALUE_ARR_LEN);
			if (detect_pulse_cnt > VALUE_ARR_LEN - 1)
			{
				*bpm_value = (victor_filer_s16(bpm_val_arr, VALUE_ARR_LEN, 80, &bpm_scatter) + 50) / 100;
			}
			else
			{
				*bpm_value = (raw_bpm + 50) / 100;
			}
			//printf("%d\t%d\t%d\n", step_cnt, raw_bpm, *bpm_value);

			if (spo2_value >= 70 && spo2_value<100)
			{
				fifopush_u8(spo2_value, spo2_val_arr, VALUE_ARR_LEN);
			}
			if (detect_pulse_cnt > VALUE_ARR_LEN - 1)
			{
				*spo_value = victor_filer_u8(spo2_val_arr, VALUE_ARR_LEN, 80, &spo_scatter);
			}
			else
			{
				*spo_value = spo2_value;
			}

			//beat_cnt++;              //放在这里好理解，每个detectPulseBySlop计算的周期计数，每包（13.25个点）开始时清零
			//sum_spo2 += spo2_value;  // 对于spo2_value==0的情况呢，beat_cnt 也计数了，spo2_value会偏小，不太合理

			data_sum_init();           // 更改为每个周期清零
			//if (detect_pulse_cnt % RESET_SPO2_EVERY_N_PULSES == 0)
			//data_sum_init();
			//printf("\t\t%d\t%d\n", step_cnt, *bpm_value);

			//debug_push(*bpm_value,HR_N);
			//debug_push(*spo_value, SPO_N);
		}

		////对raw_bpm的后期处理
		//raw_bpm = calulatePr(&raw_bpm, 1);
		//if (raw_bpm > 0) {
		//	sum_bpm += raw_bpm;   //经过后期处理后 每个点处不为0的raw_bpm的累加，每包（13.25个点）清零
		//	bpm_cnt++;            //经过后期处理后 每个点处不为0的raw_bpm的计数，每包（13.25个点）清零
		//}

		//balance_red_ir_dc();   // 暂时没有用到，先注销2012-12-12

		//return spo2_value;

	}


}


