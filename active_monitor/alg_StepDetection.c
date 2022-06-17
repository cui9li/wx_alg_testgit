/*****************************************************************************
* @file       alg_StepDetection.c
* @author
* @version    V1.0
* @date       2022-05-07
* @brief      计步，每秒计算一次，50hz
*--------------------------------------------------------------------*
* Change History :
* <Data> | <Version> | <Author>  | <Description>
*--------------------------------------------------------------------*
*
* ----------------------------------------------------------------
*****************************************************************************/
#include "alg_StepDetection.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
//#include "alg_AutoSleep.h"
//#include "active_main.h"


#define FS                            50    // 采样率
#define DET_STEPS_LENGTH              6
#define DET_LENGTH                    11
#define FILTERED_DATA_LEN             83
#define COEFF_FIR_LEN                 33
#define WINDOW_LEN                    50
#define STEP_THRSHD                   540000//530000
#define STEP_THRSHD_WRIST             470000//470000
#define STRENGTH_ALG                  25000

//peason infor
#define Weight_all    60
#define Height        165
#define Age           30
#define Sex           0x01
#define STEPlength    70
//---------------------------------------------------------


int8_t mode_flag = 1;     //mode_flag=0为腰模式；1FINESSE_MODE为手腕模式；2为睡眠模式
uint8_t point_cnt = 0;    // 每个窗口（1s）的采样点计数

uint32_t steps_display = 0;                        //显示当前总步数
int16_t xyz_sqrt_data[FILTERED_DATA_LEN] = {0};    // 加缓冲的和加速度数组，缓冲长度：fir阶数
int32_t FilterResult[FILTERED_DATA_LEN] = {0};
int32_t data_pre_alg[10] = { 500000 };

int8_t eMaxIndex_cnt = 0;                 // eMaxIndex_all的有效长度
int16_t eMaxIndex_1s[5]= {100};           // 1s内的极大值下标
int32_t extrMax_1s[5] = {0};              // 1s内的极大值 幅值
int32_t eMaxIndex_all[DET_LENGTH] = {0};
int32_t extrMax_all[DET_LENGTH] = {0};

uint32_t threshold = STEP_THRSHD;         //可以调整
uint32_t steps_sum = 0;                   // 连续运动的总步数

float avg_points_of_pp = 0;               // 峰峰间距的均值，单位：采样点个数 avg_points_of_2peaks
//int8_t stop_flag = 0x01;                // 停止运动标志（连续2s没有步数，运动停止）

int8_t step_start = 0;
int8_t stop_time = 0;                    // 开始运动后，每秒的steps为0的计数 ，(若连续2s没有步数，运动停止)
int8_t nostep_cnt = 0;                   // 没有步数的1s计数
int16_t sec_alg = 0;                     // 1s内有步数的时间计数

uint8_t temp_cnt_alg = 0;
uint8_t Estep_num = 0;
uint8_t sec_wait_alg = 0;                // 判断开始运动（>6步）所需要的时间

//Float_Data Distance;
double calorie_sum = 0;
uint16_t calorie_display = 0x00;   // 显示的总卡路里


/**********************************************************************************
* 函数名称：
* 功能描述：滤波
* 版    本： v1.0
* 参数说明：data[]，原始数据；filterCoeff[]，采用的滤波器；data_length,原始数据长度； filter_length:滤波器长度
* 返回说明：        无
* 说    明：采用卷积的方式并舍弃最后filter_length-1个点
**********************************************************************************/

void Filter(int16_t data[],int8_t filterCoeff[],int16_t data_length,int8_t filter_length)
{

	int16_t n;
	int16_t k;
	int32_t rsl_mpy;

	for(n=0; n < data_length; n ++)
	{
		FilterResult[n] = 0;
		for(k=0; (n >= k) && (k<filter_length);k++)
		{
			rsl_mpy = data[n-k]*filterCoeff[k];
			FilterResult[n] += rsl_mpy;
		}
	}
}

/* 停止运动时变量清零 */
void steps_stop_clc(void)
{
	uint8_t i = 0;

	threshold = STEP_THRSHD_WRIST;
	//step_start = 0;
	steps_sum = 0;
	sec_alg = 0;   //sec_alg清零

	eMaxIndex_cnt = 0;
	for (i = 0; i < DET_LENGTH; i++)
	{
		eMaxIndex_all[i] = 0;
		extrMax_all[i] = 0;
	}
	//calc_calorie(0,sec_wait_alg,0);
	sec_wait_alg = 0;

}

/**********************************************************************************
*功能描述:         极大值检测程序
*参数说明：        eMaxIndex极大值对应的时间点
*返回说明：        无
*通用变量：        无
*说    明：
**********************************************************************************/
void find_peaks( int32_t data[])
{
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t m = 0;

	int8_t sign_data[49];
	int8_t diff_sign_data[48];
	int16_t eMaxIndex[5] = { 0 };   //该1s内峰值点位置

	for (i = 0; i < 5; i++)         // 当前1s的eMaxIndex_1s[] extrMax_1s[]初始化
	{
		eMaxIndex_1s[i] = 100;      //清零eMaxIndex_1s
		extrMax_1s[i] = 0;
	}

	//极大值检测
	for (i=0;i < 49; i ++)
	{
		if(data[i+1] > data[i])
			sign_data[i] = 1;
		else if(data[i+1] == data[i])
			sign_data[i] = 0;
		else
			sign_data[i] = -1;
	}
	for (i = 0;i < 48;i ++)
	{
		diff_sign_data[i] = sign_data[i+1] - sign_data[i];
		if(diff_sign_data[i] == -2)
		{
			eMaxIndex[j] = i+1;
			j ++;
		}
	}

	if((data_pre_alg[9] > data_pre_alg[8]) && (data_pre_alg[9] > data[0])) /*判断上一秒的最后一个点是否峰值点，若是，继续把该点与前后第8个点进行比较，看是否大于阈值*/
	{
		if(((data_pre_alg[9] - data_pre_alg[1] > STRENGTH_ALG) || (data_pre_alg[9] - data[7] > STRENGTH_ALG)) && (data_pre_alg[9] > STEP_THRSHD_WRIST))
		{
			eMaxIndex_1s[0] = 0;
			extrMax_1s[0] = data[0];
			m ++;
		}
	}
	else            //否，判断data[0]是不是峰值点
	{
		if((data[0] > data_pre_alg[9]) && (data[0] > data[1]))
		{
			if(((data[0] - data[8] > STRENGTH_ALG) || (data[0] - data_pre_alg[2] > STRENGTH_ALG)) && (data[0] > STEP_THRSHD_WRIST))
			{
				eMaxIndex_1s[0] = 0;
				extrMax_1s[0] = data[0];
				m ++;
			}
		}
	}

	/* 对初次得到的峰值点位置eMaxIndex进行2次判断，看是否符合峰值点要求（主要从幅值上）*/
	for (i = 0; i < j; i ++)
	{
		if (eMaxIndex[i]+8 > 49) //处于右边缘的判断
		{
			if((data[eMaxIndex[i]] - data[eMaxIndex[i]-8] > STRENGTH_ALG) && (data[eMaxIndex[i]] - data[49] > 0) && (data[eMaxIndex[i]] > STEP_THRSHD_WRIST))//
			{
				eMaxIndex_1s[m] = eMaxIndex[i];//参数修改
				extrMax_1s[m] = data[eMaxIndex[i]];
				m ++;
			}
		}
		else
		{
			if (eMaxIndex[i] >= 8) //中间位置的峰值点判断
			{
				if (((data[eMaxIndex[i]] - data[eMaxIndex[i]+8] > STRENGTH_ALG) || (data[eMaxIndex[i]] - data[eMaxIndex[i]-8] > STRENGTH_ALG)) && (data[eMaxIndex[i]] > STEP_THRSHD_WRIST))//参数修改
				{
					eMaxIndex_1s[m] = eMaxIndex[i];
					extrMax_1s[m] = data[eMaxIndex[i]];
					m ++;
				}
			}
			else   //左侧边缘的峰值点判断
			{
				if (((data[eMaxIndex[i]] - data[eMaxIndex[i]+8] > STRENGTH_ALG) || (data[eMaxIndex[i]] - data_pre_alg[eMaxIndex[i]+2] > STRENGTH_ALG)) && (data[eMaxIndex[i]] > STEP_THRSHD_WRIST))// 1000
				{
					eMaxIndex_1s[m] = eMaxIndex[i];
					extrMax_1s[m] = data[eMaxIndex[i]];
					m ++;
				}
			}
		}
	}

	for (i = 0; i < 10; i++)
	{
		data_pre_alg[i] = data[i + 40];//更新上一秒的最后10个数
	}

}


/**********************************************************************************
*函数名称： StepStart
*功能描述： 判断是否已经开始运动
*参数说明：
*返回说明： 无
*通用变量： step_start 的结果 1 开始运动   0停止运动
*说    明：
**********************************************************************************/
void StepStart(int32_t data[],int16_t data_length)
{
	int8_t i;
	int8_t steps_cnt = 0; //峰值点间距矩阵step_space，当前值与后第2个的值比较在[-5,5]之间的个数
	int8_t steps_cnt1 = 0;//峰值点间距矩阵step_space，当前值与后边第2个的值比较在[-8,-5] [5 8]之间的个数
	int8_t steps_cnt2 = 0;//峰值点间距矩阵step_space，当前值与后边第1个的值比较在[-7,7]之间的个数

	int16_t step_space[DET_LENGTH] = { 0,0,0,0,0,0,0,0,0,0,0 };

	Estep_num = 0;//初始化Estep_num

	/****************wrist*********************/
	for (i = 0; i < (data_length - 1); i++)
	{
		step_space[i] = (int16_t)(data[i + 1] - data[i]);
		//if (step_space[i] < 12 || step_space[i] > 80)
		//StepStart_flag = 0;
	}
	/**********间隔步频做比较*******************/
	for (i = 0; i < (data_length - 3); i++)
	{
		if (((step_space[i] - step_space[i + 2]) < 5) && ((step_space[i] - step_space[i + 2]) > -5))//4
		{
			steps_cnt++;
		}
		else
		{
			if (((step_space[i] - step_space[i + 2]) < 8) && ((step_space[i] - step_space[i + 2]) > -8))//6
			{
				steps_cnt1++;
			}
		}
	}
	/**********相邻步频做比较*******************/
	for (i = 0; i < (data_length - 2); i++)
	{
		if (((step_space[i] - step_space[i + 1]) < 7) && ((step_space[i] - step_space[i + 1]) > -7))//10
		{
			steps_cnt2++;
		}
		else
		{
			Estep_num = i;       //范围之外代表步数出错，记录该出错的点的位置
		}
	}
	/*
	** 峰值点间隔之间的差值都在阈值范围内，stop_flag = 0; 否则 stop_flag = 1;
	*/
	//if ((steps_cnt2 >= (data_length-3)) || (steps_cnt3 >= (data_length-2)) || (steps_cnt >= (data_length-3)))//&& (steps_cnt4 < 1)
	if ((steps_cnt2 >= (data_length - 2)) || (steps_cnt >= (data_length - 3)))
	{
		step_start = 1;    //开始运动
		Estep_num = 0;
	}
	else
	{
		step_start = 0;
	}
}

/**********************************************************************************
*函数名称：
*功能描述： 卡路里计算
*参数说明：
*返回说明： 无
*通用变量：
*说    明：
**********************************************************************************/

void calc_calorie(float calor_coeff,uint8_t calor_sec_alg,uint32_t data_strength)
{
  double BMR;
  float K_coeff;
  float strength_coeff;

  if(Sex == 0x01)   //for man
  {
    //BMR = 134*Weight_all;
    //BMR = BMR + 4799*Height;
    //BMR = BMR - 5677*Age;
    //BMR = BMR + 88362;
    //BMR = BMR / 86400;
    BMR = ( 134*Weight_all + 4799*Height - 5677*Age + 88362 )/86400;
  }
  else            //for woman
  {
    BMR = ( 92*Weight_all+ 3098*Height - 4330*Age + 447593 )/86400;
  }

  //确定步频系数
  /*if(calor_coeff < 1)
  K_coeff = 0.99;//1.07
  else if(calor_coeff >= 1 && calor_coeff < 2)
  K_coeff = 5;//4.5
  else if(calor_coeff >= 2 && calor_coeff < 3)
  K_coeff = 6;//5.5
  else
  K_coeff = 12;//11.5*/
  if(calor_coeff < 1)
    K_coeff = 1.386f;//0.99
  else if(calor_coeff >= 1 && calor_coeff < 1.5f)
    K_coeff = 3.5f;//2.5
  else if(calor_coeff >= 1.5f && calor_coeff < 2)
    K_coeff = 4.9f;//3.5
  else if(calor_coeff >= 2 && calor_coeff < 2.5f)
    K_coeff = 5.785f;//6.5
  else
    K_coeff = 8.9f;

  //确定强度系数
  if(data_strength < 630000)
    strength_coeff = 0.81f;
  else if(data_strength >= 630000 && data_strength < 750000)
    strength_coeff = 0.9f;
  else
    strength_coeff = 1.1f;
  /*else
  strength_coeff = 1.25;*/

  if(mode_flag == 0)  // 腰部模式
  {
    if(data_strength == 0)
		calorie_sum += K_coeff * (BMR/1000) * calor_sec_alg;
    else
		calorie_sum += K_coeff * (BMR/1000) * calor_sec_alg * strength_coeff;
  }
  else
  {
	  calorie_sum += K_coeff * (BMR/1000) * calor_sec_alg;
  }

  calorie_display = (uint16_t)calorie_sum;

  //_NOP();
}




/**********************************************************************************
*函数名称：  mean_distance(void)
*功能描述：
*参数说明：
*返回说明：
*通用变量： 无
*说    明：
**********************************************************************************/

float mean_distance(int32_t *peaks_idx_arr)
{
	uint8_t i = 0;
	int8_t peaks_num = 0;      //eMaxIndex_all的有效长度

	int8_t distance_arr_len = 0;
	int16_t distance_2peaks_arr[DET_LENGTH] = { 0,0,0,0,0,0,0,0,0,0,0 }; //峰峰值之间的距离
	int16_t distance_sum = 0;
	float avg_tmp = 0;

	for (i = 0; i < DET_LENGTH; i++)
	{
		if (peaks_idx_arr[i] != 0)
		{
			peaks_num++;    //eMaxIndex_all的有效长度
		}
	}
	//求平均频率（峰值点之间的间距，用点数表示）
	for (i = 0; i < (peaks_num - 1); i++)
	{
		distance_2peaks_arr[i] = (int16_t)(peaks_idx_arr[i + 1] - peaks_idx_arr[i]);
		if (distance_2peaks_arr[i] != 0)
		{
			distance_arr_len++;
			distance_sum = distance_sum + distance_2peaks_arr[i];
		}
	}
	avg_tmp = (float)distance_sum / distance_arr_len;      //mean(峰峰间距)
	return avg_tmp;

}


/**********************************************************************************
*函数名称： StepActive(int32_t acc_data[])
*功能描述： 计步程序
*参数说明： acc_data：当前1s的滤波和加速度
*返回说明： 无
*通用变量： 无
*说    明： 每秒调用
**********************************************************************************/

void StepActive(int32_t acc_data[])
{
	int16_t i = 0;
	int16_t j = 0;
	int8_t peaks_num = 0;  //eMaxIndex_all的有效长度

	int16_t steps = 0;                //每s的步数
	int8_t steps_error_cnt = 0;       // 1s错误的步数
	uint8_t steps_error_flag = 0;     //1s内有步数，错误步数超过2步，flag=1

	//uint16_t K_all;
	//uint8_t K_steps;     //calorie计算公式系数

	int16_t dev_of_pp_distance;       // eMaxIndex_all[]中两相邻峰值点之间的距离-f_avg(所有间隔的平均值)
	int16_t dev_of_pp_distance_space; // eMaxIndex_all[] 中间隔1点之间的距离/2 -f_avg(所有间隔的平均值)

	float avg_stepfreq = 0; // 平均步频

	int32_t rsl_mpy;
	uint32_t threshold_sum = 0;
	//uint32_t strength_peak = 0;

   /*************************************/
   //wrist mode
   /************************************/
	if (mode_flag == 1)             //手腕mode
	{

		find_peaks(acc_data);//求极大值，index和val结果保存在eMaxIndex_1s[] extrMax_1s[]中

		for (i = 0; i < 5; i++)
		{
			if (eMaxIndex_1s[i] != 100)
			{
				steps++;      // 当前1s极大值的个数
				if (eMaxIndex_cnt >(DET_LENGTH - 1))  //当前1s内有极大值的时候，更新数组eMaxIndex_all[11] & extrMax_all[11]的值
				{
					//新数据替换原有数据
					for (j = 0; j < (DET_LENGTH - 1); j++)
					{
						eMaxIndex_all[j] = eMaxIndex_all[j + 1];
						extrMax_all[j] = extrMax_all[j + 1];
					}
					rsl_mpy = sec_alg*WINDOW_LEN;
					eMaxIndex_all[(DET_LENGTH - 1)] = eMaxIndex_1s[i] + rsl_mpy;
					extrMax_all[(DET_LENGTH - 1)] = extrMax_1s[i];
				}
				else
				{
					rsl_mpy = sec_alg*WINDOW_LEN;
					eMaxIndex_all[eMaxIndex_cnt] = eMaxIndex_1s[i] + rsl_mpy;
					extrMax_all[eMaxIndex_cnt] = extrMax_1s[i];
					eMaxIndex_cnt++;
				}
			}
		}

		steps_sum = steps_sum + steps;  //步数相加

	  /*
	  * 若开始走路，当前1s的steps>0，若走路则求steps_display     卡路里 更新阈值
	  * 判断是否继续走路step_go = 1，还是停止走路（连续2秒没步数 || 1s内出现2个错误计步）step_go = 0
	  */
		if (step_start == 1)
		{
			//K_all = steps_display;

			if (steps_sum > DET_STEPS_LENGTH)
			{
				if (steps == 0)
				{
					stop_time++;
				}
				else
				{
					stop_time = 0;

					for (i = 0; i < DET_LENGTH; i++)
					{
						if (eMaxIndex_all[i] != 0)
						{
							peaks_num++;    //eMaxIndex_all的有效长度
						}
					}

					//printf("1-eMaxIndex_cnt = %d;%d\n", eMaxIndex_cnt, peaks_num);

					for (i = 0; i < steps; i++)
					{
						dev_of_pp_distance = (int16_t)(eMaxIndex_all[peaks_num - (i + 1)] - eMaxIndex_all[peaks_num - (i + 2)]) - (int16_t)avg_points_of_pp;
						dev_of_pp_distance_space = (int16_t)(eMaxIndex_all[peaks_num - (i + 1)] - eMaxIndex_all[peaks_num - (i + 3)]) / 2 - (int16_t)avg_points_of_pp;
						if (((dev_of_pp_distance < 10) && (dev_of_pp_distance > -10)) || ((dev_of_pp_distance_space < 10) && (dev_of_pp_distance_space > -10)))//&& (extrMax_all[peaks_num - (i + 1)] > STEP_THRSHD) 8
						{
							steps_display = steps_display + 1;  // 计算当前显示的总步数
						}
						else
						{
							steps_error_cnt++;     // 当前1s的steps种错误的步数
						}
					}

					avg_points_of_pp = mean_distance(eMaxIndex_all);      //mean(峰峰间距)
					if (avg_points_of_pp > 0)
					{
						avg_stepfreq = (float)(50 / avg_points_of_pp);  //平均步频
					}

					calc_calorie(avg_stepfreq, 1, 0);       //计算卡路里

					//===========更新阈值================
					for (i = 0; i < peaks_num; i++)
					{
						threshold_sum = threshold_sum + extrMax_all[i];
					}
					//strength_peak = threshold_sum/peaks_num; //峰值的平均值
					threshold = threshold_sum / peaks_num - threshold_sum / peaks_num / 10; //参数可修改
					//==============

					temp_cnt_alg++;
					if (temp_cnt_alg >= 2)
					{
						if (steps_error_cnt >= 2)
						{
							steps_error_flag = 1;
						}
						else
						{
							steps_error_flag = 0;
						}
						steps_error_cnt = 0;
						temp_cnt_alg = 0;
					}
				}

				/* 计步错误 or 连续2秒无计步，则停止计步，初始化参数，重新计算;否则 step_go = 1;*/
				if ((steps_error_flag == 1) || (stop_time > 1))  //stop_time >= 1
				{
					//stop_flag = 1;
					step_start = 0;
					steps_stop_clc();
				}

			}
			//K_steps = steps_display - K_all;
			//calc_calorie(steps,1,0);
		}

		/*
		* 未开始走路，当前总步数<=6,等待；>6，则判断是否开始走路（stop_flag=0 即  step_start=1 ）
		* 判断初开始走路之后，求当前steps_dispaly 卡路里 更新阈值 ；否则，初始化各变量
		*/
		if (step_start == 0)
		{
			if (steps_sum > 0 && steps_sum <= DET_STEPS_LENGTH)
			{
				sec_wait_alg++;
			}
			if (steps_sum > DET_STEPS_LENGTH)
			{
				StepStart(eMaxIndex_all, steps_sum);   //得到stop_flag 的值

				/*若开始运动，求当前steps_display 卡路里 更新阈值 ；未开始运动，初始化变量重新开始*/
				if (step_start == 1) //step_start = 1; //开始运动
				{

					//steps_display = steps_display + steps_sum;
					for (i = 0; i < DET_LENGTH; i++)
					{
						if (eMaxIndex_all[i] != 0)
							peaks_num++;
					}
					//printf("eMaxIndex_cnt = %d;%d\n", eMaxIndex_cnt, peaks_num);

					avg_points_of_pp = mean_distance(eMaxIndex_all);        //mean(峰峰间距)
					if (avg_points_of_pp > 0)
					{
						avg_stepfreq = (float)(50 / avg_points_of_pp);  //每s钟的平均步数
					}

					calc_calorie(avg_stepfreq, sec_wait_alg, 0);
					sec_wait_alg = 0;

					/*************是否保留待定？？？**************/
					if (avg_points_of_pp >= 10)       //12
					{
						//if(Estep_num == 0)
						steps_display = steps_display + steps_sum;
						//else
						//steps_display = steps_display + steps_sum - (steps_sum - (Estep_num+2));
					}
					else
					{
						step_start = 0;      // 运动停止
						steps_stop_clc();   // 停止清零
					}
					/*****************************/
					//求出平均阈值
					for (i = 0; i < peaks_num; i++)
					{
						threshold_sum = threshold_sum + extrMax_all[i];
					}
					threshold = threshold_sum / peaks_num - threshold_sum / peaks_num / 10; //参数可修改100000

				}
				else  //没开始运动
				{
					//sec_alg = 0;  //sec_alg清零

					step_start = 0;
					//steps_sum = steps_sum - (Estep_num + 1);
					eMaxIndex_cnt = eMaxIndex_cnt - (Estep_num + 1);
					steps_sum = eMaxIndex_cnt;
					for (i = (Estep_num + 1); i < DET_LENGTH; i++)
					{
						eMaxIndex_all[i - (Estep_num + 1)] = eMaxIndex_all[i];
						extrMax_all[i - (Estep_num + 1)] = extrMax_all[i];
						eMaxIndex_all[i] = 0;
						extrMax_all[i] = 0;
					}
					for (i = eMaxIndex_cnt; i < DET_LENGTH; i++)
					{
						eMaxIndex_all[i] = 0;
						extrMax_all[i] = 0;
					}
					/********是否保留待定********/
					//calc_calorie(0,sec_wait_alg,0);
					/****************************/
					sec_wait_alg = 0;

				}
			}
		}

		/*
		** 该段开始走路的标志为还没有为1，但是有步数了，若出现连续2s没有steps,也清零
		*/
		if ((steps_sum > 0) && (steps_sum <= DET_STEPS_LENGTH))
		{
			if (steps == 0)
			{
				nostep_cnt++;
				//calc_calorie(0,1,0);
			}
			else
			{
				nostep_cnt = 0;
			}

			if (nostep_cnt > 1) //如果两秒没走路，清零标志位
			{
				//stop_flag = 1;
				step_start = 0;
				steps_stop_clc();
			}
		}
		/*else
		{
		if(steps == 0)
		calc_calorie(0,1,0);
		}*/

		if (steps != 0)
		{
			sec_alg++;
		}

		//K_steps = steps_display - K_all;
		//calc_calorie(1);
	}
}


/**********************************************************************************
*函数名称： alg_AM(int16_t data[])
*功能描述： 主程序
*参数说明： int16_t data_x,int16_t data_y,int16_t data_z 每个采样点的加速度值[x,y,z],对方向有要求：左手为例x指向手指的方向，y:垂直于手指的方向向外；z:垂直于手面向下
*返回说明： current_steps：当前总步数
*通用变量： 无
*说    明： 每个采样点调用，该处只适用于fs=50
**********************************************************************************/
//extern uint32_t sec_n;
void alg_AM(int16_t data_x,int16_t data_y,int16_t data_z,uint32_t *current_steps)
{

	int16_t i = 0 ;
	int16_t data;
	int32_t filtered_data[WINDOW_LEN];     // 50
	int8_t coeff_low[COEFF_FIR_LEN] = {-2,-5,-8,-10,-11,-11,-9,-5,2,10,20,30,41,50,57,62,64,
	62,57,50,41,30,20,10,2,-5,-9,-11,-11,-10,-8,-5,-2};      // sum = 486

	point_cnt++;
	data = sqrt((int32_t)data_x * data_x + (int32_t)data_y * data_y); // 手腕上采用x y 2轴的合加速度
	xyz_sqrt_data[point_cnt + 33-1] = data;

	if (point_cnt == 50)
	{
		Filter(xyz_sqrt_data, coeff_low, FILTERED_DATA_LEN, COEFF_FIR_LEN);   //滤波
		for (i = COEFF_FIR_LEN; i < FILTERED_DATA_LEN; i++)
		{
			filtered_data[i - COEFF_FIR_LEN] = FilterResult[i];          //舍去前33个点
			//printf("%d\t%d\n", sec_n*50+i- coeff_low_length +1,Result_Filter[i - coeff_low_length]);
		}

		StepActive(filtered_data);

		for (i = 50; i < 83; i++)
		{
			xyz_sqrt_data[i - 50] = xyz_sqrt_data[i];
		}
		point_cnt = 0;

	}

	*current_steps = steps_display;
}

/**********************************************************************************
*函数名称： Alg_Clear_RAM(void)
*功能描述： 变量清零
*参数说明：
*返回说明： 无
*通用变量： 无
*说    明：
**********************************************************************************/
void Alg_Clear_RAM(void)
{
	uint8_t i;

	point_cnt = 0;
	steps_display = 0;
	step_start = 0;
	steps_stop_clc();

	eMaxIndex_cnt = 0;
	for(i = 0; i < 5; i ++)
	{
		eMaxIndex_1s[i] = 100;
		extrMax_1s[i] = 0;
	}

	mode_flag = 1;

	for (i = 0; i < FILTERED_DATA_LEN; i++)
	{
		xyz_sqrt_data[i] = 0;
		FilterResult[i] = 0;
	}
	for (i=0;i<10;i++)
	{
		data_pre_alg[i] = 500000;
	}

	avg_points_of_pp = 0;               // 峰峰间距的均值，单位：采样点个数 avg_points_of_2peaks
	stop_time = 0;                    // 开始运动后，每秒的steps为0的计数 ，(若连续2s没有步数，运动停止)
	nostep_cnt = 0;                   // 没有步数的1s计数
	temp_cnt_alg = 0;
	Estep_num = 0;

	calorie_sum = 0;
	calorie_display = 0;

}
