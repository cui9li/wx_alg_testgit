#include "debug.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <direct.h>
#include "dirent.h"
#include "../alg_ComFunc.h"
//#include "main.h"

//==========================================

#ifdef output_flag

const char output_dir[LEN_NAME] = "D:\\02-pulseRate20111118\\dataSpO2\\data_wrist_MeiXin\\";
char grn_filter_addr[LEN_NAME] = "";
FILE *pfile_grnfilter = NULL;
void debug_init(void)
{
	int i;
	for (i = 0; i < LEN_NAME; i++)
	{
		grn_filter_addr[i] = output_dir[i];
	}
	strcat(grn_filter_addr, "grn_smooth//");
	_mkdir(grn_filter_addr);
}

int startup_file(char *data_name)
{
	strcat(grn_filter_addr, data_name);
	pfile_grnfilter = fopen(grn_filter_addr, "w");
	if (NULL == pfile_grnfilter)
	{
		return -1;
	}
	return 1;
}

void close_file(void)
{
	fclose(pfile_grnfilter);
}

#else

void debug_init(void)
{
}
int startup_file(char *data_name)
{
	(void)data_name;
	return 0;
}

void close_file(void)
{

}
#endif
//==========================================


//
////==========================================
//
//#ifdef OUT_HR
//const char output_dir[LEN_NAME] = "D:\\06-HRV20220209\\03-questionsFromFeng20220211\\newMY11201874\\";
//char hr_addr[LEN_NAME] = "";
//FILE *pfile_hr = NULL;
//
//void debug_init(void)
//{
//	int i;
//	for (i = 0; i < LEN_NAME; i++)
//	{
//		hr_addr[i] = output_dir[i];
//}
//	strcat(hr_addr, "hr_vc//");
//	_mkdir(hr_addr);
//}
//
//int startup_file(char *data_name)
//{
//	strcat(hr_addr, data_name);
//	pfile_hr = fopen(hr_addr, "w");
//	if (NULL == pfile_hr)
//	{
//		return -1;
//	}
//	return 1;
//}
//
//void close_file(void)
//{
//	fclose(pfile_hr);
//}
//#endif
//
//==========================================




#ifdef HR_STATS   // 统计

extern uint16_t step_cnt;               // 采样点计数（每个采样点调用一次算法）
extern int32_t detect_pulse_cnt;          // bpm>0 的心拍个数

uint8_t HR_N[ARR_LEN] = { 0 };
uint8_t hr_median = 0;
uint8_t hr_mean = 0;
float hr_std = 0;
uint8_t hr_len = 0;
uint16_t file_len = 0;
uint8_t len_hrAbnormal = 0;  // 异常值的个数

float RR_N[ARR_LEN] = { 0 };
float rr_mean = 0;
float rr_std = 0;
uint8_t len_rrIn10Mean = 0;
uint8_t len_rrIn20Mean = 0;
uint8_t len_rrIn30Mean = 0;


uint8_t SPO_N[ARR_LEN] = { 0 };
uint8_t spo_len = 0;
uint8_t spo_median = 0;
uint8_t spo_mean = 0;
float spo_std = 0;
float spo_IQR = 0;            // 四分位数间距IQR  = Q3-Q1
uint8_t len_spoAbnormal = 0;  // 血氧值异常(<95)的个数


void init_parameters(void)
{
	for (int32_t i = 0; i < ARR_LEN; i++)
	{
		HR_N[i] = 0;
		RR_N[i] = 0;
		SPO_N[i] = 0 ;
	}
	hr_median = 0;
	hr_mean = 0;
	hr_std = 0;
	hr_len = 0;
	file_len = 0;
	len_hrAbnormal = 0;

	rr_mean = 0;
	rr_std = 0;
	len_rrIn10Mean = 0;
	len_rrIn20Mean = 0;
	len_rrIn30Mean = 0;

	spo_median = 0;
	spo_len = 0;
	spo_mean = 0;
	spo_std = 0;
	spo_IQR = 0;

	len_spoAbnormal = 0;
}

float mean_f(float arr[],uint16_t NL,uint16_t NU)
{
	uint16_t i = 0;
	float sum = 0;
	float tmp = 0;
	for (i=NL;i<NU;i++)
	{
		sum += arr[i];
	}
	tmp = sum / (NU - NL);
	return tmp;
}

float std_u8(uint8_t arr[], uint8_t NL, uint8_t NU)
{
	uint8_t mean_val = uint8_t_mean(arr, NL, NU);
	float tmp = 0;
	for (uint8_t i = NL; i < NU; i++)
	{
		tmp += (arr[i] - mean_val)*(arr[i] - mean_val);
	}
	tmp = tmp / (NU - NL);
	tmp = sqrt(tmp);

	return tmp;
}

float std_f(float arr[], uint16_t NL, uint16_t NU)
{
	float mean_val = mean_f(arr, NL, NU);
	float tmp = 0;
	for (uint16_t i = NL; i < NU; i++)
	{
		tmp += (arr[i] - mean_val)*(arr[i] - mean_val);
	}
	tmp = tmp / (NU - NL);
	tmp = sqrt(tmp);

	return tmp;
}


//******************************************************************************/
//函数名称：median
//功能描述:  取数组的中位数
//参数说明：
//返回说明：
//说    明：
//**********************************************************************************/

uint8_t median_u8(uint8_t *arr, uint16_t arr_len)
{
	uint16_t i, j;
	uint8_t tmp;
	int32_t sum = 0;
	uint8_t median = 0;

	uint8_t arr_tmp[ARR_LEN];  // 为了对arr进行操作，同时不改变arr，开辟临时变量
						   /* memcpy
						   用于把资源内存s拷贝到目标内存ss,拷贝的个数用size控制,拷贝的是字节数（一个字节一个字节拷贝）
						   * 因此，实际应用的时候大小应该转化成字节数
						   memcpy(ss, s, size * sizeof(int16_t));
						   */
	memcpy(arr_tmp, arr, arr_len * sizeof(uint8_t));

	/*升序排列*/
	for (i = 0; i < (arr_len - 1); i++) {
		for (j = 0; j < (arr_len - i - 1); j++) {
			if (arr_tmp[j] > arr_tmp[j + 1]) {
				tmp = arr_tmp[j];
				arr_tmp[j] = arr_tmp[j + 1];
				arr_tmp[j + 1] = tmp;
			}
		}
	}

	if (arr_len % 2 == 0) //偶数
		tmp = (arr_tmp[arr_len / 2 - 1] + arr_tmp[arr_len / 2]) / 2;

	else                  //奇数
		tmp = arr_tmp[arr_len / 2];

	return tmp;
}

/* 四分位数 arr_len >3*/
float prctile_u8(uint8_t *arr, uint16_t arr_len,uint8_t n)
{
	uint16_t i, j;
	uint8_t tmp;
	float w = 0.0;
	float z = 0.0;
	uint8_t y = 0;
	float Q_n = 0;


	uint8_t arr_tmp[ARR_LEN];  // 为了对arr进行操作，同时不改变arr，开辟临时变量
						   /*
						   * memcpy 用于把资源内存s拷贝到目标内存ss,拷贝的个数用size控制,拷贝的是字节数（一个字节一个字节拷贝）
						   * 因此，实际应用的时候大小应该转化成字节数memcpy(ss, s, size * sizeof(int16_t));
						   */
	memcpy(arr_tmp, arr, arr_len * sizeof(uint8_t));

	/*升序排列*/
	for (i = 0; i < (arr_len - 1); i++)
	{
		for (j = 0; j < (arr_len - i - 1); j++)
		{
			if (arr_tmp[j] > arr_tmp[j + 1])
			{
				tmp = arr_tmp[j];
				arr_tmp[j] = arr_tmp[j + 1];
				arr_tmp[j + 1] = tmp;
			}
		}
	}

	if ((arr_len+1) % 4 == 0)  //整除
	{
		w = (arr_len+1)*n / 4;
		y = w;
		z = 0;
	}
	else
	{
		w = (float)(arr_len + 1)*n / 4;
		y = (arr_len + 1)*n / 4;
		z = w - (float)y;
	}
	w = (arr_len / 2);
	Q_n = arr_tmp[y - 1] + (arr_tmp[y]-arr_tmp[y-1])*z;

	return Q_n;
}


uint8_t cal_len_inrange(float mean,float range_per)
{
	uint8_t len = 0;
	for (uint16_t i = 0; i < hr_len; i++)
	{
		if (RR_N[i] <= mean + mean*range_per && RR_N[i] >= mean - mean*range_per)
		{
			len++;
		}

	}
	return len;
}

void cal_hr_statistics(void)
{
	uint16_t i = 0;
	file_len = step_cnt /50;               // 采样点计数（每个采样点调用一次算法）
	hr_len = detect_pulse_cnt;             // bpm>0 的心拍个数
	if (hr_len)
	{
		hr_median = median_u8(HR_N, hr_len);    // 中位数
		hr_mean = uint8_t_mean(HR_N, 0, hr_len);// 均值
		hr_std = std_u8(HR_N, 0, hr_len);       //标准差

		for (i = 0; i < hr_len; i++)
		{
			if ( HR_N[i]<40 || HR_N[i]>100 )
			{
				len_hrAbnormal++;              //异常值的个数：根据样本的真实值定的范围[40 100],在计算范围外的相当于异常值
			}
		}

		for (i = 0; i < hr_len; i++)
		{
			RR_N[i] = 3000.0 / (float)HR_N[i];
		}
		rr_mean = mean_f(RR_N, 0, hr_len);      // rr间期均值
		rr_std = std_f(RR_N, 0, hr_len);        // rr 标准差

		len_rrIn10Mean = cal_len_inrange(rr_mean, 0.1);
		len_rrIn20Mean = cal_len_inrange(rr_mean, 0.2);
		len_rrIn30Mean = cal_len_inrange(rr_mean, 0.3);
		//printf("%d\t%d\t%d\t%d\n", len_rrIn10Mean, len_rrIn20Mean, len_rrIn30Mean, len_hrAbnormal);
	}
}

/**/
void cal_spo_statistics(void)
{
	uint16_t i = 0;
	float spo_Q1 = 0;
	float spo_Q3 = 0;

	file_len = stepCount / 50;               // 采样点计数（每个采样点调用一次算法）
	spo_len = detect_pulse_cnt - 3;           // bpm>0 的心拍个数  3跟算法中的设置有关
	if (spo_len)
	{
		spo_median = median_u8(SPO_N, spo_len);    // 中位数
		spo_mean = uint8_t_mean(SPO_N, 0, spo_len);// 均值
		spo_std = std_u8(SPO_N, 0, spo_len);       //标准差

		for (i = 0; i < spo_len; i++)              //异常值的个数：根据样本的真实值定的范围[40 100],在计算范围外的相当于异常值
		{
			if (SPO_N[i]<95 || SPO_N[i]>100)
			{
				len_spoAbnormal++;
			}
		}
		/*四分位数及差值*/
		if (spo_len > 2)
		{
			spo_Q1 = prctile_u8(SPO_N, spo_len, 1); // 1/4分位数
			spo_Q3 = prctile_u8(SPO_N, spo_len, 3); // 1/4分位数
			spo_IQR = spo_Q3 - spo_Q1;
		}
		else
		{
			spo_IQR = -1;
		}

		printf("%d\t%d\t%f\t%f\n", spo_median, spo_mean, spo_std,spo_IQR);
	}
}

#else
void init_parameters(void)
{

}
void cal_hr_statistics(void)
{
}
void cal_spo_statistics(void)
{}
#endif // HR_STATS
