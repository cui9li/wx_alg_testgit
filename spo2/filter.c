#include "filter.h"
#include <stdint.h>
#include <string.h>

//循环移位
#define RING_ADD(a,b,m) (((a)+(b))%(m))
#define RING_SUB(a,b,m) (((a)+(m)-(b))%(m))

//===========FIR filter  参数配置:系数，阶数，增益等=====================================
//#define FIR_FILTER_LENGTH    33
#define FIR_COE_SUM          6830
/*
% matlab code
clc;
N = 50;   % 采样率
Wn = [0.5, 5] * 2 / N;
Hd = fir1(32, Wn);
plot(Hd)
freqz(Hd, 50)
*/
/*   [0.5 5]hz   PPG_FIR_COFFES[FIR_FILTER_LENGTH] = Hd*10000    */
int32_t PPG_FIR_COFFES[FIR_FILTER_LENGTH]=
{
    -23, -16, -5, 9, 16, -4, -70, -184,
    -317, -406, -374, -161, 241, 770, 1306, 1706,
    1854,
    1706, 1306, 770, 241, -161, -374, -406, -317,
    -184, -70, -4, 16, 9, -5, -16, -23
};                                                 // sum(PPG_FIR_COFFES) = 6830  设置依据是什么？
static int32_t mLpFilterIdx = 0;
static int32_t mLpFilterBuff[FIR_FILTER_LENGTH];   //原始数据缓存--用于滤波


void init_filter(void)
{
    mLpFilterIdx = 0;

    memset(mLpFilterBuff, 0, sizeof(mLpFilterBuff));
}

/**********************************************************************************
 * 函数名称：filter(int32_t data)--原版本
 * 功能描述：滤波
 * 参数说明：buf的存储从0-32，然后又从0-32
 * 返回说明：
 * 说    明：只适用于单路ppg滤波，不适合red ir green 同时滤波，注意
 **********************************************************************************/

int32_t filter(int32_t data)
{
	int64_t sum = 0;
	int64_t temp = 0;
	int32_t headIdx = 0;
	int32_t caudaIdx = 0;
	int32_t i = 0;
	int32_t result = 0;

	headIdx = mLpFilterIdx;
	caudaIdx = RING_ADD(headIdx, 1, FIR_FILTER_LENGTH);
	mLpFilterBuff[headIdx] = data;

	for (i = 0; i < FIR_FILTER_LENGTH / 2; i++)
	{

		temp = (mLpFilterBuff[headIdx] + mLpFilterBuff[caudaIdx]);
		temp *= PPG_FIR_COFFES[i];
		sum += temp;
		headIdx = RING_SUB(headIdx, 1, FIR_FILTER_LENGTH);
		caudaIdx = RING_ADD(caudaIdx, 1, FIR_FILTER_LENGTH);

	}
	if (FIR_FILTER_LENGTH % 2 != 0)  //奇数
	{

		temp = mLpFilterBuff[caudaIdx];
		temp *= PPG_FIR_COFFES[i];
		sum += temp;
	}

	mLpFilterIdx = RING_ADD(mLpFilterIdx, 1, FIR_FILTER_LENGTH);

	//result = sum / 64; //计算结果比原信号大了很多倍
	result = sum / FIR_COE_SUM;
	return result;
}

/******************************************************************************
 * 函    数：  fir_filter_s32(int32_t data,uint8_t *mLpFilterIdx,int32_t *mLpFilterBuff )
 * 功能描述:   滤波
 * 参数说明： data:原始信号 mLpFilterIdx：缓存位置  mLpFilterBuff：原始数据缓存数组
 * 版    本：
 * 返回说明：
 * 说    明：  buf的存储从0-32，然后又从0-32  ,循环存储0-32 1-32-0 2-32-1
 * 问    题：  适合多路光同时滤波
 *****************************************************************************/

int fir_filter_s32(int32_t data, uint8_t *mLpFilterIdx, int32_t *mLpFilterBuff)
{
	int32_t i = 0;
	int32_t headIdx = 0;
	int32_t caudaIdx = 0;
	int32_t result = 0;
	int64_t sum = 0;
	int64_t temp = 0;

	headIdx = *mLpFilterIdx;
	caudaIdx = RING_ADD(headIdx, 1, FIR_FILTER_LENGTH);
	mLpFilterBuff[headIdx] = data;

	for (i = 0; i < FIR_FILTER_LENGTH / 2; i++)
	{
		temp = (mLpFilterBuff[headIdx] + mLpFilterBuff[caudaIdx]);
		temp *= PPG_FIR_COFFES[i];
		sum += temp;
		headIdx = RING_SUB(headIdx, 1, FIR_FILTER_LENGTH);
		caudaIdx = RING_ADD(caudaIdx, 1, FIR_FILTER_LENGTH);
	}

	if (FIR_FILTER_LENGTH % 2 != 0)      //奇数
	{
		temp = mLpFilterBuff[caudaIdx];
		temp *= PPG_FIR_COFFES[i];
		sum += temp;
	}

	*mLpFilterIdx = RING_ADD(*mLpFilterIdx, 1, FIR_FILTER_LENGTH);

	//result = sum / 64;             //计算结果比原信号大了很多倍
	result = sum / FIR_COE_SUM;      //2021
	return result;
}

#if 0
/******************************************************************************
 * 函    数：  fir_filter_s32(int32_t data,uint8_t *mLpFilterIdx,int32_t *mLpFilterBuff )
 * 功能描述:   滤波
 * 参数说明： data:原始信号 b :滤波器系数，b_len: 滤波器长度， mLpFilterIdx：缓存位置  mLpFilterBuff：原始数据缓存数组
 * 版    本：
 * 返回说明：
 * 说    明：  buf的存储从0-32，然后又从0-32
 * 问    题：  适合多路光同时滤波
 *****************************************************************************/

int fir_filter_s32(int32_t data,int32_t *b,int32_t b_len, int32_t gain,uint8_t *mLpFilterIdx, int32_t *mLpFilterBuff)
{
	int32_t i = 0;
	int32_t headIdx = 0;
	int32_t caudaIdx = 0;
	int32_t result = 0;
	int64_t sum = 0;
	int64_t temp = 0;

	headIdx = *mLpFilterIdx;
	caudaIdx = RING_ADD(headIdx, 1, FIR_FILTER_LENGTH);
	mLpFilterBuff[headIdx] = data;

	for (i = 0; i < b_len / 2; i++)
	{
		temp = (mLpFilterBuff[headIdx] + mLpFilterBuff[caudaIdx]);
		temp *= b[i];
		sum += temp;
		headIdx = RING_SUB(headIdx, 1, b_len);
		caudaIdx = RING_ADD(caudaIdx, 1, b_len);
	}

	if (b_len % 2 != 0)      //奇数
	{
		temp = mLpFilterBuff[caudaIdx];
		temp *= b[i];
		sum += temp;
	}

	*mLpFilterIdx = RING_ADD(*mLpFilterIdx, 1, b_len);

	//result = sum / 64;             //计算结果比原信号大了很多倍
	result = sum / gain;    //2021
	return result;
}
#endif

/******************************************************************************
 * 函数名称：median
 * 功能描述:  取数组的中位数
 * 参数说明：
 * 返回说明：
 * 说    明：
 *****************************************************************************/

int32_t median_s32(int32_t *arr, uint8_t arr_len)
{
	uint8_t i, j;
	int32_t tmp;
	//int32_t sum = 0;
	//int32_t median = 0;

	int32_t arr_tmp[10];  // 为了对arr进行操作，同时不改变arr，开辟临时变量
	/*
	memcpy  用于把资源内存s拷贝到目标内存ss,拷贝的个数用size控制,拷贝的是字节数（一个字节一个字节拷贝）
	* 因此，实际应用的时候大小应该转化成字节数  memcpy(ss, s, size * sizeof(int16_t));
	*/
	memcpy(arr_tmp, arr, arr_len * sizeof(int32_t));

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

/******************************************************************************
 * 函数名称： truncated_mean
 * 功能描述:  截尾法取均值（去掉前后几个值）
 * 参数说明： int32_t start,int32_t end  要计算数组的开始位置和结束位置（C语言标准）
 * 返回说明：
 * 说    明：
 *****************************************************************************/
int32_t truncated_mean_s32(int32_t *arr, uint8_t arr_len, uint8_t start, uint8_t end) {

	uint8_t i, j;
	int32_t tmp;
	int32_t sum = 0;
	int32_t avg = 0;

	int32_t arr_tmp[10];  // 为了对arr进行操作，同时不改变arr，开辟临时变量

	memcpy(arr_tmp, arr, arr_len * sizeof(int32_t));

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

	for (i = start; i < end + 1; i++) {
		sum += arr_tmp[i];
	}

	avg = sum / (end - start + 1);
	return avg;
}


/******************************************************************************
 * 函数名称： mean
 * 功能描述:  取数组的均值
 * 参数说明： int32_t start,int32_t end  要计算数组的开始位置和结束位置（C语言标准）
 * 返回说明：
 * 说    明：
 *****************************************************************************/
int32_t mean_s32(int32_t *arr, uint8_t start, uint8_t end) {

	uint8_t i = 0;
	int32_t sum = 0;
	int32_t avg = 0;

	for (i = start; i < end + 1; i++) {
		sum += arr[i];
	}

	avg = sum / (end - start + 1);
	return avg;
}
