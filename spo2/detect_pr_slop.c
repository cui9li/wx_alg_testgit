/*****************************************************************************
* @file         detect_pr_slop.c
* @author       sky
* @version      V0.1
* @date         2020-12-09
* @brief
*--------------------------------------------------------------------*
* Change History :
* <Data>      |<Version> |<Author>  | <Description>
*--------------------------------------------------------------------*
*  2021-12-13 |1.0       |wx_John   |1. valueShakeCnt 更改为 systoleCnt ---收缩时间（快速上升期）  VALUE_SHAKE_THRESHOLD更改为SYSTOLE_THL
*                                    2. isFirstPulse 初始化由0变为1，1：是第一个；0非第一个，跟原来相反
*                                    3. deviationBuf[i] > 10000 重复，把10000更改为100000
*  2022-04-14 |1.1       |wx_John   |1. 调用心率计算函数时，先去除趋势项，使信号在0附近上下波动，选择峰值点时，+峰值点的幅值>0的限制条件
*
* ----------------------------------------------------------------
*****************************************************************************/

#include "detect_pr_slop.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
//#include "../alg_ComFunc.h"

/*---------for debug----------------- */
//#include <string.h>
//uint8_t debug_print = 0;
//static int32_t resultCnt = 0;               //测试用
/* ----------end debug --------------*/


/* Pulse detection parameters */

#define SYSTOLE_THL                 6        // 7//12 //5  value 5 easy count huge hr
#define PULSE_BPM_SAMPLE_SIZE       5        // Moving average size
#define STEP_OVERFLOW_COMPENSATION  256      // not mention?
#define DEFAULT_PR                  75       // not mention?

// 如果光强或者结构有改变，以下2个变量应该适当调整
#define DATA_BASE_DIFF_MIN          0        // 200 //800 //1600 //value too large easy loss beat place
#define DATA_BASE_DIFF_MAX          80000    // value to filter noise

#define COUNT_HR_TIME_OUT           4        // 30BMP time * 2
#define HR_SHAKE_THL                15
#define UNVALUE_HR_SHAKE            60       // judgeHrQuality() 当前hr与上一个之间的偏差阈值
#define SHAKE_HR_MAX_TIMES          4

//20220308 RR间期阈值
#define RR_LOWER                    11       // 对应心率bpm = 60*fs/rr = 250
#define RR_UPPER                    75       // 对应心率bpm = 60*fs/rr = 40

//detectPulseBySlop()
static int32_t ppgSample = 50;               // 采样率，初始化的时候对该值进行更改
static int32_t preSensorValue = 0;           // 上一个采样点的幅值
static int32_t currentBeatIdx = 0;           // 当前峰值点位置
static int32_t lastBeatIdx = 0;              // 上一个峰值点的位置
static int32_t systoleCnt = 0;               // 峰值点处清零,收缩期（下坡）连续计数
static int32_t preSlop1 = 0;                 //
static int32_t preSlop2 = 0;
static int32_t currentSlop = 0;              // 当前value与上一个点的diff ,斜率
static uint16_t step_cnt = 0;                // 采样点计数（每个采样点调用一次算法）

static uint8_t slopFlag = 0;                 // 0 上斜坡  1：下坡
static uint8_t isFirstPulse = 1;             // 第一个波峰   202112

//calulatePr()
static int32_t prePr = 0;
static int32_t valuesBPM[PULSE_BPM_SAMPLE_SIZE];  //calulatePr() checkPrValue() 缓存pr值
static uint8_t BPMCnt;                            // valueBPM[]的下标计数
static uint8_t clearCnt;                          // valueBPM[]的下标计数
static int32_t hrCnt = 0;                         //计算不出bpm的点计数

//judgeHrQuality()
static int32_t judge_pre_hr = -1;
static int32_t hr_shark_cnt;                 // 与上一个pr 偏差超过阈值的 计数
static int32_t shark_step;


// checkDeviation()
static int32_t currentValue;                 //当前采样点的幅值
static int32_t deviationLimit;               //diff 阈值（收缩期幅值信息）
static int32_t deviationBuf[PULSE_BPM_SAMPLE_SIZE];  // 缓存 峰值点与之后连续下降的第6个点之间的幅值差
static int32_t deBufCnt;                     // deviationBuf 计数
static int32_t getHrCnt;                     // 每次采集开始后，detectPulseBySlop得到瞬时脉率进行计数，5个数后开始缓存结果到valuesBPM[]


void initDetectPulseBySlop(int32_t sampleRate)
{
	int32_t i;

	ppgSample = sampleRate;
	preSensorValue = 0;
	currentBeatIdx = 0;
	lastBeatIdx = 0;
	step_cnt = 0;
	systoleCnt = 0;
	//resultCnt = 0;
	preSlop1 = 0;
	preSlop2 = 0;
	currentSlop = 0;

	slopFlag = 0;
	isFirstPulse = 1;

	prePr = 0;
	hrCnt = 0;
	BPMCnt = 0;
	clearCnt = 0;
	judge_pre_hr = -1;
	hr_shark_cnt = 0;
	shark_step = 0;
	deviationLimit = 0;
	deBufCnt = 0;
	getHrCnt = 0;

	for(i = 0; i < PULSE_BPM_SAMPLE_SIZE; i++){
		valuesBPM[i] = 0;
		deviationBuf[i] = 0;
	}
}


/**********************************************************************************
* 函数名称：countNewDevLimit 原版本
* 功能描述：计算阈值---5个峰值点计算一次阈值
* 版    本：v1.0
* 参数说明：        无
* 返回说明：        无
* 说    明：
**********************************************************************************/

int32_t countNewDevLimit()
{
	int32_t i,j, tmp;
	//int32_t max_val = 0;


     /*升序排列: 此处改变了原来devitionBuf[] 中的顺序 源程序中没有关系，因为因程序中是更新完所有的数之后在排序   */
	for(i = 0; i < (PULSE_BPM_SAMPLE_SIZE - 1); i++){
		for(j = 0; j < (PULSE_BPM_SAMPLE_SIZE - i - 1); j++){
			if(deviationBuf[j] > deviationBuf[j+1]){
				tmp = deviationBuf[j];
				deviationBuf[j] = deviationBuf[j+1];
				deviationBuf[j+1] = tmp;
			}
		}
	}

	for (j = PULSE_BPM_SAMPLE_SIZE - 1; j > 0;j--)  // 排除最大峰值
	{
		if ((deviationBuf[j] * 1.0 / deviationBuf[j - 1]) < 2)
		{
			break;
		}
	}
	for(i = 0; i < j+1; i++){
		if(((deviationBuf[j]) * 1.0 / deviationBuf[i]) < 2){
			break;
		}
	}

	tmp = deviationBuf[i];

	return tmp;
}


/**********************************************************************************
* 函数名称：countNewDevLimit
* 功能描述：计算阈值
* 版    本：v1.0
* 参数说明：        无
* 返回说明：        无
* 说    明：
**********************************************************************************/
int32_t countNewDevLimit_min(int32_t*arr, uint8_t arr_len)
{

	uint8_t i, j;
	int32_t tmp;

	int32_t arr_tmp[10];  // 为了对arr进行操作，同时不改变arr，开辟临时变量
	memcpy(arr_tmp, arr, arr_len * sizeof(int32_t)); //把arr 拷贝到arr_tmp中

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

	for (i = 0; i < PULSE_BPM_SAMPLE_SIZE; i++) {
		if (((arr_tmp[PULSE_BPM_SAMPLE_SIZE - 1]) * 1.0 / arr_tmp[i]) < 1.6) {
			break;
		}
	}

	return tmp;
}


/**********************************************************************************
* 函数名称： checkDeviation()
* 功能描述：判断两点之间的幅值差是否符合要求（收缩期幅值）
* 版    本：v1.0
* 参数说明：top:峰值点幅值  bottom 峰值点后连续下降的第6个点的幅值  time :周期点数
* 返回说明： 无
* 说    明：
**********************************************************************************/

int32_t checkDeviation(int32_t top, int32_t bottom, int32_t time)
{
	int32_t diff;

	diff = abs(top - bottom);
	deviationBuf[deBufCnt++] = diff;

	// 每保存满5个更新一次阈值  问题：如果中间长时间未存阈值还准么？
	if(deBufCnt >= PULSE_BPM_SAMPLE_SIZE){
		deBufCnt = 0;
		deviationLimit = countNewDevLimit();
	}

	if( (diff > deviationLimit*0.6 && diff < deviationLimit*3) || (deviationLimit==0) ){    //幅值在 [0.6*min,3*min] 内
		return 1;
	}

	return 0;
}

/**********************************************************************************
* 函数名称：detectPulseBySlop
* 功能描述：计算周期性极大值点处的脉率（Pr）
* 版    本：v1.0
* 参数说明：
* 返回说明：返回值ret  =bpm*100
* 说    明：每个采样点调用
**********************************************************************************/
int16_t detectPulseBySlop(SPO2Parameter *parameter)
{

	int16_t ret = 0;                        // 输出bpm*100
	int32_t value;                          // 当前幅值
	int32_t beatDuration = 0;               //计算两个峰值点之间的距离

	if(parameter == NULL)
		return -1;

	value = parameter->sensorValue;

	step_cnt++;                           //uint16_t 0-65536 没有清零机制，连续测量21min之后会不会有溢出风险
	currentSlop = value - preSensorValue;

	if ((currentSlop < 0) && (preSlop1 >= 0) && (slopFlag == 0) && value>0) // 峰值点 注：只有在进行去除飘移，使信号在0上下波动的时候，才能使用限制条件value>0
	{
		//如果光强变化了，阈值是不是应该调整?        防止突变点
		if (((abs(preSlop1) > DATA_BASE_DIFF_MIN) && (abs(preSlop1) < DATA_BASE_DIFF_MAX))
			|| ((abs(preSlop2) > DATA_BASE_DIFF_MIN) && (abs(preSlop2) < DATA_BASE_DIFF_MAX))) { //no connect human judge

			currentBeatIdx = step_cnt;  // 记录当前位置
			currentValue = value;
			slopFlag = 1;                //峰值点处，
			systoleCnt = 0;

		}
	}

	//连续下降计数--收缩期计时
	if ((slopFlag == 1) && (currentSlop < 0))
	{
		systoleCnt++;                              // 快速流入时间计数清零
	}
	else if ((slopFlag == 1) && (currentSlop > 0))  // 波谷转折点
	{
		systoleCnt = 0;
		slopFlag = 0;
	}

	/***********峰值点确定之后连续下坡6个点处位置  计算对应的PR值 *******/
	if (systoleCnt >= SYSTOLE_THL)    //流入时间>=6/50=0.12s (单位s)，一般流入时间[0.2 0.5]*T (s)
	{

		slopFlag = 0;             //? 该处对么？如果斜坡还继续下降呢？2021.12
		systoleCnt = 0;

		if (isFirstPulse)        // 第一个波
		{
			lastBeatIdx = currentBeatIdx;
			isFirstPulse = 0;    // 非第一个波
			//slopFlag = 0;      //? 该处对么？如果斜坡还继续下降呢？
			//systoleCnt = 0;    //流入时间清零
		}
		else  // 非第一个波
		{
			beatDuration = currentBeatIdx - lastBeatIdx; //计算两个峰值点之间的距离

			//幅值与周期检测
			uint8_t checkDeviation_flag = checkDeviation(currentValue, value, beatDuration);
			if (checkDeviation_flag) //检查幅值偏置是否>limit
			{
				if ((beatDuration <= RR_UPPER) && (beatDuration > RR_LOWER)) //fs = 50hz,对应bpm = [40,250]，如果为了省计算量，可以如此写，如果为了不同采样率下通用，可以换算成BPM
				{
					lastBeatIdx = currentBeatIdx;
					int16_t rawBPM = 0;
					if (beatDuration > 0)
					{
						rawBPM = BASE_PR * ppgSample * 100 / ((float)beatDuration);// ppgSample = 1??? 初始化的时候又给ppgSample重新赋值了50hz,见initDetectPulseBySlop(int sampleRate)
					}
					ret = rawBPM ;   // 实时计算，结果未经过滤波的  +0.5 为了四舍五入
					//printf("%d\t%d\n", step_cnt - SYSTOLE_THL, ret);
				}
			}
			else
			{
				//printf("peak_idx = %d  (amp = %d )< (deviationLimit = %d)\n", step_cnt - SYSTOLE_THL, currentValue - value, deviationLimit);
			}

			//isFirstPulse = 0;//不是第一个
			//slopFlag = 0;
			//systoleCnt = 0;

			//pr<40BPM
			if (beatDuration > RR_UPPER) {
				lastBeatIdx = currentBeatIdx;
				//isFirstPulse = 1;  //第一个点（重新开始计）
			}
		}
	}

	preSlop2 = preSlop1;
	preSlop1 = currentSlop;
	preSensorValue = value;

	return ret;
}


/**********************************************************************************
* 函数名称： checkPrValue
* 功能描述： 当前pulserate与前其他4个值进行比较（为什么不是5个值，因为有一个值是他自己），存在2个以上15bpm的，说明该值无效
* 版    本： v1.0
* call  by： calulatePr()
* 参数说明：
* 返回说明：
* 说    明：  数组中有2个以上0的时候，返回值肯定为0，增加了calulatePr计算结果的延时
* **********************************************************************************/
int8_t checkPrValue(int32_t pr)
{
	int32_t i = 0;
	int32_t cnt = 0;

	for(i = 0; i < PULSE_BPM_SAMPLE_SIZE; i++){

		if((abs(pr - valuesBPM[i])) > HR_SHAKE_THL){
			cnt++;
		}
	}

	if(cnt >= 2){
		return 0;
	}

	return 1;
}



/**********************************************************************************
* 函数名称：calulatePr
* 功能描述：计算每一个点的pr ,后期处理：缓存5个计算值，判断然后平均
* 参数说明：
* 返回说明：
* 说    明：
**********************************************************************************/
int32_t  calulatePr(const int32_t *rawPr, int32_t dataSize)
{
	int32_t i, j;
	int32_t sumBPM = 0;
	int32_t ret = -1;
	int32_t cnt = 0;

	if(rawPr == NULL || dataSize <= 0){
		return ret;
	}

	for(i = 0; i < dataSize; i++){
		if((rawPr[i] >= 30) && (rawPr[i] <= 250)){
			//fprintf(stdout,"get hr is %d\n", rawPr[i]);

			//获取到5个峰值之后才开始缓存valueBPM
			if(getHrCnt > 5){
				valuesBPM[BPMCnt++] = rawPr[i];        //save
				cnt = 0;
				for(j = 0; j < PULSE_BPM_SAMPLE_SIZE; j++)    // 当前valuesBPM[]包含了当前的pr,判断缓存中每个值的有效性
				{
					if((valuesBPM[j] != 0) && (checkPrValue(valuesBPM[j])))
					{
						sumBPM += valuesBPM[j];
						cnt++;
					}
				}
				if(BPMCnt >= PULSE_BPM_SAMPLE_SIZE)   //没有按时间顺序存储
					BPMCnt = 0;

				if(cnt >= 3)                          //缓存中有3个以上的值有效，求有效值的均值作为当前输出值
					ret = (sumBPM / (float)cnt);
			}
			else{
				getHrCnt++;
			}
		} // end if((rawPr[i] >= 30) && (rawPr[i] <= 250))
	}

#if 1
    //当前不是峰值点，或者是峰值点计算出的结果不符合以上的条件，保持4s，4s后清零
	if(ret <= 0){
		hrCnt++;                                        //计算不出bpm的点计数，保持计时
		if(hrCnt >= (ppgSample * COUNT_HR_TIME_OUT)){   //保持4s
			prePr = 0;
			hrCnt = 0;
			valuesBPM[clearCnt++] = 0;
			if(clearCnt >= PULSE_BPM_SAMPLE_SIZE)
				clearCnt = 0;
		}
		ret = prePr;  //保持上一个结果，保持COUNT_HR_TIME_OUT时长，该处为4s
	}else{
		hrCnt = 0;
	}
#endif

	prePr = ret;

	return ret;
}


/*
* 连续5个abs(当前值-上一个值) > 60 , 输出1
*/
int32_t judgeHrQuality(int32_t hr)
{
	int32_t ret = 0;

	if(hr == 0)
		return ret;

	if(judge_pre_hr != -1){
		if(abs(judge_pre_hr - hr) >  UNVALUE_HR_SHAKE)
		{
			hr_shark_cnt++;
			shark_step = 0;
			//printf("hr_shark_cnt is %d\n", hr_shark_cnt);
		}

		if(hr_shark_cnt > SHAKE_HR_MAX_TIMES)
			ret = 1;
	}

	shark_step++;

	if(shark_step > 5)
		hr_shark_cnt = 0;

	judge_pre_hr = hr;

	return ret;
}
