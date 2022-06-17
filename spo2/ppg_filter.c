#include "ppg_filter.h"
#include <stdint.h>
#include "filter.h"

// judgePpgDataQuality() parameters
#define JUDGE_START_STEP     50
#define JUDGE_MAX_STEP       200
#define SINGAL_MAX_OFFSET    10000  // 与幅值有关系

static int32_t maxData;
static int32_t minData;
static uint8_t dataStep;

void initPpgFilter(void)
{
    init_filter();  //--Asd 可以拿出来单独初始化

    maxData = 0;
    minData = 0x7FFFFFFF;
    dataStep = 0;
}

int32_t  ppgFilter(SPO2Parameter *parameter)
{
	parameter->sensorValue = filter(parameter->sensorValue);

	return 0;
}

/**********************************************************************************
* 函数名称：judgePpgDataQuality(int32_t value)
* 功能描述：通过最大最小之间的幅值差，来判断数据质量 点[50 200]之间 AMP(最大值-最小值)<10000,说明数据质量差？  原始版本的佩戴识别？
* 参数说明：
* 返回说明：
* 说    明： 1. 每计数到200点，重新计算，也就是每4s判断一次数据质量，中间的1s不需要判断？
**********************************************************************************/

int32_t  judgePpgDataQuality(int32_t value)
{
	int32_t ret = 0;

	dataStep++;
	if(dataStep > JUDGE_START_STEP) //从51个采样点开始判断
	{
		if(value > maxData)
			maxData = value;

		if(value < minData)
			minData = value;
	}
	if(dataStep > JUDGE_MAX_STEP)  // 第200个采样点做判断
	{
		if ((maxData - minData) < SINGAL_MAX_OFFSET)
		{
			ret = 1;
		}

		dataStep = 0;
		maxData = 0;
		minData = 0x7FFFFFFF;
	}

	return ret;
}


