/*****************************************************************************
* @file         alg_ComFunc.c
* @author       wx_John
* @version      V1.0
* @date         2021-12-13
* @brief
*--------------------------------------------------------------------*
* Change History :
* <Data>      |<Version> |<Author>       | <Description>
*--------------------------------------------------------------------*
*  2021-12-13 |1.0       |wx_John         |
* ----------------------------------------------------------------
*****************************************************************************/

#include "alg_ComFunc.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

/**********************************************************************************
* Function Name:
* Purpose      : 均值 函数模板--整型
* Parameter    : type Data[]：输入数组, type NL：数组开始点, type NU：结束)
* return       :
**********************************************************************************/
#define MEAN_FUN_Implement(type)\
	type type##_mean( type Data[], type NL, type NU)\
	{\
		type i, meanData = 0;\
		int32_t sumData = 0;\
		for( i = NL; i < NU; i++ )\
			sumData += Data[i];\
		meanData = (type)(sumData / (NU - NL));\
		return meanData;\
	}
MEAN_FUN_Implement(int16_t);       //实现了 int16_t_mean(int16_t Data,int6_t NL,int16_t NU)函数
MEAN_FUN_Implement(uint16_t);
MEAN_FUN_Implement(uint8_t);

/**********************************************************************************
* Function Name:
* Purpose      : 标准差 函数模板
* Parameter    : type Data[]：输入数组, type NL：数组开始点, type NU：结束)
* return       :
**********************************************************************************/
#define STD_FUN_Implement(type)\
	type type##_std( type Data[], type NL, type NU)\
	{\
		type i, meanData = 0;\
		int32_t sumData = 0;\
		for( i = NL; i < NU; i++ )\
			sumData += Data[i];\
		meanData = (type)(sumData / (NU - NL));\
		sumData = 0;\
		for( i = NL; i < NU; i++ )\
			sumData += (Data[i] - meanData) * (Data[i] - meanData);\
		meanData = (type)sqrt(sumData * 1.0 / (NU - NL - 1));\
		return meanData;\
	}
STD_FUN_Implement(int16_t);  //实现了 int16_t_std(int16_t Data,int6_t NL,int16_t NU)函数
STD_FUN_Implement(uint16_t);


/**********************************************************************************
* Function Name:
* Purpose      : 降序排列 函数模板 实例化时采用两种方式 其中之一  ,2中方法结果都正确
* Parameter    :
* return       :
**********************************************************************************/
#define DESCEND_FUN_Implement( type )\
	type* type##desfun( type Data[] )\
	{\
		uint8_t j, k;\
		type tmps;\
		type ams[3] = {Data[0], Data[1], Data[2]};\
		type *Ttmp = ams;\
		for(j = 0; j < 3; j++ )\
		{\
			for( k = j + 1; k < 3; k++ )\
			{\
				if( Ttmp[k] > Ttmp[j] )\
				{\
					tmps = Ttmp[j];\
					Ttmp[j] = Ttmp[k];\
					Ttmp[k] = tmps;\
				}\
			}\
		}\
		return Ttmp;\
	}
#define call_desfun(type) type##desfun
DESCEND_FUN_Implement(uint8_t);
DESCEND_FUN_Implement(uint16_t);


/************************************************************************************
函数名：void quick_sort(int16_t s[], uint8_t l, uint8_t r)
功  能：从小到大排列
参  数：
返回值：
************************************************************************************/
void quick_sort(int16_t s[], uint8_t l, uint8_t r)
{
    if (l<r)
    {
        uint8_t i = l, j = r;
        int16_t x = s[l];
        while (i<j)
        {
            while (i<j && s[j] >= x) // 从右向左找第一个小于x的数
                j--;
            if (i<j)
                s[i++] = s[j];
            while (i < j && s[i] < x) // 从左向右找第一个大于等于x的数
                i++;
            if (i<j)
                s[j--] = s[i];
        }
        s[i] = x;
        quick_sort(s, l, i); // 递归调用
        quick_sort(s, i + 1, r);
    }
}

/************************************************************************************
函数名：fifopush_u8(uint8_t data, uint8_t s[], uint16_t size)
功  能：把data压入到数组大小为size的数组S[]中 从左到右：新--旧
参  数：
返回值：
************************************************************************************/
void fifopush_u8(uint8_t data, uint8_t s[], uint16_t size)
{
    int16_t i;
    for (i = size - 1; i>0; i--)
        s[i] = s[i - 1];
    s[0] = data;
}

void fifopush_s16(int16_t data, int16_t s[], uint8_t size)
{
    uint8_t i;
    for (i = size - 1; i>0; i--)
        s[i] = s[i - 1];
    s[0] = data;
}

void fifopush_int32(int32_t data, int32_t s[], uint8_t size)
{
    uint8_t i;
    for (i = size - 1; i>0; i--)
        s[i] = s[i - 1];
    s[0] = data;
}

void fifopush_f(float data, float s[], uint8_t size)
{
    uint8_t i;
    for (i = size - 1; i>0; i--)
        s[i] = s[i - 1];
    s[0] = data;
}

/************************************************************************************
* 函数名：float scatter_avg_s16(int16_t s[], uint8_t size)
* 功  能：评价数据离散程度
* 参  数：
* 返回值：评价数据离散程度的数，0代表全相等，1代表极端离散
************************************************************************************/
float scatter_avg_s16(int16_t s[], uint8_t size)
{
    uint8_t i;
    int32_t avg = 0;
    float tmp = 0;
    float scatter = 0;

    for (i = 0; i < size; i++)
    {
        avg += s[i];
    }
    avg = (avg + size / 2) / size;

    for (i = 0; i < size; i++)
    {
        //		tmp = (avg + s[i]) * (avg - s[i]);
        tmp = avg - s[i];
        tmp = ABS(tmp);
        scatter += tmp;
    }
    //	scatter = sqrt(scatter/size)/ (float)avg;
    //	scatter = (scatter/size)/ (float)avg; //更改成一下形式，因为avg有为0的时候 2017-10-25
    //  BUG:分母可能为0
    if (avg)
        scatter = (scatter / size) / (float)avg;
    else
        scatter = 100;

    return scatter;
}


/******************************************************************************
* 函数名称：int16_t victor_filer_s16(int16_t s[], uint8_t size, uint8_t r, float *pscatter)
* 功能描述：先对大小为size*r的数组排序，滑动窗计算离散度小的小窗内均值
* 参数说明：
* 返回说明：离散度最小的均值
* 说    明：
**********************************************************************************/
int16_t victor_filer_s16(int16_t s[], uint8_t size, uint8_t r, float *pscatter)
{
    int16_t	ss[8];
    uint8_t i, x;
    uint8_t point = 0;
    int32_t avg = 0;
    float scatter = 0;
    float scatterb = 0;

    /* memcpy
    用于把资源内存s拷贝到目标内存ss,拷贝的个数用size控制,拷贝的是字节数（一个字节一个字节拷贝）
    * 因此，实际应用的时候大小应该转化成字节数
    */
    memcpy(ss, s, size * sizeof(int16_t));  //string.h

    x = (size * r + 50) / 100;/*求占比*/
    quick_sort(ss, 0, size - 1);/*传进来的数组进行排序*/
    scatterb = scatter_avg_s16(&ss[0], x);//数据离散程度

    for (i = 1; i < size + 1 - x; i++)/*求差值以及对应坐标*/
    {
        scatter = scatter_avg_s16(&ss[i], x);
        if (scatterb > scatter)
        {
            point = i;
            scatterb = scatter;
        }
    }
    for (i = 0; i < x; i++)
    {
        avg += ss[point + i];
    }
    avg = (avg + x / 2) / x;
    *pscatter = scatterb;
    return (int16_t)avg;
}

uint8_t victor_filer_u8(uint8_t s[], uint8_t size, uint8_t r, float *pscatter)
{
    uint8_t i;
    int16_t tmp_arr[8];
    for (i = 0; i<size; i++)
        tmp_arr[i] = s[i];
    return (uint8_t)victor_filer_s16(tmp_arr, size, r, pscatter);
}


