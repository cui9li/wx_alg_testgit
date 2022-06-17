/*****************************************************************************
* @file         algSmartView.c
* @author       wx_John
* @version      V1.0
* @date         2022-03
* @brief        1. 抬手亮屏；反转手腕灭屏
*
*--------------------------------------------------------------------*
* Change History :
* <Data>      |<Version> |<Author>       | <Description>
*--------------------------------------------------------------------*
*  2022-03-03 |1.0       |wx_John        |
*  2022-03-11 |1.1       |wx_John        |角度范围判断以前用0.4s的均值，现在用0.1s数据，相对会灵敏
*  2022-04-07 |1.2       |wx_John        |摇一摇过后，有时会出现快速切换到另一屏的现象，修改
* ----------------------------------------------------------------
*****************************************************************************/

#include "algSmartView.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "../alg_ComFunc.h"

#define L_LEN				   20
#define FORWARD_N              8
#define Mpy_Sign(X, Y)		   ((X) * (Y))
#define MIN(X, Y)		       ((X) > (Y) ? (Y) : (X))
#define SquareOfDiff(X, Y)     (((X)-(Y))*((X)-(Y)))

/* 摇一摇中每次检测的信息 */
typedef struct
{
	uint8_t gFlgMaxMin;  // 1:代表极小值  2：代表极大值
	uint8_t gPreMaxMin;  // 上一个标志位

	uint32_t preIndex;   // 上一个峰值点的位置
	int16_t preVaule;    //上一个峰值点的值
	int16_t MaxVaule;    // 峰值点幅值的和
	uint8_t A;           //峰值点个数
	uint8_t gCntMax;     //峰值点个数（未排除干扰点）

	uint32_t preMinIndex;// 上一个极小值点的位置
	int16_t preMinVaule; //上一个极小值点的值
	int16_t MinVaule;    //极小值点幅值的和
	uint8_t I;           // 极小值点个数

	int16_t gShakeStd;   // 标准差 [2:6]

	uint8_t cntShakeSW;  //
	uint8_t gExterNum;   // 简易峰值点（分别大于前后2个点）计数
	uint8_t gFlg;        // 摇一摇标志位

} ParShake;              //Parameters of shack

/* ***摇一摇中涉及到的阈值*** */
typedef struct
{
	int16_t maxVauleFirst;
	int16_t maxVauleSecond;
	int16_t minVauleFirst;
	int16_t minVauleSecond;
	int16_t maxMinVaule;
	int16_t absVaule;
	int16_t diffTime;
} stThsold;

/* angle_detect limit by acc3D*/
typedef struct
{
	int16_t x_lower ;  /* acc_x lower limit*/
	int16_t x_upper ;  /* acc_x upper limit*/
	int16_t y_lower;
	int16_t y_upper;
	int16_t z_lower;
	int16_t z_upper;
} acc3D_LIMITS;

uint8_t Beat_flag = 0;       //软件置位Beat_flag： 当切换屏幕的时候 Beat_flag = 1
uint8_t Up_flag = 0;         //亮屏标志位

//fUpBeatWatch()
int16_t cntUpBeat = 0;       // 两次亮屏间隔        fUpBeatWatch() Up_flag=1的连续计数
uint8_t CntUpEnd = 0;        //fUpBeatWatch() 亮屏状态，不在角度范围内的计数，超过0.7s就灭屏

int16_t Avr_x[L_LEN] = { 0 };//每次传入加速度的均值average,L_LEN=20 保存2s 每0.1s的xyz轴的均值
int16_t Avr_y[L_LEN] = { 0 };
int16_t Avr_z[L_LEN] = { 0 };

//brightViewByBeatShack()
int32_t std_x = 0;           //每次传入5点数据的方差(此处是var*n)   Up_Hand()  fupWatch() stopflat()
int32_t std_y = 0;
int32_t std_z = 0;

int16_t Data_x = 0;          //前一次传入数据的最后一个点(用于前后点的差分)
int16_t Data_y = 0;
int16_t Data_z = 0;

// fupWatch 抬手信号
uint8_t CntView = 0;         //角度符合抬手亮屏的计时 fupWatch()
uint8_t CntViewKeep = 0;     //角度保持时间总时间 fupWatch()
uint8_t L_Cnt2 = 0;          //抬手状态计数        fupWatch/fupWaist()
uint8_t L_dey_Cnt2 = 0;      //抬手看角度保持时间
//acc3D_LIMITS angle_limits = {-350,350,-958,300,200,980};  /* Up_Angle_Detect() ---orignal */
acc3D_LIMITS angle_limits = { -350,350,-958,300,200,1100 };  /* Up_Angle_Detect() */

int16_t upStdx = 0;          //抬手时刻方差        stopflat()
int16_t upStdy = 0;

uint8_t Shake_up_dey = 0;    //敲击亮摇一摇信号标志位    fupWatch() 中计算 相邻2点做差，|3轴两两相邻点的差值|有一个>700,shake_up_dey =1
uint8_t Cnt_shake_dey = 0;   //敲击摇一摇信号标志 持续时间  Shake_up_dey=1的计数

//uint8_t Beat_flag = 0;     //换屏标志位      fbeat()  stopflat() fUpBeatWatch()
//uint8_t Beat_cnt = 0;      //亮屏时间计数    fbeat()，若是碰到换屏信号，重新开始计数
//uint8_t After_flag = 0;    //换屏轻敲标志    fbeat()中计算(diff_x^2+  diff_y^2 +diff_z^2)>63530 && 静止（avg_x<300&&avg_y<150）
//uint8_t Shake_dey = 0;     //换屏重敲标志    fbeat()中计算 方法：|3轴两两相邻点的差值|有一个>500 ,即为重敲

/*******摇一摇看表shakeWatch() 相关变量************************/

stThsold smartViewTh={100,10,-50,-10,1800,100,10}; // 摇一摇亮屏的阈值

uint8_t gFlgShakeView = 0;    //摇一摇亮屏标志位
uint8_t cntShakeView = 0;     // 摇一摇亮计数

ParShake yParShake;           //y轴的极大/小值峰值、位置、个数等等特征
ParShake xParShake;
ParShake zParShake;

uint32_t secShake=0;          // 调用摇一摇计数（1天 = 24*60*60/0.1=864000 （单位0.1s）>2**16 ,因此用32位）

int16_t gXData[9]={0};        //x 9个采样点数组  shakeWatch()
int16_t gYData[9]={0};
int16_t gZData[9]={0};

uint8_t gCntStd=0;           // xyz标准差都<20的计数(连续)
uint8_t gTimeStd=0;          // 3轴std>150 计数（连续的）
uint8_t gTimeXYZ=0;          // 一组数据中只要有一轴符合摇一摇的条件就计数

/*********end shakeWatch()***************/


/**********************************************************************************
* Function Name: clcbeat
* Purpose      : 变量清0
* Parameter    :
* return       :
**********************************************************************************/
void clcbeat(void)
{
    int16_t i = 0;

	Up_flag = 0;

	for(i = 0; i < L_LEN; i++)
	{
		Avr_x[i] = 0;
		Avr_y[i] = 0;
		Avr_z[i] = 0;
	}

	std_x = 0;
	std_y = 0;
	std_z = 0;
	Data_x = 0;
	Data_y = 0;
	Data_z = 0;

	cntUpBeat = 0;   // 两次亮屏间隔        fUpBeatWatch() Up_flag=1的连续计数
	CntUpEnd = 0;    //fUpBeatWatch() 亮屏状态，不在角度范围内的计数，超过0.7s就灭屏

	CntView = 0;     //角度符合抬手亮屏的计时 fupWatch()
	CntViewKeep = 0; //角度保持时间总时间 fupWatch()
	L_Cnt2 = 0;      //抬手状态计数        fupWatch/fupWaist()
	L_dey_Cnt2 = 0;  //抬手看角度保持时间

	//fbeat()
	//Beat_cnt = 0;
	//After_flag = 0;
	//Shake_dey = 0;

	//brightViewByBeatShack()
	Shake_up_dey = 0;    //敲击亮摇一摇信号标志位    fupWatch() 中计算 相邻2点做差，|3轴两两相邻点的差值|有一个>700,shake_up_dey =1
	Cnt_shake_dey = 0;   //敲击摇一摇信号标志 持续时间  Shake_up_dey=1的计数

	upStdx = 0;
	upStdy = 0;


}

/**********************************************************************************
* Function Name:
* Purpose      : 亮屏信号置位，变量清0
* Parameter    :
* return       :
**********************************************************************************/

void up_hand_clc(void)
{
	CntView = 0;         //角度延时
	L_dey_Cnt2 = 0;      //抬手看角度保持时间
	L_Cnt2 = 0;          //抬手状态计数

	Shake_up_dey = 0;    //敲击亮摇一摇信号标志位
	Cnt_shake_dey = 0;   // 敲击摇一摇信号标志 持续时间
}

void upclc(void)
{
    Up_flag = 1;          // Up_flag置1时，一般下边的相关变量需要清理

	up_hand_clc();       //亮屏后，初始化变量*/
    upStdx = std_x;      //抬手时刻方差保存
    upStdy = std_y;
}




/**********************************************************************************
* Function Name: Up_Angle_Detect
* Purpose      : 是否在看表角度
* Parameter    :
* return       :
**********************************************************************************/
uint8_t Up_Angle_Detect(int16_t XMean, int16_t YMean, int16_t ZMean,acc3D_LIMITS angle_limits)
{
	/*
    ** 角度：x与水平的夹角（-20 20） y与水平的夹角（17 62） z与垂直方向的夹角（0 78）;
    ** sleep_mode 情况下的参数确定是根据采集斜靠椅背，平躺抬手较高，平躺抬手较低综合考虑y+z的值
    */
	if ( (XMean>angle_limits.x_lower && XMean<angle_limits.x_upper)
		&& (YMean>angle_limits.y_lower && YMean<angle_limits.y_upper)
		&& (ZMean>angle_limits.z_lower && ZMean<angle_limits.z_upper)
		)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**********************************************************************************
* Function Name:
* Purpose      :
* Parameter    :
* return       :
**********************************************************************************/

uint8_t IsBeatShake(int16_t *data_x, int16_t *data_y, int16_t *data_z, int16_t len, int16_t strength)
{
	int16_t i = 0;
	uint8_t flag = 0;

	for (i = 0; i<len; i++)
	{
		if ( (i == 0 &&(Data_x || Data_y|| Data_z)&&((abs(Data_x - data_x[0]) >strength) || (abs(Data_y - data_y[0]) >strength) || (abs(Data_z - data_z[0]) > strength))) \
			|| (i > 0 &&(data_x[i - 1]|| data_y[i - 1]|| data_z[i - 1])
				&& ((abs(data_x[i] - data_x[i - 1]) > strength) || (abs(data_y[i] - data_y[i - 1]) > strength) || (abs(data_z[i] - data_z[i - 1]) > strength))))
		{
			flag = 1;
			break;
		}
	}
	return flag;
}

/**********************************************************************************
* Function Name: brightViewByBeatShack()
* Purpose      : 相邻2点做差，只要有1轴存在abs(差值)>阈值的情况，shake_up_dey =1
* Parameter    :
* return       : Up_flag
**********************************************************************************/

void brightViewByBeatShack(int16_t *data_x, int16_t *data_y, int16_t *data_z)
{

	int16_t XMean = (Avr_x[L_LEN - 1] + Avr_x[L_LEN - 2] + Avr_x[L_LEN - 3] + Avr_x[L_LEN - 4]) >> 2; // 0.4s均值
	int16_t YMean = (Avr_y[L_LEN - 1] + Avr_y[L_LEN - 2] + Avr_y[L_LEN - 3] + Avr_y[L_LEN - 4]) >> 2;
	int16_t ZMean = (Avr_z[L_LEN - 1] + Avr_z[L_LEN - 2] + Avr_z[L_LEN - 3] + Avr_z[L_LEN - 4]) >> 2;

	if (Shake_up_dey == 1)       //相邻2点做差，只要有1轴存在abs(差值)>700,shake_up_dey =1
	{
		Cnt_shake_dey++;         // shake_up_dey的计数
		if (Cnt_shake_dey > 50)  //敲击摇一摇标志产生后5s内没有停下来，认为是干扰，清0标志量
		{
			Shake_up_dey = 0;
			Cnt_shake_dey = 0;
		}
		else
		{
			// 静止状态检测部分+敲击信号
			if (abs(XMean) < 300 && YMean < -150 && ZMean > -800)
			{
				if ((std_z < 700) && (std_y < 800) && (std_x < 800))
				{
					upclc();      //    亮
				}
				if ((std_z>700) && (std_y < 2500) && (std_x<2500))
				{
					upclc();     //     亮
				}
			}
			else
			{
				Up_flag = 0;
			}
		}
	}//end if Shake_up_dey == 1
	else //  Shake_up_dey == 0
	{
		/* 计算敲击摇一摇信号是否产生：相邻2点做差，只要有1轴存在abs(差值)>700,shake_up_dey =1 */
		if (IsBeatShake(data_x,data_y,data_z,5,700))
		{
			Shake_up_dey = 1;
		}

	}// end  if(Shake_up_dey == 1) else

}

/**********************************************************************************
* Function Name: fupWatch
* Purpose      : 手腕模式抬手看表
* Parameter    : *data_x,*data_y,*data_z 加速度数据，长度为5，int16_t类型
* return       : Up_flag==1抬手亮屏信号
**********************************************************************************/
void fupWatch(int16_t *data_x, int16_t *data_y, int16_t *data_z)
{

	int32_t sum_xyz_squarediff = 0;

	/****************角度检测部分 ***************************/
	if( Up_Angle_Detect(Avr_x[L_LEN-1], Avr_y[L_LEN - 1], Avr_z[L_LEN - 1], angle_limits) )
	{
		CntView++;
		CntViewKeep++;
		CntView = MIN(CntView, 200);
		CntViewKeep = MIN(CntViewKeep, 200);
		if(CntViewKeep > 4)	//超过0.3s仍保持这个状态，则不再产生抬手信号，只产生一次
		{
		    CntView = 0;	//若保持一个角度始终不变，灭屏后即不再亮屏
		}
	}
	else
	{
		CntView = 0;
		CntViewKeep = 0;    //动一下 会重新计数

	}
	/*************变化过程检测部分 当前与1s前的变化来判断*************************/
	if( Up_Angle_Detect(Avr_x[L_LEN - 1], Avr_y[L_LEN - 1], Avr_z[L_LEN - 1],angle_limits) )
	{
		//计算状态变化量
		if (Avr_x[L_LEN - FORWARD_N] || Avr_y[L_LEN - FORWARD_N] || Avr_z[L_LEN - FORWARD_N])
		{
			sum_xyz_squarediff = SquareOfDiff(Avr_x[L_LEN - FORWARD_N], Avr_x[L_LEN - 1]) \
				+ SquareOfDiff(Avr_y[L_LEN - FORWARD_N], Avr_y[L_LEN - 1]) \
				+ SquareOfDiff(Avr_z[L_LEN - FORWARD_N], Avr_z[L_LEN - 1]);
		}

		if(sum_xyz_squarediff > 300000)
		{
			L_Cnt2++;                       // 抬手过程计数（连续）
			L_Cnt2 = MIN(L_Cnt2, 254);      //防溢出
		}
		else
		{
			L_Cnt2 = 0;
		}

		if(L_Cnt2 >= 1)
		{
			L_dey_Cnt2++;
		}
		else
		{
			L_dey_Cnt2 = 0;
		}
	}

    /*此段判断程序表明：手部时只用到了角度，睡眠时用到了角度抬手过程*/
	if(CntView==3 && L_dey_Cnt2>=3)  //保持时间   if(CntView==3 || L_dey_Cnt2>=3)
	{
		upclc();                     //亮屏 并对亮屏后的变量清零
	}
	else
	{
		Up_flag=0;
	}

	///**********敲击\摇一摇亮屏检测部分 *********/
	//if(Up_flag == 0)
	//{
	//	brightViewByBeatShack(data_x, data_y, data_z);
	//}

}

/**********************************************************************************
* Function Name: stopflat()
* Purpose      : 摇一摇后静止检测程序
* Parameter    :
* return       :
**********************************************************************************/

/*
void stopflat()
{
	//换屏检测I  摇一摇停下来的方差跟亮屏时刻方差比较
	if((abs(Avr_x[L_LEN-1]) < 300) && (Avr_y[L_LEN-1] < 150) \
			&& (std_y < upStdy + 2000) && (std_x < upStdx + 2000))
	{
		Beat_flag = 1;		//换屏标志位置1
	}
	else
	{
		Beat_flag = 0;
	}
	//换屏检测II
	if(Beat_flag == 0)
	{
		if(std_z < 700)
		{
			 if((abs(Avr_x[L_LEN-1]) < 300) && (Avr_y[L_LEN-1] < 150)
						&& (std_y < 800) && (std_x < 800))
			 {
				Beat_flag = 1;
				if(std_y > upStdy + 500)
				{
					upStdy=std_y;	//换屏后更新upStdy
				}
				if(std_x > upStdx + 500)
				{
					upStdx=std_x;
				}
			 }
			 else
			 {
				Beat_flag=0;
			 }
		}
		else
		{
			if((abs(Avr_x[L_LEN-1]) < 300) && (Avr_y[L_LEN-1] < 150) \
					&& (std_y < 2500) && (std_x < 2500))
			{
				Beat_flag = 1;
				if(std_y > upStdy + 500)
				{
				  upStdy = std_y;
				}
				if(std_x > upStdx + 500)
				{
				  upStdx = std_x;
				}
			}
			else
			{
			  Beat_flag = 0;
			}
		}
	}
}
*/

/**********************************************************************************
* Function Name: fbeat
* Purpose      : 敲击摇一摇换屏--原版
* Parameter    : *data_x,*data_y,*data_z 加速度数据，长度为5，int16_t类型
* return       : Beat_flag==1换屏信号
**********************************************************************************/
//void fbeat(int16_t *data_x, int16_t *data_y, int16_t *data_z, uint8_t time)
//{
//  uint32_t diff_sum_three = 0;
//  int16_t i = 0;
//  uint16_t tmp = 0;
//
//  if(Beat_cnt < time)
//  {
//    Beat_cnt++;      //计算亮屏时间
//    if(Beat_cnt > 1)
//    {
//    /*
//    ** 1.若没有切屏标志&&和重敲的情况下：
//    **      若有轻敲，根据停下来的角度及停下来的标准差与前边比较看是否切屏；
//    **      若没有轻敲，去判断是否有轻敲（方法：两两相邻点差分的平方和 是否大于阈值&xy是否在某角度）
//	*/
//      if(Beat_flag == 0 && Shake_dey == 0)
//      {
//        if(After_flag == 1)
//        {
//          stopflat();  //是否静止
//        }
//        else  //After_flag~=1
//        {
//			//
//          for(i = 0; i < 4; i++)
//          {
//            diff_sum_three = 0;
//            if(i==0)  //20150519 fabs->abs
//            {
//
//				diff_sum_three = SquareOfDiff(Data_x, data_x[0])
//								+ SquareOfDiff(Data_y, data_y[0])
//								+ SquareOfDiff(Data_z, data_z[0]);
//            }
//            else
//            {
//
//				diff_sum_three = SquareOfDiff(data_x[i+1], data_x[i])
//								+ SquareOfDiff(data_y[i+1], data_y[i])
//								+ SquareOfDiff(data_z[i+1], data_z[i]);
//             }
//            if(diff_sum_three > 63530)
//            {
//              if((abs(Avr_x[L_LEN-2]) < 300) && (Avr_y[L_LEN-2] < 150)) //20131010(fabs(Avr_z[L_LEN-2])>700)
//              {                                                      //20131016
//                Beat_flag=0;
//                After_flag=1;
//                break;
//              }
//            }
//            else
//            {
//              Beat_flag=0;
//            }
//          }
//        }
//      }// end if(Beat_flag==0)
//
//      /*
//	    ** 2.若没有切屏标志&&和轻敲的情况下：
//	    **      若有重敲，根据停下来的角度及停下来的标准差与前边比较看是否切屏；
//	    **      若没有重敲，去判断是否有重敲（方法：|3轴两两相邻点的差值|有一个>500 ,即为重敲Shake_dey=1）
//		*/
//      if(Beat_flag==0&&After_flag==0)
//      {
//        if(Shake_dey==1)
//        {
//          stopflat();
//        }//end if(Shake_dey==1)
//        else
//        {
//          for(i=0;i<4;i++)  //20150519 fabs->abs
//          {
//            if((abs(data_x[i+1]-data_x[i])>500)||(abs(Data_x-data_x[0])>500)
//              ||(abs(data_y[i+1]-data_y[i])>500)|| (abs(Data_y-data_y[0])>500)
//                || (abs(data_z[i+1]-data_z[i])>500)||(abs(Data_z-data_z[0])>500))
//            {
//              Shake_dey=1;
//              break;
//            }
//            else
//            {
//              Beat_flag=0;
//            }
//          }  //end for
//        } // end  if(Shake_dey==1)else
//      }
//	  /*
//	  ** 若是换屏信号，换屏，并重新计亮屏时间
//	  */
//      if(Beat_flag==1)
//      {
//        After_flag=0;
//        Shake_dey=0;
//        if(Beat_cnt<7) //两次换屏信号间隔小于0.7s认为是干扰
//        {
//          Beat_flag=0;
//        }
//        else
//        {
//          Beat_flag=1;
//          Up_flag=1; //在现在基础上，将Up_flag=1，重新计数5s
//		  //ViewNumTim = 50;
//          Beat_cnt=0;
//        }
//      }
//    }//Beat_cnt>7
//    else
//    {
//      Beat_flag=0;
//    }
//  }  // if 50
//  else
//  {
//	// Method_flag = Up_flag + 1;
//    Beat_cnt=0;
//    Up_flag=0;
//    Beat_flag=0;
//
//  } // <50的else
//}
//
//
//

/**********************************************************************************
* Function Name: fUpBeatWatch
* Purpose      : 手腕模式子程序
* Parameter    : *data_x,*data_y,*data_z 加速度数据，长度为5，int16_t类型
* return       :
**********************************************************************************/
void fUpBeatWatch(int16_t *data_x, int16_t *data_y, int16_t *data_z, uint8_t time)
{

    uint8_t i = 0;

	if(Up_flag == 0) //灭屏
	{
	    //灭屏时没有换屏，因此有关变量清零
		//After_flag = 0;
		//Beat_flag = 0;
		//Shake_dey = 0;

		fupWatch(data_x, data_y, data_z);                   // 抬手 计算up_flag的值

		if (Up_flag == 0)
		{
			brightViewByBeatShack(data_x, data_y, data_z);  //敲击\摇一摇亮屏检测部分
		}

		if(cntUpBeat < 20 && cntUpBeat > 10)               //两次亮屏间隔要大于 15-5 s
		{
			cntUpBeat = 0;
			Up_flag = 0;                       // 因为上边的判断有可能会让Up_flag = 1,但是如果两次亮屏间隔太小，给Up_flag置零
		}
		else
		{
			cntUpBeat = 0;
		}
	}
	else     // 亮屏 判断灭屏
	{
		cntUpBeat++;
		cntUpBeat = MIN(cntUpBeat, 1000);

        /* 1- 换屏（手动），时间重新计算 */
		//fbeat(data_x, data_y, data_z, time);       //切屏判断(亮屏状态下)
		if (Beat_flag==1)   // beat_flag 切屏 由软件发送
		{
			cntUpBeat = 0;  //亮屏计数清0
			Beat_flag = 0;
		}

		/* 2-亮屏时间>time，灭屏 */
		if(cntUpBeat>time)
		{
			Up_flag = 0;
		}

		/*  3-角度变化--亮到灭屏 */
		if(cntUpBeat > 10 && !Up_Angle_Detect(Avr_x[L_LEN - 1], Avr_y[L_LEN - 1], Avr_z[L_LEN - 1], angle_limits))
		{
			CntUpEnd++;
			if(CntUpEnd > 5)  //超过0.5s不满足角度 才灭屏
			{
			    //是否有从范围内到范围外的角度变换
			    for(i=0;i<L_LEN;i++)
			    {
					if(Up_Angle_Detect(Avr_x[i], Avr_y[i], Avr_z[i], angle_limits))
					{
					    Up_flag = 0;  //灭屏
					    CntUpEnd = 0;
						break;
					}

			    }

			}
		}
		else
		{
			CntUpEnd = 0;
		}

		/* Up_flag 由1-0转折时 清零 */
		if (Up_flag == 0)
		{
			up_hand_clc(); //清零
		}

	}// end if(Up_flag == 0) //灭屏

}

/**********************************************************************************
* Function Name: notActiveBeat()
* Purpose      : 敲击摇一摇不计步
* Parameter    :
* return       :
**********************************************************************************/
//void notActiveBeat()
//{
//	if(Beat_flag==1)     //有敲击摇一摇信号不计步
//	{
//		gBuFlag=1;
//	}
//
//	if(Up_flag==0)
//	{
//		gBuFlag=0;
//	}
//
//}

/**********************************************************************************
* Function Name:
* Purpose      : 摇一摇shakeWatch 相关变量 清0
* Parameter    :
* return       :
**********************************************************************************/
void clcSWatch(void)
{
	uint8_t i;

	xParShake.MaxVaule=0;
	yParShake.MaxVaule=0;
	zParShake.MaxVaule=0;
	xParShake.MinVaule=0;
	yParShake.MinVaule=0;
	zParShake.MinVaule=0;

	xParShake.A=0;
	yParShake.A=0;
	zParShake.A=0;
	xParShake.I=0;
	yParShake.I=0;
	zParShake.I=0;

	xParShake.cntShakeSW=0;
	yParShake.cntShakeSW=0;
	zParShake.cntShakeSW=0;

	xParShake.preIndex=0;
	yParShake.preIndex=0;
	zParShake.preIndex=0;
	xParShake.preVaule=0;
	yParShake.preVaule=0;
	zParShake.preVaule=0;

	xParShake.preMinIndex=0;
	yParShake.preMinIndex=0;
	zParShake.preMinIndex=0;
	xParShake.preMinVaule=0;
	yParShake.preMinVaule=0;
	zParShake.preMinVaule=0;

	xParShake.gCntMax=0;
	yParShake.gCntMax=0;
	zParShake.gCntMax=0;
	xParShake.gExterNum=0;
	yParShake.gExterNum=0;
	zParShake.gExterNum=0;
	xParShake.gFlg=0;
	yParShake.gFlg=0;
	zParShake.gFlg=0;

	xParShake.gFlgMaxMin=0;
	xParShake.gPreMaxMin=0;
	yParShake.gFlgMaxMin=0;
	yParShake.gPreMaxMin=0;
	zParShake.gFlgMaxMin=0;
	zParShake.gPreMaxMin=0;

	/*gSumStdSW=0;
	gTimeSW=0;*/

	secShake=0;
	gTimeXYZ=0;

	for(i=0;i<9;i++)
	{
		gXData[i]=0;
		gYData[i]=0;
		gZData[i]=0;
	}

   gCntStd=0;
   gTimeStd=0;

}

/**********************************************************************************
* Function Name:
* Purpose      : 摇一摇方差计算
* Parameter    :
* return       :
**********************************************************************************/
//int16_t stdSWatch(int16_t *data)
//{
//	uint8_t i=0;
//	int32_t sumData=0;
//	int16_t meanData=0;
//	for(i=0;i<5;i++)
//	{
//	sumData+=data[i];
//	}
//	meanData=sumData/5;
//	sumData=0;
//	for(i=0;i<5;i++)
//	{
//	sumData+=Mpy_Sign((data[i]-meanData),(data[i]-meanData));
//	}
//	meanData=(int16_t)sqrt(sumData/5);
//	return meanData;
//}


/**********************************************************************************
* Function Name: YShakeWatchMax
* Purpose      : 摇一摇看表 y轴判断
* Parameter    :
* return       :
**********************************************************************************/
void YShakeWatchMax(int16_t yStd, uint32_t tmpIndex,uint8_t i,stThsold parThsold)
{

	if( (gYData[i]-gYData[i-1]>parThsold.maxVauleFirst || (gYData[i]>=gYData[i-1] && gYData[i]-gYData[i-2]>parThsold.maxVauleFirst)\
	||(gYData[i]>=gYData[i-1] && gYData[i]-gYData[i-2]>parThsold.maxVauleSecond && gYData[i-1]>=gYData[i-2]))\
	&& (gYData[i]-gYData[i+1]>parThsold.maxVauleFirst || (gYData[i]>=gYData[i+1] && gYData[i]-gYData[i+2]>parThsold.maxVauleFirst)\
		||(gYData[i]>=gYData[i+1] && gYData[i]-gYData[i+2]>parThsold.maxVauleSecond && gYData[i+1]>=gYData[i+2]))\
		&& gYData[i]>parThsold.absVaule)
	{
		yParShake.gShakeStd=yStd;
		yParShake.gCntMax++;
		if(yParShake.gCntMax>254)
		{
			yParShake.gCntMax=254;
		}
		if (tmpIndex+i-yParShake.preIndex>40 && yParShake.preIndex!=0)
		{
			yParShake.MaxVaule=0;
			yParShake.A=0;
		}
		if(tmpIndex+i-yParShake.preIndex>parThsold.diffTime)//10
		{
			yParShake.gFlgMaxMin=2;
			yParShake.MaxVaule+=gYData[i];
			yParShake.A++;
			yParShake.preIndex=tmpIndex+i;
			yParShake.preVaule=gYData[i];
		}
		else
		{
			yParShake.gFlgMaxMin=2;
			if(gYData[i]>yParShake.preVaule)
			{
				yParShake.MaxVaule+=gYData[i]-yParShake.preVaule;
				yParShake.preIndex=tmpIndex+i;
				yParShake.preVaule=gYData[i];
			}
		}
	}
}

/**********************************************************************************
* Function Name: xzShakeWatchMax
* Purpose      : 摇一摇看表 xz轴判断极大判断
* Parameter    :
* return       :
**********************************************************************************/
ParShake xzShakeWatchMax(int16_t *data,ParShake xzParShake,int16_t xzStd,uint8_t i,uint32_t tmpIndex,stThsold parThsold)
{
   // 峰值点，分别与前后2个点进行比较
	if( (data[i]-data[i-1]>parThsold.maxVauleFirst ||(data[i]>=data[i-1] && data[i]-data[i-2]>parThsold.maxVauleFirst))\
	&& (data[i]-data[i+1]>parThsold.maxVauleFirst || (data[i]>=data[i+1] && data[i]-data[i+2]>parThsold.maxVauleFirst))\
	&& data[i]>parThsold.absVaule)
	{
		xzParShake.gShakeStd=xzStd;
		xzParShake.gCntMax++;
		if(xzParShake.gCntMax>254)
		{
			xzParShake.gCntMax=254;
		}

		if (tmpIndex+i-xzParShake.preIndex>40 && xzParShake.preIndex!=0)//当前峰值点的位置与上一个峰值点位置的距离>40 (0.8s)，幅值和和峰值点计数清0
		{
			xzParShake.MaxVaule=0;
			xzParShake.A=0;
		}
		if(tmpIndex+i-xzParShake.preIndex>parThsold.diffTime)//10  //当前峰值点与上衣峰值点的距离>10
		{
			xzParShake.gFlgMaxMin=2;
			xzParShake.MaxVaule+=data[i];
			xzParShake.A++;
			xzParShake.preIndex=tmpIndex+i;
			xzParShake.preVaule=data[i];
		}
		else  // 小于阈值，
		{
		// 当峰值点距离过近时，保留幅值大的，去掉幅值小的
			xzParShake.gFlgMaxMin=2;
			if(data[i]>xzParShake.preVaule)//当前峰值点>上一个的幅值
			{
				xzParShake.MaxVaule+=data[i]-xzParShake.preVaule; // 去点上一个峰值点
				xzParShake.preIndex=tmpIndex+i;  //更新上一个峰值点的位置
				xzParShake.preVaule=data[i];
			}
		}
	}
	return xzParShake;
}


/**********************************************************************************
* Function Name: xyzShakeWatchMin
* Purpose      : 摇一摇看表 xyz轴每轴的判断极小值： flg==0:x,z轴条件 flg==1 y轴条件
* Parameter    :
* return       :
**********************************************************************************/
ParShake xyzShakeWatchMin(int16_t *data,ParShake xyzParShake,uint8_t i,uint32_t tmpIndex ,uint8_t flg,stThsold parThsold)
{
    // 极小值点  （与前后2点的比较）
	if((data[i]-data[i-1]<parThsold.minVauleFirst||(data[i]<=data[i-1]&&data[i]-data[i-2]<parThsold.minVauleFirst)\
		||(flg==1 &&data[i]<=data[i-1] && data[i]-data[i-2]<parThsold.minVauleSecond && data[i-1]<=data[i-2]))\

		&&(data[i]-data[i+1]<parThsold.minVauleFirst||(data[i]<=data[i+1]&&data[i]-data[i+2]<parThsold.minVauleFirst)\
			||(flg==1&& data[i]<=data[i+1] && data[i]-data[i+2]<parThsold.minVauleSecond && data[i+1]<=data[i+2]))\
		&& data[i]<-parThsold.absVaule)
	{
		if(tmpIndex+i-xyzParShake.preIndex>20 && xyzParShake.preIndex!=0) // 与上一个峰？位置比较，距离>20 (0.6s) ,峰值点幅值和  + 峰值点计数清零
		{
			xyzParShake.MaxVaule=0;
			xyzParShake.A=0;
		}
		if(tmpIndex+i-xyzParShake.preMinIndex>40 && xyzParShake.preMinIndex!=0) //与上一个极小值点位置距离>40 (0.8s),极小值点和清零+极小值计数清零
		{
			xyzParShake.MinVaule=0;
			xyzParShake.I=0;
		}
		if (tmpIndex+i-xyzParShake.preMinIndex>5)  //与上一个极小值点位置距离>5 (0.1s),
		{
			xyzParShake.gFlgMaxMin=1;     //  标志位1代表极小值 2代表极大值
			xyzParShake.MinVaule+=data[i];// 极小值点幅值之和
			xyzParShake.I++;
			xyzParShake.preMinIndex=Mpy_Sign(5,secShake)+i;  // 记录位置
			xyzParShake.preMinVaule=data[i];                 // 记录幅值
		}
		else // 距离<5,距离太近，去干扰：去掉峰值点幅值小的（此处不应该去掉峰值点大的么???）
		{
			xyzParShake.gFlgMaxMin=1;
			if(data[i]>xyzParShake.preMinVaule)
			{
				xyzParShake.MinVaule+=data[i]-xyzParShake.preMinVaule;
				xyzParShake.preMinIndex=Mpy_Sign(5,secShake)+i;
				xyzParShake.preMinVaule=data[i];
			}
		}
	}
	return xyzParShake;
}

/**********************************************************************************
* Function Name: clcManyShake
* Purpose      : 摇一摇看表  极大值和极小值个数不匹配     清0
* Parameter    :
* return       :
**********************************************************************************/
ParShake clcManyShake(ParShake xyzParShake)
{
// 极大值个数-极小值个数>1 or  上一个和当前都为极大值或者极小值
	if(abs(xyzParShake.A-xyzParShake.I)>1 || (xyzParShake.gFlgMaxMin==xyzParShake.gPreMaxMin\
	&& xyzParShake.gPreMaxMin!=0 && xyzParShake.gFlgMaxMin!=0) )
	{
		xyzParShake.cntShakeSW=0;
		xyzParShake.MaxVaule=0;
		xyzParShake.MinVaule=0;
		xyzParShake.A=0;
		xyzParShake.I=0;
		xyzParShake.gPreMaxMin=0;
		xyzParShake.gFlgMaxMin=0;
	}
	if(xyzParShake.gFlgMaxMin!=0)
	{
		xyzParShake.gPreMaxMin=xyzParShake.gFlgMaxMin;
		xyzParShake.gFlgMaxMin=0;
	}
	return xyzParShake;
}


/**********************************************************************************
* Function Name: xyzFlgShake
* Purpose      : 摇一摇看表  xyz轴符合条件判断，符合个数+幅值等条件xyzParShake.gFlg=1;
* Parameter    :
* return       :
**********************************************************************************/
ParShake xyzFlgShake(ParShake xyzParShake,stThsold thsold)
{
	// 1.峰值点、谷值点个数都>1; 2.峰值点的平均幅值-顾值点的平均峰值>阈值；3.
	if(xyzParShake.A>1 && xyzParShake.I>1 && gFlgShakeView==0 \
	&& xyzParShake.MaxVaule/xyzParShake.A-xyzParShake.MinVaule/xyzParShake.I>thsold.maxMinVaule\
	&& xyzParShake.gCntMax<10 && xyzParShake.gExterNum<10)
	{
		xyzParShake.gFlg=1;
	}
	return xyzParShake;
}

/**********************************************************************************
* Function Name: xyz30ExterNumShake
* Purpose      : 摇一摇看表  xyz超过30 清0判断
* Parameter    :
* return       :
**********************************************************************************/
ParShake xyz30ExterNumShake(ParShake xyzParShake, int16_t *data)
{
	uint8_t i = 0;
	if(xyzParShake.cntShakeSW>30)
	{
		xyzParShake.cntShakeSW=0;
		xyzParShake.MaxVaule=0;
		xyzParShake.MinVaule=0;
		xyzParShake.A=0;
		xyzParShake.I=0;
	}
	if(xyzParShake.MaxVaule>0)
	{
		for(i=2;i<7;i++)
		{
			if( data[i]>data[i-1] && data[i]>data[i-2]\
			&& data[i]>data[i+1] && data[i]>data[i+2] )
			{
				xyzParShake.gExterNum++;
			}
		}
	}
	else
	{
		xyzParShake.gExterNum=0;
	}
	return xyzParShake;
}


/**********************************************************************************
* Function Name: shakeWatch
* Purpose      : 摇一摇看表
* Parameter    :
* return       : 摇一摇标志位 （global variate）：gFlgShakeView
**********************************************************************************/
void shakeWatch(int16_t *data_x,int16_t *data_y,int16_t *data_z,stThsold parThsold,uint8_t flgMode)
{
	uint8_t i=0;
	int16_t xStd=0;//当前5个点的标准差
	int16_t yStd=0;
	int16_t zStd=0;
	uint32_t tmpIndex=0;
	int16_t yMean=0;
	int16_t tmp=0;

	// 更新数组
	for(i=0;i<4;i++)
	{
		gXData[i]=gXData[i+5]; // (0:3)上一组的后4个采样点
		gYData[i]=gYData[i+5];
		gZData[i]=gZData[i+5];
	}
	for(i=0;i<5;i++)
	{
		gXData[i+4]=data_x[i]; //（4：8）当前数组的5个采样点
		gYData[i+4]=data_y[i];
		gZData[i+4]=data_z[i];
	}

	// y轴 9个采样点的均值
	for(i=0;i<9;i++)
	{
		yMean+=gYData[i];
	}
	yMean=yMean/9;

	secShake++;

	xStd = int16_t_std(data_x, 0, 5); //data_x[5] 标准差
	yStd = int16_t_std(data_y, 0, 5);
	zStd = int16_t_std(data_z, 0, 5);

	tmpIndex=Mpy_Sign(5,secShake);

	tmp=parThsold.absVaule;

	for(i=2;i<7;i++)
	{
		xParShake=xzShakeWatchMax(gXData,xParShake,xStd,i,tmpIndex,parThsold);
		if(yMean>0)
		{
			YShakeWatchMax(yStd,tmpIndex,i,parThsold);
		}
		else
		{
			parThsold.absVaule=tmp-250;
			YShakeWatchMax(yStd,tmpIndex,i,parThsold);
			parThsold.absVaule=tmp;
		}

		zParShake=xzShakeWatchMax(gZData,zParShake,zStd,i,tmpIndex,parThsold);

		xParShake=xyzShakeWatchMin(gXData, xParShake,i,tmpIndex,0,parThsold);
		yParShake=xyzShakeWatchMin(gYData, yParShake,i,tmpIndex,1,parThsold);
		zParShake=xyzShakeWatchMin(gZData, zParShake,i,tmpIndex,0,parThsold);
	}

	xParShake=clcManyShake(xParShake);
	yParShake=clcManyShake(yParShake);
	zParShake=clcManyShake(zParShake);

	xParShake=xyzFlgShake(xParShake,parThsold);
	yParShake=xyzFlgShake(yParShake,parThsold);
	zParShake=xyzFlgShake(zParShake,parThsold);

	if( yParShake.gFlg==1 || \
	( (xParShake.gFlg==1  || zParShake.gFlg==1) && (!flgMode)))
	{
		gTimeXYZ++;
		if(gTimeXYZ<30)  // 摇一摇时间<3s
		{
			if(xStd<50 && yStd<50 && zStd<50)  // 静止
			{
				gFlgShakeView=1;
				clcSWatch();// 摇一摇亮屏后，清除所有摇一摇的有关变量
			}
		}
		else // 摇一摇超过3s没有静止下来，标志位清零
		{
			gTimeXYZ=0;
			xParShake.gFlg=0;
			yParShake.gFlg=0;
			zParShake.gFlg=0;
		}
	}
	if(xStd<150 && yStd<150 && zStd<150)
	{
		xParShake.gCntMax=0;
		yParShake.gCntMax=0;
		zParShake.gCntMax=0;
	}

	if(xParShake.A>0 || xParShake.I>0)
	{
		xParShake.cntShakeSW++;
	}
	if(yParShake.A>0 || yParShake.I>0)
	{
		yParShake.cntShakeSW++;
	}
	if(zParShake.A>0 || zParShake.I>0)
	{
		zParShake.cntShakeSW++;
	}

	if(xStd>50 && yStd>50 && zStd>50) // 3轴标准表都>50
	{
		gTimeStd++;   // 计时
		if(gTimeStd>254)
		{
			gTimeStd=100;
		}

		if(gTimeStd>40 && gFlgShakeView==1 ) // 当时间>4s + 摇一摇亮平时  灭屏
		{
			gFlgShakeView=0;
		}
		else
		{
			if(gTimeStd<40 && gFlgShakeView==1)
			{
				gTimeStd=0;
			}
		}
	}
	else
	{
		gTimeStd=0;
	}

	if(xStd<20 && yStd<20 && zStd<20)
	{
		gCntStd++;    // xyz标准差都<20的计数
		if(gCntStd>250)
		{
			gCntStd=5;
		}
		if(gCntStd>1)
		{
			secShake=0;
			xParShake.preIndex=0;
			yParShake.preIndex=0;
			zParShake.preIndex=0;
			clcSWatch();//与摇一摇有关的变量清零
		}
	}
	else
	{
		gCntStd=0;
	}

	xParShake=xyz30ExterNumShake(xParShake,gXData);
	yParShake=xyz30ExterNumShake(yParShake,gYData);
	zParShake=xyz30ExterNumShake(zParShake,gZData);

	if(gFlgShakeView == 1) //摇一摇亮屏了
	{
		clcSWatch();
		cntShakeView++; //摇一摇亮计数
		if(cntShakeView >= 50)  // 超过5s 清零
		{
			gFlgShakeView = 0;
			cntShakeView = 0;
			clcSWatch();
		}
	}
	else   // 灭屏，计数清零
	{
		cntShakeView=0;
	}

	//y轴>1000,摇一摇标志清零
	if(yMean>1000)
	{
		gFlgShakeView=0;
	}
	//Up_flag=gFlgShakeView;
}

/**********************************************************************************
* Function Name: Up_Hand
* Purpose      : 抬手敲击摇一摇主程序--更改2022/02/25
* Parameter    : *data_x,*data_y,*data_z 加速度数据，长度为5，int16_t类型
* return       :
**********************************************************************************/
void Up_Hand(int16_t *data_x, int16_t *data_y, int16_t *data_z, uint8_t time)
{
	uint8_t j = 0;

	/****** 计算均值 并保存 *****/
	for (j = 0; j < L_LEN - 1; j++)    //数据移位
	{
		Avr_x[j] = Avr_x[j + 1];
		Avr_y[j] = Avr_y[j + 1];
		Avr_z[j] = Avr_z[j + 1];
	}
	Avr_x[L_LEN - 1] = int16_t_mean(data_x, 0, 5);		//传入均值
	Avr_y[L_LEN - 1] = int16_t_mean(data_y, 0, 5);
	Avr_z[L_LEN - 1] = int16_t_mean(data_z, 0, 5);

	/***** 计算当前3个轴n个点标准差 ********/
	std_x = 0;                 // var*n  方差*n  单次调用 先清零在计算
	std_y = 0;
	std_z = 0;
	for (j = 0; j < 5; j++)
	{
		std_x += SquareOfDiff(data_x[j], Avr_x[L_LEN - 1]);
		std_y += SquareOfDiff(data_y[j], Avr_y[L_LEN - 1]);
		std_z += SquareOfDiff(data_z[j], Avr_z[L_LEN - 1]);
	}

	/********亮屏检测 *******************/

	shakeWatch(data_x, data_y, data_z, smartViewTh, 0);  //摇一摇 计算gFlgShakeView
	fUpBeatWatch(data_x, data_y, data_z, time);          //亮屏、切屏判断  Up_flag

	if (Up_flag <= 1)
	{
		Up_flag = (gFlgShakeView || Up_flag);
	}
	if (gFlgShakeView == 1)
	{
		clcSWatch();
		up_hand_clc();
	}
	gFlgShakeView = 0;

	/* 保留最后一个采样点 */
	Data_x = data_x[4];
	Data_y = data_y[4];
	Data_z = data_z[4];

}
