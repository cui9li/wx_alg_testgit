/*****************************************************************************
* @file         judge_Iswearing.c
* @author       wx_c
* @version      V0.1
* @date         2022-05-05
* @brief        judge whether to wear  based on ir_ppg  更改结构 或者配置的时候必须调整参数或者阈值

*--------------------------------------------------------------------*
* Change History :
* <Data>      |<Version> |<Author>  | <Description>
*--------------------------------------------------------------------*
* 2022-05-25  |V0.1      |wx_cjl    |1.不同固件：4000版本/MeiXin版本/公安版本 参数不同
* ----------------------------------------------------------------
*****************************************************************************/

#include"judge_Iswearing.h"

#define IR_DC_LIMIT_VERSION_4000       360000  //362000 358000 // 不同的光强下要更改
#define RED_DC_LIMIT_VERSION_4000      360000
#define GREEN_DC_LIMIT_VERSION_4000    360000

#define IR_DC_LIMIT_VERSION_MEIXIN      6000
#define RED_DC_LIMIT_VERSION_MEIXIN     5000
#define GREEN_DC_LIMIT_VERSION_MEIXIN   2000

#define IR_DC_LIMIT_VERSION_POLICE      20000
#define RED_DC_LIMIT_VERSION_POLICE     20000
#define GREEN_DC_LIMIT_VERSION_POLICE   20000

#define UNWEAR_TIME_POINTS    10             // judge time of on to off
#define WEAR_TIME_POINTS      50             // judge time of off to on

uint8_t wear_flag = 1;                       //佩戴标志位
static uint16_t ir_wear_cnt = 0;
static uint16_t ir_unwear_cnt = 0;
static uint16_t red_wear_cnt = 0;
static uint16_t red_unwear_cnt = 0;
//static uint16_t green_wear_cnt = 0;
//static uint16_t green_unwear_cnt = 0;

static int32_t ir_dc_limit = 0;
static int32_t red_dc_limit  = 0;
//static int32_t green_dc_limit = 0;

/**********************************************************************************
* 函数名称：wear_init
* 功能描述：初始化（重要）
* 版    本：v1.0
* 参数说明：wear_init_flag：1代码佩戴，0未佩戴，一般初始化为1；
             version_num ：版本号码 1：version_4000  2: version_MeiXin  3: version_police
* 返回说明：1.用于整个程序启动时候，例如开机等* 说    明：
             2. 每个版本必须进行初始化（初始化中涉及到了每个版本的阈值）
             3. 1：version_4000  2: version_MeiXin  3: version_police
**********************************************************************************/
void wear_init(uint8_t wear_init_flag, uint8_t version_num)
{
    wear_flag = wear_init_flag;
    ir_wear_cnt = 0;
    ir_unwear_cnt = 0;
    red_wear_cnt = 0;
    red_unwear_cnt = 0;
    //green_wear_cnt = 0;
    //green_unwear_cnt = 0;

    if (1 == version_num) // version_4000
    {
        ir_dc_limit = IR_DC_LIMIT_VERSION_4000;
        red_dc_limit  = RED_DC_LIMIT_VERSION_4000;
        //green_dc_limit = GREEN_DC_LIMIT_VERSION_4000;
    }
    if (2 == version_num) // version_Meixin
    {
        ir_dc_limit = IR_DC_LIMIT_VERSION_MEIXIN ;
        red_dc_limit  = RED_DC_LIMIT_VERSION_MEIXIN ;
        //green_dc_limit = GREEN_DC_LIMIT_VERSION_MEIXIN ;
    }
    if (3 == version_num) // version_police
    {
        ir_dc_limit = IR_DC_LIMIT_VERSION_POLICE;
        red_dc_limit  = RED_DC_LIMIT_VERSION_POLICE;
        //green_dc_limit = GREEN_DC_LIMIT_VERSION_POLICE;
    }

}

/**********************************************************************************
* 函数名称：judge_Iswearing(int32_t ir_data)
* 功能描述：是否佩戴-主函数
* 版    本：v1.0
* 参数说明：
* 返回说明：1佩戴；0未佩戴
* 说    明：1.每个采样点调用;
            2. 每个版本必须进行初始化（初始化中涉及到了每个版本的阈值）
            3. 1：version_4000  2: version_MeiXin  3: version_police
            4. 如果没有red_data或者green_data,传参数 -10  （因为信号不会出现负值）
**********************************************************************************/

uint8_t judge_Iswearing(int32_t ir_data, int32_t red_data, int32_t green_data)
{
    uint8_t rel = 0;

    if (wear_flag == 1)  // 若佩戴，检测没有佩戴
    {
        if ( ir_data <= ir_dc_limit )
        {
            ir_unwear_cnt++;
        }
        if ( (red_data >= 0 && red_data <= red_dc_limit) )
        {
            red_unwear_cnt++;
        }
        if (ir_unwear_cnt > UNWEAR_TIME_POINTS || red_unwear_cnt > UNWEAR_TIME_POINTS)
        {
            wear_flag = 0;
            ir_unwear_cnt = 0;
            red_unwear_cnt = 0;
            ir_wear_cnt = 0;
            red_wear_cnt = 0;

        }
    }
    else               //若没佩戴，检测佩戴
    {
        if (ir_data > ir_dc_limit )
        {
            ir_wear_cnt++;
        }
        if ( red_data >= 0  )  // 若有红光
        {
            if (red_data > red_dc_limit)
            {
                red_wear_cnt++;
            }
            if (ir_wear_cnt > WEAR_TIME_POINTS && red_wear_cnt > WEAR_TIME_POINTS)
            {
                wear_flag = 1;
                ir_wear_cnt = 0;
                red_wear_cnt = 0;
                ir_unwear_cnt = 0;
                red_unwear_cnt = 0;
            }
        }
        else                   // 没有红光
        {
            if (ir_wear_cnt > 10 )
            {
                wear_flag = 1;
                ir_wear_cnt = 0;
                red_wear_cnt = 0;
            }

        } // end if ( red_data >= 0  ) else...

    }

    rel = wear_flag;
    return rel;
}

uint8_t judge_wear_status(void)
{
    return wear_flag;
}
