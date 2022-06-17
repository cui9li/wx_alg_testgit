/*****************************************************************************
* @file
* @author
* @version      V1.0
* @date         2022-05-07
* @brief        1. 批量读取文件夹中的文件
                2.读取每个文件中的数据，每次读取1s :150个点（x y z x y z......）
				3.调用计步函数，计算步数
*--------------------------------------------------------------------*
* Change History :
* <Data>      |<Version> |<Author>  | <Description>
*--------------------------------------------------------------------*
*
* ----------------------------------------------------------------
*****************************************************************************/

#include "active_main.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <direct.h>
#include "dirent.h"
#include "../active_monitor/alg_StepDetection.h"


#define  LEN_NAME		200    //文件名字符长度
#define  COLUMN         3      //数据列数
#define  FS             50     //数据采样率

//const char comname[LEN_NAME] = "D:\\12-steps20220402\\02-database\\03-treadwrist_jiuan\\";
const char comname[LEN_NAME] = "D:\\12-steps20220402\\02-database\\05-police\\acc_xyz\\";
const char * fileType = ".txt";
char sname_path[LEN_NAME];

/* 单个文件数据读取--变量 */
float data_xyz[150] = { 0 };
int16_t data_x[50] = { 0 };
int16_t data_y[50] = { 0 };
int16_t data_z[50] = { 0 };

int16_t xyz_raw_data[50];         // 1s 和加速度数组
uint32_t sec_n = 0;   //计时秒
uint32_t current_steps = 0; // 当前总步数


void init_parameters_file(void)
{
    int16_t i = 0;
    for (i = 0; i < 50; i++)
    {
        data_xyz[i] = 0;
        data_x[i] = 0;
        data_y[i] = 0;
        data_z[i] = 0;

        xyz_raw_data[i] = 0;
        sec_n = 0;
        current_steps = 0;
    }
}

void initial_sname(void)
{
    uint16_t i = 0;
    for (i = 0; i<LEN_NAME; i++)
    {
        sname_path[i] = comname[i];
    }
    //debug_init();    // for debug
}


void active_main(void)
{

    int16_t i = 0;

    DIR *dir_data;
    struct dirent *readdir_data;
    uint16_t file_n = 0;         // 文件个数计数

    char * filePath;

    // 文件信息
    FILE * fp_data = NULL;

    // code
    initial_sname();             /* 初始化名字*/
    dir_data = opendir(sname_path);

    if (dir_data)
    {
        while ((readdir_data = readdir(dir_data)) != NULL)
        {
            if (readdir_data->d_namlen>5 && strstr(readdir_data->d_name, fileType))
            {
                file_n++;
                //printf("\n%d\t%s\t%d\t%d", file_n, readdir_data->d_name, readdir_data->d_namlen, readdir_data->d_reclen);
                printf("\n%d\t%s\t", file_n, readdir_data->d_name);

                if (file_n >= 0)                                          // 第几个文件  控制调试哪个文件
                {
                    // 处理单个文件
                    initial_sname();
                    filePath = strcat(sname_path, readdir_data->d_name);  //文件路径
                    fp_data = fopen(filePath, "r");                       //读文件
                    init_parameters_file();
                    Alg_Clear_RAM();

                    if (fp_data)                                          // 打开文件
                    {
                        while (!feof(fp_data))                            //(是否读到文件结尾)
                        {
                            /*读文件中的数据*/
                            if (strcmp(fileType, ".txt") == 0)
                            {
                                for (i = 0; i < COLUMN * 50; i++)
                                {
                                    fscanf(fp_data, "%f", &data_xyz[i]);
                                }

                                for (i = 0; i < 50; i++)   // 该处应该对应现时xyz与软件输出的xyz相对应上
                                {
                                    data_x[i] = (int16_t)data_xyz[COLUMN * i];
                                    data_y[i] = (int16_t)data_xyz[COLUMN * i + 1];
                                    data_z[i] = (int16_t)data_xyz[COLUMN * i + 2];
                                }

                                // ========计步===========================
                                for (i = 0; i<50; i++)
                                {
                                    alg_AM(data_x[i], data_y[i], data_z[i], &current_steps); //计步
                                }


                                sec_n++;

                                //printf("%d\t%d\n", sec_n, steps_display);

                                //============end 计步 ====================

                            }

                        } //end while (!feof(fp_data))
                        printf("\tsec_n=%d\tsteps_display=%d\tcurrent_steps=%d\n", sec_n, steps_display, current_steps);
                    }

                    fclose(fp_data);

                }

            }

        }
    }
    else
    {
        printf("sname_path is NULL!");
    }

    closedir(dir_data);
    //getchar();

}
