
#include "view_main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "dirent.h"
#include "../alg_ComFunc.h"
#include "../taishou/algSmartView.h"


#define LEN_NAME  200

static const char comname[LEN_NAME] = "D:\\07-SmartView20220221\\VC\\data_wx\\";
static const char * fileType = ".txt";
char sname1[LEN_NAME];



//不同数据库的方向不一样，导致xyz的变化，因此该算法对每次输入的数据要首先进行相应的变换
// #define DATA_JIUAN  // [fx,fy,fz] = [x,y,z]
#define DATA_WX       // [fx,fy,fz] = [-y,-x,-z]
char *filename;

uint8_t debug = 1;
int16_t CntNs = 0;	    //Ns
int16_t Cnt01s = 0;	    //计数器，每个点单位：ARR_LEN/FS        此处：5/50 = 0.1s


						//路径字符串 初始化子函数 每次路径更新前调用
static void initialsname(void)
{
	uint16_t i = 0;
	for (; i < LEN_NAME; i++)
	{
		sname1[i] = comname[i];
	}
}

static void init_parameters_file(void)
{
	CntNs = 0;	    //Ns
	Cnt01s = 0;	    //计数器，每个点单位：ARR_LEN/FS        此处：5/50 = 0.1
}

void view_main(void)
{

	DIR *dir_data;
	FILE *fid = NULL;
	struct dirent *readdir_data;
	int16_t file_n = 0;        //文件个数计数

	int16_t i, j;
#ifdef DATA_JIUAN
	int16_t data[DIS_LEN_THREE_AXIS] = { 0 };
#endif

#ifdef DATA_WX
	float data[DIS_LEN_THREE_AXIS] = { 0 };
#endif
	int16_t x_raw_data[ARR_LEN] = { 0 };
	int16_t y_raw_data[ARR_LEN] = { 0 };
	int16_t z_raw_data[ARR_LEN] = { 0 };

	//uint8_t Array_Up_flag[LEN_ARR_UPFLAG] = { 0 };
	//uint8_t Array_Beat_flag[LEN_ARR_BEATFLAG] = { 0 };
	//uint8_t Array_After_flag[LEN_ARR_AFTERFLAG] = { 0 };
	//uint8_t Array_Method_flag[LEN_ARR_UPFLAG] = { 0 };


	initialsname();            //路径字符串 初始化
	dir_data = opendir(sname1);//

	if (dir_data)
	{
		while ((readdir_data = readdir(dir_data)) != NULL )
		{
			if (readdir_data->d_namlen>5 && strstr(readdir_data->d_name, fileType))
			{
				file_n++;
				initialsname();
				init_parameters_file();

				if (file_n > 0)
				{
					filename = strcat(sname1, readdir_data->d_name); //文件路径
					fid = fopen(filename,"r");   //打开文件

					printf("%d\t%s\n",file_n, readdir_data->d_name);
					printf(".............\n");

					if (fid == NULL)
					{
						printf("文件打开失败\n");
					}
					else
					{

						// 初始化算法全局变量
						clcbeat();      // 每个文件开始
						clcSWatch();    // 摇一摇shakeWatch 相关变量 清0
						do
						{
							//  读入数据

#ifdef DATA_JIUAN
							for (i = 0; i < 3 * ARR_LEN; i++)
							{
								fscanf(fid, "%d", &data[i]);
							}

							for (j = 0; j < ARR_LEN; j++)
							{
								x_raw_data[j] = data[3 * j];
								y_raw_data[j] = data[3 * j + 1];
								z_raw_data[j] = data[3 * j + 2];
							}
#endif

#ifdef DATA_WX
							for (i = 0; i < 3 * ARR_LEN; i++)
							{
								fscanf(fid, "%f", &data[i]);
							}
							for (j = 0; j < ARR_LEN; j++)
							{
								y_raw_data[j] = -data[3 * j];
								x_raw_data[j] = -data[3 * j + 1];
								z_raw_data[j] = -data[3 * j + 2];
							}
#endif // DATA_WX

							CntNs++;
							Cnt01s++;   //0.1s计时

							for (i = 0; i < LEN_ARR_UPFLAG; i++)
							{
								//Array_Up_flag[i] = 0;
								//Array_Beat_flag[i] = 0;
								//Array_After_flag[i] = 0;
							}

							/******调试--时间断点*****/
							if (Cnt01s == 2405)
							{
								debug = 1;
							}
							/***************************/

							//Up_Hand_Orig(x_raw_data, y_raw_data, z_raw_data, 50); //抬手主函数--原版本
							Up_Hand(x_raw_data, y_raw_data, z_raw_data, 50); //抬手主函数--修改

							printf("%d\t%d\n", Cnt01s, Up_flag);

							//Array_Up_flag[Cnt01s] = Up_flag;
							//Array_Beat_flag[Cnt01s] = Beat_flag;
							//Array_After_flag[Cnt01s] = After_flag;
							//Array_Method_flag[Cnt01s] = Method_flag;
							//Method_flag = 0;

						} while (!feof(fid));


					}  //end if (fid == NULL)

					fclose(fid);

					printf(".............\n");

					//单个文件处理结束
				}  // end if (file_n > 0)

			}
		}
	}

	closedir(dir_data);
	getchar();







}
