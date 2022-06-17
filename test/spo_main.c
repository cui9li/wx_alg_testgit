#include "spo_main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <direct.h>
#include "dirent.h"
#include "adpd_spo2.h"
#include "../alg_ComFunc.h"
#include "debug.h"

//*********添加路径下文件的调用*******
#define CHANNEL    3              //ppg光路3路：red ir green
#define SENT_PACK  0             // 软件设置：整个1min采集数据中：34包开始发送给上位机  发送包[34 213]

static const char comname[LEN_NAME] = "D:\\10-signalQT_hardware\\data\\06-wrist-3mA\\ppg_RedIrGreen\\";
static const char * fileType = ".txt";
char sname1[LEN_NAME];
// 保存结果
static const char *stats_adrr = "C:\\Users\\cjl\\Desktop\\rel\\hr_stats.txt";
static FILE *fp_stats = NULL;

//路径字符串 初始化子函数 每次路径更新前调用
static void initialsname()
{
	uint16_t i = 0;
	for (; i < LEN_NAME; i++)
	{
		sname1[i] = comname[i];
	}
	debug_init();    // for debug
}


void spo_main()
{
	// 路径文件夹信息
	DIR *dir_ppg;
	struct dirent *readdir_ppg;
	//文件信息
	FILE *fp_ppg;
	//变量
	uint16_t pack_point_cnt = 0;
	int32_t pack_ppg[CHANNEL*PACK_POINTS] = { 0 };
	//int32_t spo2_data[CHANNEL] = { 0 };

	char *filePath;

	int16_t size;

	uint8_t Spo2_value = 0;  //每包的spo值
	uint8_t bpm_value = 0;   //每包的bpm值
	int16_t pack_n = 0;      //小包的计数
	int16_t file_n = 0;      //文件个数计数

	// ******for debug*********
	uint8_t debug1 = 1;
	//uint8_t spo_1s = 0;     //每1s的输出spo值（1s = 4个包）
	uint8_t pr_1s = 0;      //每1s的输出bpm值（1s = 4个包）
	//uint8_t arr_cnt = 0;
	uint8_t value_cnt = 0;  //bpm spo >0 的个数
	//************************

	//路径字符串 初始化
	initialsname();
	dir_ppg = opendir(sname1);

	fp_stats = fopen(stats_adrr,"w");   // 保存所有文件的结果,写入
	//fprintf(fp_stats,"%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n", "file_n", "readdir_ppg->d_name", "file_len", "hr_len", "hr_median", "hr_mean", "hr_std", "rr_mean", "rr_std");

	if (dir_ppg) {
		while ( (readdir_ppg = readdir(dir_ppg)) != NULL) {
			if (readdir_ppg->d_namlen>5 && strstr(readdir_ppg->d_name, fileType))
			{
				file_n++;
				//printf("\n%d\t%s\t%d\t%d\n", file_n, readdir_ppg->d_name, readdir_ppg->d_namlen, readdir_ppg->d_reclen);
				//printf("dc_removal:%s\t%s\t%s;\t\t%s\t%s\t%s\n", "red_data", "red_dc", "red_value", "ir_data", "ir_dc", "ir_value");

				if (file_n >= 0)  // 第几个文件
				{

					// debug
					startup_file(readdir_ppg->d_name);
					init_parameters();

					// 处理单个文件
					initialsname();
					filePath = strcat(sname1, readdir_ppg->d_name); //文件路径
					fp_ppg = fopen(filePath, "r");                  //读文件

					if (fp_ppg)
					{
						adpd_spo2_init(1, 1); // 计算之前初始化
						Spo2_value = 0;
						bpm_value = 0;
						pack_n = 0;
						//arr_cnt = 0;
						value_cnt = 0;

						while (!feof(fp_ppg))   //文件不为空
						{
							//模拟小包的形式
							if (strcmp(fileType, ".txt") == 0)
							{
								for (pack_point_cnt = 0; pack_point_cnt < CHANNEL*PACK_POINTS; pack_point_cnt++) {  //读13行，小包数据
									fscanf(fp_ppg, "%d", &pack_ppg[pack_point_cnt]);
								}
							}
							if (strcmp(fileType, ".csv") == 0)
							{
								//模拟小包的形式
								for (pack_point_cnt = 0; pack_point_cnt < 3 * PACK_POINTS; pack_point_cnt += 3) {  //读13行，小包数据
									fscanf(fp_ppg, "%d,%d,%d,", &pack_ppg[pack_point_cnt], &pack_ppg[pack_point_cnt + 1], &pack_ppg[pack_point_cnt + 2]);
								}
							}
 							pack_point_cnt = 0;  // 包计数清零
							pack_n++;

							// *******调用每个小包算法**********************************
							size = PACK_POINTS;
							adpd_spo2_data_deal(size, pack_ppg, &Spo2_value,&bpm_value);

							/*if (Spo2_value || bpm_value)
							{
								printf("%d\tspo = %d;bpm = %d\n", pack_n, Spo2_value, bpm_value);
							}*/

							//***********************************************************

							//********每4个包的输出值************************************
							if (Spo2_value)
							{
								//spo_1s = Spo2_value;
							}
							if (bpm_value)
							{
								pr_1s = bpm_value;
							}
							if (pack_n % 4 == 0)
							{
								if (pr_1s && pack_n >= SENT_PACK) //
								{
									value_cnt++;
								}
								//printf("%.1f\t spo_1s = %d;bpm_1s = %d\n", (float)pack_n*PACK_POINTS/50, spo_1s, pr_1s);
								//printf("%d\t%d\n", pack_n / 4,  pr_1s);
								//spo_1s = 0;
								pr_1s = 0;
							}
							//***********************************************************

							if (debug1 && Spo2_value>0)
							{
								debug1 = 1;
							}

						}// end while (!feof(fp_ppg))	 //文件不为空
					}// end if (fp_ppg)

					fclose(fp_ppg);
					close_file();

					// 统计量计算
#ifdef HR_STATS
					cal_hr_statistics();
					//printf("%d\t%s\t%d\t%d\t%d\t%d\t%.1f\t%.1f\t%.1f\n",file_n, readdir_ppg->d_name,file_len, hr_len, hr_median,hr_mean, hr_std,rr_mean,rr_std);
					//fprintf(fp_stats,"%d\t%s\t%d\t%d\t%d\t%d\t%.1f\t%.1f\t%.1f\n", file_n, readdir_ppg->d_name, file_len, hr_len, hr_median, hr_mean, hr_std, rr_mean, rr_std);
					fprintf(fp_stats, "%d\t%s\t%d\t%d\t%d\t%d\n", file_n, readdir_ppg->d_name, len_rrIn10Mean, len_rrIn20Mean, len_rrIn30Mean, len_hrAbnormal);

#endif

					printf(" %s\t value_cnt=%d\t pack_t = %d\n ", readdir_ppg->d_name, value_cnt, (pack_n- SENT_PACK) /4);// 每个文件计算的结果个数

					//单个文件处理结束
				}

			}// end if (readdir_ppg->d_namlen>5 && strstr(readdir_ppg->d_name,".txt"))

		}// end while ( (readdir_ppg = readdir(dir_ppg)) != NULL)
	} //end if (dir_ppg)

	fclose(fp_stats);

	closedir(dir_ppg);
	getchar();
}
