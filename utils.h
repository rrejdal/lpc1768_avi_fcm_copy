#ifndef _UTILS_
#define _UTILS_

#include "structures.h"
#include "xbus.h"
#include "pGPS.h"

#define D2R 0.017453292f
#define R2D 57.29578122f
#define EarthR  6372795
#define DPM (180.0f/EarthR/3.14159265359f)   // degrees per meter

/* returns distance in meters and course in deg (+/-180) from north CW from (1) to (2) */
/* accurate only for short distances, a few kilometers, since it assumes the Earth is flat */
float DistanceCourse(double lat1, double long1, double lat2, double long2, float *course);
float Distance(float lat1, float long1, float lat2, float long2);


unsigned int Ticks1();
unsigned int Ticks2(unsigned int ticks1);
unsigned int Ticks2us(unsigned int ticks1);
unsigned int Ticks_us();
void SysTick_Run();
void Ticks_wait(unsigned int us);
unsigned int GetTime_ms();
unsigned int GetTime_us();
unsigned int Ticks_us_minT(unsigned int minT_us, int *utilization);

volatile void perf_t1();
volatile void perf_t2();
void perf_printf();

char LoadConfig_Float(const char *name, float *value, int N);
char LoadConfig_Int(const char *name, int *value, int N);
char LoadConfig_Byte(const char *name, unsigned char *value, int N);
char LoadConfig_Char(const char *name, char *value, int N);
char Config_Open(char *filename);
void Config_Close();
void SaveConfig_Float(FILE *fp, const char *name, float *value, int N);
void SaveConfig_Int(FILE *fp, const char *name, int *value, int N);
void SaveConfig_Byte(FILE *fp, const char *name, unsigned char *value, int N);

void GetLogFileName(char *filename);
void LoadGyroCalibData(float ofs[3]);

bool Streaming_Process(T_HFC *hfc);

void Profiling_Process(T_HFC *hfc, ConfigData *pConfig);

void CalibrateSensors(T_HFC *hfc, float gB[3], ConfigData *pConfig);

/* float32/float16 conversion
** largest +/-65504, smallest +/-6.10352e—6 */
float Float16toFloat32(const unsigned short int in);
unsigned short int Float32toFloat16(const float in);

/* watchdog API */
void WDT_Kick();
void WDT_Init(float timeout);
bool WDT_ResetByWDT();

void SensorCalib(T_HFC *hfc, float dT);
void GyroCalibDynamic(T_HFC *hfc);

#endif
