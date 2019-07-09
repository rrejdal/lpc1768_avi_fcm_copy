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

void perf_t1();
void perf_t2();
void perf_printf();

bool Streaming_Process(FlightControlData *hfc);

//void Profiling_Process(FlightControlData *hfc, const ConfigData *pConfig);

/* float32/float16 conversion
** largest +/-65504, smallest +/-6.10352e—6 */
float Float16toFloat32(const unsigned short int in);
unsigned short int Float32toFloat16(const float in);

/* watchdog API */
void WDT_Kick();
void WDT_Init(float timeout);
bool WDT_ResetByWDT();

void SensorCalib(FlightControlData *hfc, float dT);
void GyroCalibDynamic(FlightControlData *hfc);

uint32_t crc32b(uint8_t *message, uint32_t length);
bool N1WithinPercentOfN2(float n1, float percentage, float n2);
uint32_t GetResetReason(void);

#endif
