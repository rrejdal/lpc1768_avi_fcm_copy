#include "utils.h"
#include "mbed.h"
#include "defines.h"
#include "HMC5883L.h"
#include "mymath.h"

static unsigned int ticks;
static unsigned int ticks_base=0;    // counts how many times SysTick wrapped around
static unsigned int ticks_last;
static unsigned int p1_counter = 0;
static int avg_delta = 0;
static int min_delta = 1<<30;
static int max_delta = 0;

float DistanceCourse(double lat1, double long1, double lat2, double long2, float *course)
{
    float dLat = (float)(lat2 - lat1);
    float dLon = (float)(long2 - long1);
    float distLat = dLat/DPM;
    float distLon = dLon/DPM*COSfD(((float)lat2));
    float dist = sqrtf(distLat*distLat + distLon*distLon);
    float cour = 0;
    if (distLat!=0 || distLon!=0)
      cour = 90-ATAN2fD(distLat, distLon);
    if (cour>180)
        cour-=360;
    else if (cour<-180)
        cour+=360;

    if (course)
        *course = cour;
    return dist;
}

float Distance(float lat1, float long1, float lat2, float long2)
{
    float dLat = (lat2 - lat1);
    float dLon = (long2 - long1);
    float distLat = dLat/DPM;
    float distLon = dLon/DPM*COSfD(lat2);
    float dist = sqrtf(distLat*distLat + distLon*distLon);

    return dist;
}

void perf_t1()
{
    if ((LPC_SC->PCONP & (1<<16)) != (1<<16))
    {
        LPC_SC->PCONP |= 1<<16;   // power for RIT
        LPC_SC->PCLKSEL1 |= 1<<26;    // CCLK/1
        LPC_RIT->RICOUNTER = 0;
        LPC_RIT->RICTRL = 0x8;    // enable
    }
    p1_counter = LPC_RIT->RICOUNTER;
}

void perf_t2()
{
    unsigned int p2 = LPC_RIT->RICOUNTER;
    int delta = p2 - p1_counter;
    max_delta = Max(max_delta, delta);
    min_delta = Min(min_delta, delta);
    avg_delta = (delta + 63*avg_delta+32)>>6;
}

void perf_printf()
{
    if (max_delta>0)
    {
        printf("Avg %5d Min %5d Max %5dus\n", (avg_delta+48)/96, (min_delta+48)/96, (max_delta+48)/96);
        min_delta = 1<<30;
        max_delta = 0;
    }
}

unsigned int Ticks1()
{
    return LPC_RIT->RICOUNTER;
}

unsigned int GetTime_ms()
{
    unsigned int t = LPC_RIT->RICOUNTER;
    long long res;
    if (t<ticks_last)
        ticks_base++;
    ticks_last = t;
    res = t + (((long long)ticks_base)<<32);
    return res/96000;
}

/* wraps every 4294 seconds because of the int output */
unsigned int GetTime_us()
{
    unsigned int t = LPC_RIT->RICOUNTER;
    long long res;
    if (t<ticks_last)
        ticks_base++;
    ticks_last = t;
    res = t + (((long long)ticks_base)<<32);
    return res/96;
}

unsigned int Ticks2(unsigned int ticks1)
{
    unsigned int ticks2 = LPC_RIT->RICOUNTER;
    unsigned int d = ticks2 - ticks1-8;
    return d;
}

unsigned int Ticks2us(unsigned int ticks1)
{
    unsigned int ticks2 = Ticks2(ticks1);
    return (ticks2+48)/96;
}

unsigned int Ticks_us()
{
    unsigned int ticks2 = Ticks2(ticks);
    ticks = Ticks1();
    return (ticks2+48)/96;
}

unsigned int Ticks_us_minT(unsigned int minT_us, int *utilization)
{
    unsigned int dT1 = (Ticks2(ticks)+48)/96;
    unsigned int dT = dT1;
    
    while(dT<minT_us)
    {
      dT = (Ticks2(ticks)+48)/96;
    }
    if (dT && utilization)
      *utilization = dT1*1000/dT;
    ticks = Ticks1();
    return dT;
}

void SysTick_Run()
{
  LPC_SC->PCONP |= 1<<16;   // power for RIT
  LPC_SC->PCLKSEL1 |= 1<<26;    // CCLK/1
  LPC_RIT->RICOUNTER = 0;
  LPC_RIT->RICTRL = 0x8;    // enable
  SysTick->LOAD  = SysTick_LOAD_RELOAD_Msk;                                  /* set reload register */
  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
  ticks = Ticks1();
}

void Ticks_wait(unsigned int us)
{
    int ticks1 = Ticks1();
    if (us>87000)
        us = 87000;
    while (Ticks2us(ticks1)<us);
}

static char *config_mem = NULL;
static int   config_size = 0;
#define MAX_LINE    200

/* copies config file line into str, which starts with string name */
static char LoadConfig_FindLine(const char *name, char *str)
{
    if (config_mem && config_size>0)
    {
        char *pstr = str;
        int ch = 0;
        int len = strlen(name);
        int i;

        for (i=0; i<config_size; i++)
        {
            if ((pstr-str)>=MAX_LINE)
            {
                printf("Config line too long, max %d\r\n", MAX_LINE);
                while(1);
            }
            ch = config_mem[i];
            if (ch==0xa)
            {
                /* complete line */
                *pstr++ = 0;
                if (!strncmp(name, str, len))
                {
                    /* found the line */
                    return 1;
                }
                pstr = str;
            }
            else
            if (ch!=0xd)
                *pstr++ = ch;
        }
    }
    else
    {
        FILE *fp = fopen("/local/config.txt", "r");
        if (fp)
        {
            char *pstr = str;
            int ch = 0;
            char eof = 0;
            int len = strlen(name);

            while (!eof)
            {
                ch = getc(fp);
                if (ch==EOF)
                {
                  ch = 0xa;
                  eof = 1;
                }
                if (ch==0xa)
                {
                    /* complete line */
                    *pstr++ = 0;
                    if (!strncmp(name, str, len))
                    {
                        /* found the line */
                        fclose(fp);
                        return 1;
                    }
                    pstr = str;
                }
                else
                if (ch!=0xd)
                    *pstr++ = ch;
            }
            fclose(fp);
        }
    }
    printf("===== Did not find %s =====\r\n", name);
    return 0;
}

static char *FindNextItem(char *pstr)
{
    while (*pstr)
    {
        if (*pstr==' ' || *pstr=='\t')
            break;
        pstr++;

    }
    while (*pstr)
    {
        if (*pstr!=' ' && *pstr!='\t')
            break;
        pstr++;

    }
    return pstr;
}

char LoadConfig_Float(const char *name, float *value, int N)
{
    char str[MAX_LINE];
    char *pstr = str;
    if (LoadConfig_FindLine(name, str))
    {
        char str1[64];
        int i;
        sscanf(pstr, "%s", str1);
        printf("%s\t", str1);
        pstr = FindNextItem(pstr);
        for (i=0; i<N; i++)
        {
            sscanf(pstr, "%f", value+i);
            printf("%8.4f\t", value[i]);
            pstr = FindNextItem(pstr);
        }
        printf("\r\n");
        return 1;
    }
    return 0;
}

char LoadConfig_Int(const char *name, int *value, int N)
{
    char str[MAX_LINE];
    char *pstr = str;
    if (LoadConfig_FindLine(name, str))
    {
        char str1[64];
        int i;
        sscanf(pstr, "%s", str1);
        printf("%s\t", str1);
        pstr = FindNextItem(pstr);
        for (i=0; i<N; i++)
        {
            sscanf(pstr, "%d", value+i);
            printf("%8d\t", value[i]);
            pstr = FindNextItem(pstr);
        }
        printf("\r\n");
        return 1;
    }
    return 0;
}

char LoadConfig_Byte(const char *name, unsigned char *value, int N)
{
    char str[MAX_LINE];
    char *pstr = str;
    if (LoadConfig_FindLine(name, str))
    {
        char str1[64];
        int i;
        sscanf(pstr, "%s", str1);
        printf("%s\t", str1);
        pstr = FindNextItem(pstr);
        for (i=0; i<N; i++)
        {
            int v;
            sscanf(pstr, "%d", &v);
            printf("%8d\t", v);
            value[i] = (unsigned char)v;
            pstr = FindNextItem(pstr);
        }
        printf("\r\n");
        return 1;
    }
    return 0;
}

char LoadConfig_Char(const char *name, char *value, int N)
{
    char str[MAX_LINE];
    char *pstr = str;
    if (LoadConfig_FindLine(name, str))
    {
        char str1[64];
        int i;
        sscanf(pstr, "%s", str1);
        printf("%s\t", str1);
        pstr = FindNextItem(pstr);
        for (i=0; i<N; i++)
        {
            int v;
            sscanf(pstr, "%d", &v);
            printf("%8d\t", v);
            value[i] = (char)v;
            pstr = FindNextItem(pstr);
        }
        printf("\r\n");
        return 1;
    }
    return 0;
}

void GetLogFileName(char *filename)
{
    int i;
    for (i=0; i<100; i++)
    {
        sPRINTd(filename, (char*)"/local/log%02d.txt", i);
        FILE *fp = fopen(filename, "r");
        /* this one does not exist, so use it */
        if (!fp)
            return;
        fclose(fp);
    }
}

char Config_Open(char *filename)
{
    int c=0;
    FILE *fp = fopen(filename, "r");
    if (!fp)
        return false;
    config_size = 0;
    while (1)
    {
        c = getc(fp);
        if (c==EOF)
            break;
        config_size++;
    }
    printf("Config file size=%d\r\n", config_size);
    fclose(fp);
    config_size++;
    config_mem = (char*)malloc(config_size);
    if (!config_mem)
    {
        printf("Failed to malloc config\n");
        while(1);
    }

    fp = fopen(filename, "r");
    fread(config_mem, 1, config_size, fp);
    config_mem[config_size-1] = 0xa;
    fclose(fp);

    printf("Config file opened\r\n");
    return true;
}

void Config_Close()
{
    if (config_mem)
        free(config_mem);
    config_mem=NULL;
}

void SaveConfig_Float(FILE *fp, const char *name, float *value, int N)
{
    int i;
    fprintf(fp, "%s", name);
    for (i=0; i<N; i++)
        fprintf(fp, "\t%f", value[i]);
    fprintf(fp, "\n");
}

void SaveConfig_Int(FILE *fp, const char *name, int *value, int N)
{
    int i;
    fprintf(fp, "%s", name);
    for (i=0; i<N; i++)
        fprintf(fp, "\t%d", value[i]);
    fprintf(fp, "\n");
}

void SaveConfig_Byte(FILE *fp, const char *name, unsigned char *value, int N)
{
    int i;
    fprintf(fp, "%s", name);
    for (i=0; i<N; i++)
        fprintf(fp, "\t%d", value[i]);
    fprintf(fp, "\n");
}

void LoadGyroCalibData(float ofs[3])
{
    FILE *fp = fopen("/local/gyro_cal.txt", "r");
    if (fp)
    {
        float temp;
        fscanf(fp, "%f %f %f %f\n", &temp, &ofs[0], &ofs[1], &ofs[2]);
        fclose(fp);
    }
}

union u32
{
    uint32_t    u32;
    float       f;
} f_u32;

// Fast half-precision to single-precision floating point conversion
//  - Supports signed zero and denormals-as-zero (DAZ)
//  - Does not support infinities or NaN
float Float16toFloat32(const unsigned short int in)
{
    union
    {
        uint32_t    u32;
        float       f;
    } t1;
    uint32_t t2 = in & 0x8000;              // Sign bit
    uint32_t t3 = in & 0x7c00;              // Exponent

    t1.u32 = in & 0x7fff;                   // Non-sign bits
    t1.u32 <<= 13;                          // Align mantissa on MSB
    t2 <<= 16;                              // Shift sign bit into position
    t1.u32 += 0x38000000;                   // Adjust bias
    t1.u32 = (t3 == 0 ? 0 : t1.u32);        // Denormals-as-zero
    t1.u32 |= t2;                           // Re-insert sign bit
    return t1.f;
}

// Fast single-precision to half-precision floating point conversion
//  - Supports signed zero, denormals-as-zero (DAZ), flush-to-zero (FTZ),
//    clamp-to-max
//  - Does not support infinities or NaN
unsigned short int Float32toFloat16(const float in1)
{
    union
    {
        uint32_t    u32;
        float       f;
    } inu;

    inu.f = in1;
    uint32_t t1 = inu.u32 & 0x7fffffff;        // Non-sign bits
    uint32_t t2 = inu.u32 & 0x80000000;        // Sign bit
    uint32_t t3 = inu.u32 & 0x7f800000;        // Exponent

    t1 >>= 13;                             // Align mantissa on MSB
    t2 >>= 16;                             // Shift sign bit into position
    t1 -= 0x1c000;                         // Adjust bias
    t1 = (t3 < 0x38800000) ? 0 : t1;       // Flush-to-zero
    t1 = (t3 > 0x8e000000) ? 0x7bff : t1;  // Clamp-to-max
    t1 = (t3 == 0 ? 0 : t1);               // Denormals-as-zero
    t1 |= t2;                              // Re-insert sign bit
    return t1;
}
   
extern HMC5883L compass;   
extern XBus xbus;

static uint16 Streaming_GetValue(FlightControlData *hfc, byte param)
{
    byte index = param&0x3;
    uint16 value = 0;
    char i;
    param>>=2;
    
    switch (param)
    {
case LOG_PARAM_XBUS_INPUT0_3:
        value = Float32toFloat16(xbus.valuesf[index]);
        break;
case LOG_PARAM_XBUS_INPUT4_7:
        value = Float32toFloat16(xbus.valuesf[index+4]);
        break;
case LOG_PARAM_JOY_INPUT:
        value = Float32toFloat16(hfc->joy_values[index]);
        break;
case LOG_PARAM_GYRO_RAW:
        value = Float32toFloat16(hfc->gyro[Min(2, index)]);
        break;
case LOG_PARAM_GYRO_LP:
        value = Float32toFloat16(hfc->gyroFilt[Min(2, index)]);
        break;
case LOG_PARAM_ACC_RAW:
        value = Float32toFloat16(hfc->acc[Min(2, index)]);
        break;
case LOG_PARAM_ACC_LP:
        value = Float32toFloat16(hfc->accFilt[Min(2, index)]);
        break;
case LOG_PARAM_COMPASS:
        if (index==0)
            value = Float32toFloat16(hfc->compass_heading);
        else
            value = Float32toFloat16(compass.dataXYZcalib[index-1]);
        break;
case LOG_PARAM_ORIENT:
        value = Float32toFloat16(hfc->IMUorient[Min(2, index)]*R2D);
        break;
case LOG_PARAM_BARO:
        if (index==0)
            value = Float32toFloat16(hfc->baro_altitude_raw_lp);
        else if (index==1)
            value = Float32toFloat16(hfc->baro_pressure);
        else
            value = Float32toFloat16(hfc->baro_temperature);
        break;
case LOG_PARAM_PITOT:             // pitot [pressure in kPA / speed in m/s / temp in deg C] (f16)
        value = 0;
        break;
        // TODO::SP: Need to account for multiple servo nodes
case LOG_PARAM_SERVOS0_3:
        value = Float32toFloat16(hfc->servos_out[index]);
        break;
case LOG_PARAM_SERVOS4_7:
        value = Float32toFloat16(hfc->servos_out[index+4]);
        break;
case LOG_PARAM_CTRL_MODE:
        for (i=0; i<5; i++) value |= (hfc->control_mode[(int)i] & 0x7) << (i*3);
        break;
case LOG_PARAM_CTRL_RAW0:
        value = Float32toFloat16(hfc->ctrl_out[RAW][index]);
        break;
case LOG_PARAM_CTRL_RAW1:
        value = Float32toFloat16(hfc->ctrl_out[RAW][4]);
        break;
case LOG_PARAM_CTRL_RATE:           // ctrl_rate [P/R/Y] in deg/s (f16)
        value = Float32toFloat16(hfc->ctrl_out[RATE][Min(2, index)]);
        break;
case LOG_PARAM_CTRL_ANGLE:          // ctrl_angle [P/R/Y] in deg (f16)
        value = Float32toFloat16(hfc->ctrl_out[ANGLE][Min(2, index)]);
        break;
case LOG_PARAM_CTRL_SPEED:          // ctrl_speed in m/s [F/R/U] (f16)
        if (index==2)
            value = Float32toFloat16(hfc->ctrl_out[SPEED][COLL]);
        else
            value = Float32toFloat16(hfc->ctrl_out[SPEED][Min(1, index)]);
        break;
case LOG_PARAM_ALTITUDE:            // altitude [CTRL / current IMU / lidar] in m (f16)
        if (index==0)
            value = Float32toFloat16(hfc->ctrl_out[POS][COLL]);
        else if (index==1)
            value = Float32toFloat16(hfc->altitude);
        else
            value = Float32toFloat16(hfc->altitude_lidar);
        break;
case LOG_PARAM_GROUND_SPEED:        // ground speed in m/s [total/E/N/U] (f16)
        if (index==0)
            value = Float32toFloat16(hfc->gps_speed);
        else
            value = Float32toFloat16(hfc->IMUspeedGroundENU[index-1]);
        break;
case LOG_PARAM_HELI_SPEED:          // heli speed in m/s [R/F/U] (f16)
        value = Float32toFloat16(hfc->speedHeliRFU[index]);
        break;
case LOG_PARAM_CPU:                 // CPU stats [process_period in uS int / utilization in % f16]
        if (index==0)
            value = hfc->ticks_curr;
        else
            value = Float32toFloat16(hfc->cpu_utilization_curr);
        break;
case LOG_PARAM_TIME:                // CPU time in ms (int)
        value = hfc->time_ms;
        break;
case LOG_PARAM_POSITION:            // heli position relative to home [horizontal distance / altitude difference] in m (f16)
        if (index==2)
            value = Float32toFloat16(hfc->gps_to_home[2]);     // 0-distance/1-heading/2-altitude difference
        else
        {
            float course;
            float dist = DistanceCourse(hfc->positionLatLon[0], hfc->positionLatLon[1], hfc->home_pos[0], hfc->home_pos[1], &course);
            if (index==0)
                value = Float32toFloat16(dist);
            else
                value = Float32toFloat16(course);
        }
        break;
    }
    return value;
}

/*
    char    streaming_enable;       // enables data streaming
    byte    streaming_types[7]      // selects individual data types for streaming
    byte    streaming_channels;     // number of channels included
    byte    streaming_channel;      // current channel
    byte    streaming_samples;      // samples per packet, multiple channel caount as one sample
    byte    streaming_sample;       // current sample
    f16     stream_data[120];
*/

/* returns true when the internal storage is full and the packets needs to be generated */
bool Streaming_Process(FlightControlData *hfc)
{
    if (!hfc->streaming_enable)
        return false;

    byte param = hfc->streaming_types[Min(6, hfc->streaming_channel)];
    byte index = hfc->streaming_sample*hfc->streaming_channels + hfc->streaming_channel;
    if (index>=120)
    {
        /* something went wrong, disable streaming */
        hfc->streaming_enable = false;
        return false;
    }
    /* get and store the value */
    hfc->stream_data[index] = Streaming_GetValue(hfc, param);
    
    /* increment channel, then sample */
    hfc->streaming_channel++;
    if (hfc->streaming_channel>=hfc->streaming_channels)
    {
        hfc->streaming_channel = 0;
        hfc->streaming_sample++;
        if (hfc->streaming_sample>=hfc->streaming_samples || hfc->profile_mode==PROFILING_FINISH)
        {
            /* cache is full */
            hfc->streaming_sample = 0;
            return true;
        }
    }
    return false;
}

void Profiling_Process(FlightControlData *hfc, const ConfigData *pConfig)
{
    if (hfc->profile_mode == PROFILING_START)
    {
        int i;
        //DigitalOut **LEDs = (DigitalOut**)hfc->leds;
        
        //LEDs[0]->write(1); LEDs[1]->write(1); LEDs[2]->write(1); LEDs[3]->write(1);
        hfc->profile_mode = PROFILING_ON;
        hfc->profile_start_time = GetTime_ms();
        
        /* save current control values to detect any motion during the auto-profiling and terminate it if necessary */
        for (i=0; i<5; i++) hfc->ctrl_initial[i] = hfc->ctrl_out[RAW][i];
    }
        
    /* automatic profiling control */
    if (hfc->profile_mode == PROFILING_ON)
    {
        int i;
        //DigitalOut **LEDs = (DigitalOut**)hfc->leds;
        float *controlled = &hfc->ctrl_out[RAW][hfc->profile_ctrl_variable];    // variable being controlled, RAW here means the same stick input
        float delta = 0;            // value added to the controlled variable
        int t = GetTime_ms() - hfc->profile_start_time;  // timeline in ms
        int Td = hfc->profile_lag;
        int Tt = hfc->profile_period;
        float dC = hfc->profile_delta*0.001f*pConfig->Stick100range;
        
//        printf("%d %d %d %4.3f\n", t, Td, Tt, dC);
        if (t>(Td+5*Tt))        // test complete
        {
//            printf("profiling finish\r\n");
            hfc->profile_mode = PROFILING_FINISH;
            //LEDs[0]->write(0); LEDs[1]->write(0); LEDs[2]->write(0); LEDs[3]->write(0);
            return;
        }
        else if (t>(Td+4*Tt))   // test complete
            delta = 0;
        else if (t>(Td+3*Tt))   // third leg
            delta = dC;
        else if (t>(Td+1*Tt))   // second leg
            delta = -dC;
        else if (t>Td)          // first leg
            delta = dC;
        
        /* terminate auto-profiling if any stick motion is detected */
        for (i=0; i<5; i++)
            if (ABS(hfc->ctrl_initial[i] - hfc->ctrl_out[RAW][i]) > AUTO_PROF_TERMINATE_THRS)
            {
//                printf("Profiling terminated\r\n");
                hfc->profile_mode = PROFILING_OFF;
                hfc->streaming_enable = false;
                delta = 0;
                //LEDs[0]->write(0); LEDs[1]->write(0); LEDs[2]->write(0); LEDs[3]->write(0);
                break;
            }

        *controlled = (*controlled) + delta;
    }
}

void GyroCalibDynamic(FlightControlData *hfc)
{
    int i;
    for (i=0; i<3; i++)
      hfc->gyroOfs[i] += hfc->gyro_lp_disp[i];
}

void CalibrateSensors(FlightControlData *hfc, float gB[3], ConfigData *pConfig)
{
    int i;
    //DigitalOut **LEDs = (DigitalOut**)hfc->leds;
    
    if (!hfc->calibrate)
        return;

    /* when gyro temp curve is enabled, use secondary offsets to eliminate the drift */
    if (!pConfig->gyro_fixed_offsets)
    {
        GyroCalibDynamic(hfc);

        printf("= Calibrated: ");
        printf("%2.3f %2.3f %2.3f at %5.2fdeg\r\n", hfc->gyroOfs[0], hfc->gyroOfs[1], hfc->gyroOfs[2], hfc->gyro_temp_lp);
        hfc->calibrate = 0;
        return;
    }

    /* in fixed gyro temperature compensation mode, average the raw gyro values and use them as offsets */
    if (hfc->calibrate==CALIBRATE_SAMPLES)
    {
      for (i=0; i<3; i++) hfc->calib_gyro_avg[i]  = gB[i];
      hfc->calib_count=1;
    }
    else
    {
      for (i=0; i<3; i++) hfc->calib_gyro_avg[i]  += gB[i];
      hfc->calib_count++;
      if ((hfc->calib_count & 0x7f)==0)
      {
        printf("%2.3f %2.3f %2.3f\r\n", hfc->calib_gyro_avg[0]/hfc->calib_count, hfc->calib_gyro_avg[1]/hfc->calib_count, hfc->calib_gyro_avg[2]/hfc->calib_count);
        //for (i=0; i<4; i++) LEDs[i]->write(!LEDs[i]->read());
      }
    }
    hfc->calibrate--;
    if (!hfc->calibrate)
    {
      for (i=0; i<3; i++)
      {
        hfc->rw_cfg.gyro_ofs[i]  = hfc->calib_gyro_avg[i]/hfc->calib_count;
        hfc->calib_gyro_avg[i]  = 0;
        hfc->gyroOfs[i] = 0;
      }
      //LEDs[0]->write(0); LEDs[1]->write(0); LEDs[2]->write(0); LEDs[3]->write(0);
      printf("= Calibrated: ");
      printf("%2.3f %2.3f %2.3f at %5.2fdeg\r\n", hfc->rw_cfg.gyro_ofs[0], pConfig->gyro_ofs[1], pConfig->gyro_ofs[2], hfc->gyro_temp_lp);
      {
        FILE *fp = fopen("/local/gyro_cal.txt", "w");  // Open "out.txt" on the local file system for writing
        if (fp)
        {
            fprintf(fp, "%f\t%f\t%f\t%f\r\n", hfc->gyro_temp_lp, hfc->rw_cfg.gyro_ofs[0], pConfig->gyro_ofs[1], pConfig->gyro_ofs[2]);
            fclose(fp);
        }
      }      
    }
}

// "kick" or "feed" the dog - reset the watchdog timer
// by writing this required bit pattern
void WDT_Kick()
{
    LPC_WDT->WDFEED = 0xAA;
    LPC_WDT->WDFEED = 0x55;
}

/* timeout in seconds */
void WDT_Init(float timeout)
{
    LPC_WDT->WDCLKSEL = 0x1;                // Set CLK src to PCLK
    uint32_t clk = SystemCoreClock / 16;    // WD has a fixed /4 prescaler, PCLK default is /4
    LPC_WDT->WDTC = timeout * (float)clk;
    LPC_WDT->WDMOD = 0x3;                   // Enabled and Reset
    WDT_Kick();
}

/* to be called during startup, returns true if the reset by WDT timeout,
** an external reset otherwise */
bool WDT_ResetByWDT()
{
    if ((LPC_WDT->WDMOD >> 2) & 1)
        return true;
    else
        return false;
}

/**
  * @brief  CRC32b utility
  * @param  *data_p: pointer to data block to be crc'd
  * @param  length: length of data to be crc'd
  * @retval the calculated crc
  */
uint32_t crc32b(unsigned char *message, uint32_t length)
{
   int i, j;
   unsigned int byte_val, crc, mask;

   i = 0;
   crc = 0xFFFFFFFF;
   while (length--) {
      byte_val = message[i];            // Get next byte.
      crc = crc ^ byte_val;
      for (j = 7; j >= 0; j--) {    // Do eight times.
         mask = -(crc & 1);
         crc = (crc >> 1) ^ (0xEDB88320 & mask);
      }
      i = i + 1;
   }
   return ~crc;
}

