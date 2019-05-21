#ifndef GPS_h
#define GPS_h

#include <stdlib.h>

#define MAX_GPS_CHANNELS		2	// defines the maximum number of GPS channels supported

#define GPS_FIX_NONE        0
#define GPS_FIX_OK          1
#define GPS_FIX_DIFF        2

#define GPS_CHECK_OK        0
#define GPS_CHECK_NOFIX     1
#define GPS_CHECK_PDOP      2
#define GPS_CHECK_ALTITUDE  3
#define GPS_CHECK_POSITION  4
#define GPS_CHECK_NOSIGNAL  5

typedef struct
{
    unsigned char fix;
    unsigned long date;
    unsigned long time;
    float altitude; // meters
    unsigned char sats;
    int       lat;  //  *10M
    int       lon;  //  *10M
    float     latF;
    float     lonF;
    double    latD;
    double    lonD;
    unsigned short int PDOP; // *100
    float HspeedC;
    float courseC;
    float speedENU[3];
} GpsData;

class GPS
{

public:
    typedef struct {
        unsigned char gps_id;         /* Current GPS in use */
        unsigned char sats_fix;       /* 0-1 fix, 2-7 sats */
        unsigned char gps_status;     /* Status of GPS units. Bit 0 = Gps 0 State, Bit 1 = GPS 1 State */
        unsigned char other_sats_fix; /* sats_fix from other GPS */
        int altitude; // mm
    } GpsAviCanMsg0;

    typedef struct {
        int     lat;        //*10M
        int     lon;        //*10M
    } GpsAviCanMsg1;

    typedef struct {
        short int   speedE;     // m/s *100
        short int   speedN;     // m/s *100
        short int   speedU;     // m/s *100
        unsigned short  PDOP;   // *100
    } GpsAviCanMsg2;

    typedef struct {
        unsigned int date;
        unsigned int time;
    } GpsAviCanMsg3;

    typedef struct {
        short int glitch_data;
        short int crc_err_cnt;
        int msg_cnt;
    } GpsAviCanMsg4;

    typedef struct {
        GpsAviCanMsg0 msg0;
        GpsAviCanMsg1 msg1;
        GpsAviCanMsg2 msg2;
        GpsAviCanMsg3 msg3;
        GpsAviCanMsg4 msg4;
        int new_data;
        int glitch;
        int last_update; // tracks uS from last update
        int glitches;
        short int speedENUprev[3];  // for glitch detect
        short int speedENU[3];      // new filtered values
        int lat_prev;   // for glitch detect
        int lon_prev;   // for glitch detect
        int alt_prev;
        unsigned char fix;
        unsigned char online_;
        unsigned char other_fix_;
        unsigned short int glitch_data_;    // As detected on the reporting node
        unsigned short int crc_err_cnt_;    // from the reporting node
        unsigned int msg_cnt_;              // from the reporting node
    } GPSChannelData;

public:
    int selected_channel_;
    int glitch_;
    int glitches_;
    GpsData gps_data_;

    int Init(int number_of_channels);
    void AddGpsData(int channel, int msg_id, char *msg);
    GpsData GpsUpdate(int dTus, char* new_data_flag);

    int GpsUpdate(void);

    GpsData GetGpsData(void);
    void GetFixSatsOther(char *pfix, char *psats);
    int PreFlightCheck(void);

    void SetNextChannel() { set_next_channel_ = 1; }
    int GetLastUpdate() { return (gps_channel_[selected_channel_].last_update); }

    GPS();   // Constructor

private:
    GPSChannelData gps_channel_[MAX_GPS_CHANNELS];
    int number_of_channels_;
    int set_next_channel_;

    void CalculateSpeedCourse();
    void GlitchDetect(int channel);
    void FindNewChannel();
};

#endif
