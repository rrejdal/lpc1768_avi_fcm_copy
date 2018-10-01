/*****************************************************************************
 * File name: IMU_calib.cpp
 * Purpose: Helper Functions for calibrating accelerometer and gyroscope
 * 			of IMU. Gains and offsets of the accelerometer and calculated
 * 			while only the gains for the gyroscope are calculated here since
 * 			the offsets are temperature dependent and measured separately.
 * 			See main.cpp and the #TEMP_CHAMBER flag for more information about
 * 			GYRO temperature compensation.
 *
 * 	@author: Mark Malak Ramzy Ibrahim
 * 	@Date:  1-March-2018 (01/03/2018)
 * 	@version: 1.0
 ******************************************************************************/

#include "IMU_calib.h"
#include "stdio.h"
#include "mbed_wait_api.h"
#include "mymath.h"
#include "PID.h"
#include "utils.h"
#include "PwmOut.h"
#include "string.h"

static int accel_calib(MPU6050* mpu, NokiaLcd* myLcd, DigitalIn* btnEnter, DigitalIn* btnCancel, double Ca[3][3], double aofs[3]);
static int  gyro_calib(MPU6050* mpu, NokiaLcd* myLcd, DigitalIn* btnEnter, DigitalIn* btnCancel, double Cg[3][3], double gofs[3]);

static void  gyro_valid(MPU6050* mpu, NokiaLcd* myLcd, DigitalIn* btnEnter, DigitalIn* btnCancel, double Cg[3][3], double gofs[3]);
static void accel_valid(MPU6050* mpu, NokiaLcd* myLcd, DigitalIn* btnEnter, DigitalIn* btnCancel, double Ca[3][3], double aofs[3]);

static void calibrate(float data_raw[3], float data_cali[3], double C_matrix[3][3], double ofs[3]);
static void write_calib_file(int accel_done, int gyro_done,double Ca[3][3], double aofs[3],double Cg[3][3], double gofs[3]);

void getAccelOrient(float *AccelOrient, float *AccData);
/* copies config file line into str, which starts with string name */
static char Search_file(const char *name, char *str, const char* filename);
static char Load_Double(const char *name, double *value, int N, const char* filename);
static char *FindNext(char *pstr);
static void refresh_LCD( NokiaLcd* myLcd );

static void integrate_matrix3x3(float data[3], float data_int[3], float dT);
static void      add_matrix3x3( double m1[3][3], double m2[3][3], double m[3][3]);
static void subtract_matrix3x3( double m1[3][3], double m2[3][3], double m[3][3]);
static void multiply_matrix3x3( double m1[3][3], double m2[3][3], double m[3][3]);
static void  inverse_matrix3x3( double  m[3][3] );
static void print_matrix3x3( double m[3][3] );

static void easter_egg( NokiaLcd* myLcd );

void compass_calib_helper(HMC5883L* compass, NokiaLcd* myLcd, DigitalIn* btnEnter, DigitalIn* btnCancel)
{
	int ticks = 0;
	float dT = 0;
	int i = 0;
	int counter = 0;

	char str[80];
	FILE *fcompass;
	int calib_type = 0;
	const char* calib[] = {"ELLIPSOID","MAX/MIN"};

	float compassMin[3] = {0};
	float compassMax[3] = {0};
	float ofs[3];
	float gain[3];

	myLcd->SetLine(0, (char*)"COMPASS CALIB:", 0);
	myLcd->SetLine(1, (char*)"CHOOSE BETWEEN", 0);
	myLcd->SetLine(2, (char*)"ELLIPSOID OR  ", 0);
	myLcd->SetLine(3, (char*)"MIN/MAX CALIB ", 0);
	myLcd->SetLine(4, (char*)"              ", 0);
	myLcd->SetLine(5, (char*)"ELIP   MIN/MAX", 0);
	refresh_LCD(myLcd);

	while(1)
	{
		if( btnEnter->read() == 0 )
		{
			calib_type = 0;
			break;
		}
		else if( btnCancel->read() == 0 )
		{
			calib_type = 1;
			break;
		}
	}

	myLcd->SetLine(0, (char*)"COMPASS CALIB:", 0);
	myLcd->SetLine(1, (char*)"SELECTED      ", 0);
    sprintf(str,"--> %s ", calib[calib_type]);
	myLcd->SetLine(2, str, 0);
	myLcd->SetLine(3, (char*)"              ", 0);
	myLcd->SetLine(4, (char*)"              ", 0);
	myLcd->SetLine(5, (char*)"              ", 0);
	refresh_LCD(myLcd);
	wait_ms(1000);

	if( calib_type == 0 )
	{
		myLcd->SetLine(0, (char*)"ELLIPSOID CAL:", 0);
		myLcd->SetLine(1, (char*)"-MOUNT COMPASS", 0);
		myLcd->SetLine(2, (char*)"ON FULL DRONE ", 0);
		myLcd->SetLine(3, (char*)"-PRESS ENTER  ", 0);
		myLcd->SetLine(4, (char*)"TO START DATA ", 0);
		myLcd->SetLine(5, (char*)"ENTER   CANCEL", 0);
		refresh_LCD(myLcd);
		wait_ms(1000);

		while(1)
		{
			if( btnEnter->read() == 0 || btnCancel->read() == 0  )
			{
				break;
			}
			else if( btnCancel->read() == 0 )
			{
				return;
			}
		}

		fcompass = fopen("/local/compass.txt", "w");
		fprintf(fcompass,"COMP_X,COMP_Y,COMP_Z");

		myLcd->SetLine(0, (char*)"-MOVE DRONE IN", 0);
		myLcd->SetLine(1, (char*)"FIGURE 8 AND  ", 0);
		myLcd->SetLine(2, (char*)"SPIN CW 2X,CCW", 0);
		myLcd->SetLine(3, (char*)"2X, DRONE UP/D", 0);
		myLcd->SetLine(4, (char*)"STOP WHEN RDY ", 0);
		myLcd->SetLine(5, (char*)"          STOP", 0);
		refresh_LCD(myLcd);
		wait_ms(1000);

	    SysTick_Run();
		while(1)
		{
			ticks = Ticks_us();
			dT = (ticks*0.000001f);
			if (compass->getRawValues(dT))
			{
				/*dataXYZcalib[] has its indices assigned to be in line with the
				 * orientation of the drone as set by config file CompOrient values*/
				fprintf(fcompass,"\r\n%d,%d,%d,",
						compass->dataXYZ[COMP_X],
						compass->dataXYZ[COMP_Y],
						compass->dataXYZ[COMP_Z]);

				/*Limit file size to about 500 kbytes*/
				if(ftell(fcompass) >= 500000)
				{
					printf("\r\nFile is Full!\r\n");
					fclose(fcompass);
					break;
				}
			}

			if( btnCancel->read() == 0 )
			{
				fclose(fcompass);
				break;
			}
		}

		myLcd->SetLine(0, (char*)"COMPASS CALIB:", 0);
		myLcd->SetLine(1, (char*)"RAW DATA IN   ", 0);
		myLcd->SetLine(2, (char*)" COMPASS.TXT  ", 0);
		myLcd->SetLine(3, (char*)"MOUNT MBED AND", 0);
		myLcd->SetLine(4, (char*)"RUN MATLAB    ", 0);
		myLcd->SetLine(5, (char*)"SCRIPT ON DATA", 0);
		refresh_LCD(myLcd);
		wait_ms(4000);

	}
	else if( calib_type == 1 )
	{
		myLcd->SetLine(0, (char*)"STABLIZE NUMS ", 0);
		myLcd->SetLine(1, (char*)" MAX  ::  MIN ", 0);

	    SysTick_Run();
		while(1)
		{
			ticks = Ticks_us();
			dT = (ticks*0.000001f);
			counter += ticks;

			if (compass->getRawValues(dT))
			{
				for (i=0; i<3; i++)
				{
					compassMin[i] = min(compassMin[i], compass->dataXYZ[i]);
				}
				for (i=0; i<3; i++)
				{
					compassMax[i] = max(compassMax[i], compass->dataXYZ[i]);
				}
			}

			if(counter % 1000000 <= ticks)
			{
				sprintf(str,"X>%5.1f::%5.1f",compassMax[0],compassMin[0]);
				myLcd->SetLine(2, str, 0);
				sprintf(str,"Y>%5.1f::%5.1f",compassMax[1],compassMin[1]);
				myLcd->SetLine(3, str, 0);
				sprintf(str,"Z>%5.1f::%5.1f",compassMax[2],compassMin[2]);
				myLcd->SetLine(4, str, 0);
				myLcd->SetLine(5, (char*)"DONE?   CANCEL", 0);
				refresh_LCD(myLcd);
			}

			if( btnEnter->read() == 0 )
				break;
			else if( btnCancel->read() == 0 )
				return;
		}

		for (i=0; i<3; i++)
		{
			ofs[i] = (compassMin[i]+compassMax[i]+1)/2;
		}
		for (i=0; i<3; i++)
		{
			gain[i] = 500.0f/((compassMax[i]-compassMin[i])/2.0f);
		}

		fcompass = fopen("/local/COMP_CALIB.txt", "w");
		if (fcompass)
		{
			fprintf(fcompass, "ofs\t%f\t%f\t%f\r\n", ofs[0], ofs[1], ofs[2]);
			fprintf(fcompass, "gains\t%f\t%f\t%f\r\n", gain[0], gain[1], gain[2]);
			fclose(fcompass);
		}

		myLcd->SetLine(0, (char*)"COMPASS CALIB:", 0);
		myLcd->SetLine(1, (char*)"CALIBRATION   ", 0);
		myLcd->SetLine(2, (char*)"FILE CREATED  ", 0);
		myLcd->SetLine(3, (char*)"MOUNT MBED SEE", 0);
		myLcd->SetLine(4, (char*)"COMP_CALIB.TXT", 0);
		myLcd->SetLine(5, (char*)"              ", 0);
		refresh_LCD(myLcd);
		wait_ms(4000);
	}

	return;
}

/*GYRO Temperature Offset Calibration routine
 * Indefinite Loop to print out GYRO X,Y and Z while changing temperature
 * Conditions:
 * - must be using awesome-sauce temperature chamber
 * - Temp chamber sits inside a freezer, waterbottles help as cold batteries
 * PID Controller
 * - control the RATE OF CHANGE OF TEMPERATURE of the IMU
 *   In this case the PID is used to control the rate of increase of
 *   temperature of the IMU by modifying the the current_ctrl pwm siganal
 * - pid_temp.decay = indicator whether GYRO is heating (0) or cooling (1)
 * - pid_temp.acceleration = The rate of change in temperature in degC/s
 * Control Signals
 * - PwmOut* current_ctrl: used to drive the gate of an NFET transistor, as the
 *   	throttle increases the current through the NFET increases.  The NFET
 *   	feeds current to resistors mounted on a metal plate that the IMU is
 *   	sitting on. Therefore, increase throttle = increase current = increase
 *   	IMU temperature
 *   	period = 1s , max hi pulse = 0.7 s, duty cycle = 70%
 *   	VDD = 12 V, R_eq_total = 1.5 ohm
 *   	Imax = 8A, Pmax_inst = 96 W, Pavg = 96 W * 0.7 = 67.2 W
 * - PwmOut* door_ctrl: used to open door for cooling and close door for heating
 *      period = 3333us, 300Hz
 *      door closed pwm = 1735 ----> pid_temp.decay = 0
 *      door open pwm = 1180   ----> pid_temp.decay = 1
 */
void gyro_temp_calib(MPU6050* mpu,  NokiaLcd* myLcd,
		DigitalIn* btnCancel, Serial* pc)
{
	char c;
	char str[40];

	int ticks;
	double counter = 0;
	float dT;

	PwmOut current_ctrl(p26); //S1 on FCM;
	int current_pwm = 0;

	PwmOut door_ctrl(p25); //S2 on FCM
	int door_open = 1180;
	int door_close = 1735;
	int door_pwm = door_close;

	// max and min temperature for Chamber to reach
	T_PID pid_temp;
	float max_temp = 50;
	float min_temp = -10;

	float accRaw[3];
	float gyroRaw[3];
	float temp;

	/* Set period for current control servo to 1s*/
	current_ctrl.period_us(1000000);

	/* FCM_SERVO_CH2 is used to control the TEMP_CHAMBER door
	 * - OPEN when the IMU is cooling, decay flag is 1
	 * - CLOSE when IMU is heating, decay flag is 0
	 * - period is 300 HZ = 3333 us
	 * - initialize to OPEN*/
	door_ctrl.period_us(3333);


	/*mmri:  Create/Initialize the PID object to control rate of Delta_TEMP
	 *                            Kp    Ki      Kd  COofs  COmax  COmin*/
	float temp_pid_params[6] = {3.00, 0.02, 0.0020,   0.0,700000,     0};
	PID_Init(&pid_temp, temp_pid_params, 0, 0);

	/* set initial rate of change of temp for IMU in degC/s
	 * 0.2 degC/min = 0.003333 degC/s
	 * 0.5 degC/min = 0.008333 degC/s
	 * 1.0 degC/min = 0.016667 degC/s
	 * 2.0 degC/min = 0.033333 degC/s*/
	pid_temp.acceleration = 0.033333;

	/* decay flag used to indicate if IMU temp is rising (0) or dropping (1)
	 * - initialize to 1 always*/
	pid_temp.decay = 0;

    SysTick_Run();

    printf("\r\n,Time,Throttle,Temperature,Pitch,Roll,Yaw,");
	while( 1 )
	{
		current_ctrl.pulsewidth_us(current_pwm);
		door_ctrl.pulsewidth_us(door_pwm);

		ticks = Ticks_us();
		dT = (float)ticks * 0.000001f;
		counter += ticks;

		mpu->readMotion7_blocking(accRaw, gyroRaw, &temp);

		// print data every 0.1 seconds
		if(fmod(counter,100000) <= ticks)
			printf("\r\nTime-Thr-T-drift[P R Y],"
					"%7.3f,%d,%+6.4f,%f,%f,%f,",
					counter*0.000001f, current_pwm, temp,
					gyroRaw[0],gyroRaw[1], gyroRaw[2]);

		/* decay flag used to shut off PID
		 * - when decay = 0, PID raises temp
		 * - when decay = 1 PID is shut off, throttle = 0*/
		if ( (pid_temp.decay == 0) && (temp > max_temp) )
		{
			current_pwm = 0;
			pid_temp.decay = 1;
			printf("\r\n\r\n Cooling Down\r\n ,"
					"Time,Throttle,Temperature,"
					"Pitch,Roll,Yaw,");
		}
		else if ( (pid_temp.decay == 1) && (temp < min_temp) )
		{
			pid_temp.decay = 0;
			printf("\r\n\r\n Heating Up\r\n ,"
					"Time,Throttle,Temperature,"
					"Pitch,Roll,Yaw,");
		}
		else if (pid_temp.decay == 0)
		{
			current_pwm = PID_temp(&pid_temp, temp, dT, current_pwm);
		}

		// change the door servo pwm by 1us every 7ms
		if ( fmod(counter,10000) <= ticks )
		{
			if( pid_temp.decay == 0 )
			{
				door_pwm += 1;
				if( door_pwm >= door_close )
					door_pwm = door_close;
			}
			if( pid_temp.decay == 1 )
			{
				door_pwm -= 1;
				if( door_pwm <= door_open )
					door_pwm = door_open;
			}
		}

	    if( pc->readable() )
		{
	    	c = pc->getc();

	    	if (c=='q')
		    {
				pid_temp.acceleration += 0.001667;
				if (pid_temp.acceleration >= 0.083333)
					pid_temp.acceleration = 0.083333;
				printf("====Temperature Rate of Change = %f deg/min",
						pid_temp.acceleration*60);
		    }
			// if user inputs 'a' on MBED serial port, decrease throttle width by 10
		    else if (c=='a')
		    {
		    	pid_temp.acceleration -= 0.001667;
				if (pid_temp.acceleration <= 0.001667)
					pid_temp.acceleration = 0;
		        printf("====Temperature Rate of Change = %f deg/min",
		        		pid_temp.acceleration*60);
		    }
		}


		if( fmod(counter,200000) <= ticks)
		{
			myLcd->SetLine(0, (char*)"GYRO TEMP CALI", 0);
			sprintf(str,"TEMP:%+2.1fdegC",temp);
			myLcd->SetLine(1, str, 0);
			myLcd->SetLine(2, (char*)"GYRO X  Y  Z  ", 0);
			sprintf(str,"%4.2f,%4.2f,%4.2f",gyroRaw[0],gyroRaw[1],gyroRaw[2]);
			myLcd->SetLine(3, str, 0);

			if( pid_temp.decay == 0 )
				myLcd->SetLine(4, (char*)"HEATING,DoorCL", 0);
			else if( pid_temp.decay == 1 )
				myLcd->SetLine(4, (char*)"COOLING,DoorOP", 0);
			else if( pid_temp.decay == 1 )
				myLcd->SetLine(4, (char*)"COOLING,DOOROP", 0);
			else if( door_pwm > door_close )
				myLcd->SetLine(4, (char*)"CLOSING DOOR! ", 0);
			else if( door_pwm < door_open )
				myLcd->SetLine(4, (char*)"OPENING DOOR! ", 0);

			myLcd->SetLine(5, (char*)"        CANCEL", 0);

			refresh_LCD(myLcd);
		}

		if( btnCancel->read() == 0 )
			break;
	}

	printf("\r\n Exiting TEMP LOOP!!\r\n");

	wait_ms(1000);
	return;

}

int imu_calib_helper(MPU6050* mpu, NokiaLcd* myLcd, DigitalIn* btnEnter, DigitalIn* btnCancel)
{
	char str[40];

	int i;

	const char* accel_files[] = {"NO CALIB",
	   	   	 	 	 	 	 	 "CURRENT CALIB",
						   	   	 "/local/GAIN_CAL.TXT",
								 "/local/ACC_GAIN.TXT",
						   	   	 "/local/CONFIG.TXT"};

	const char* gyro_files[] = {"NO CALIB",
	 	 	 	 	 	 	 	"CURRENT CALIB",
								"/local/GAIN_CAL.TXT",
								"/local/GYRO_GAIN.TXT",
								"/local/CONFIG.TXT"};

	//if 0, no calib data read
	//if 1, calib data read from current calibration
	//if 2, data read from GAIN_CAL.txt
	//if 3, data read from ACC_GAIN.txt || GYRO_GAIN.txt
	//if 4, data read from config.txt
	int accel_done = 0;
	int gyro_done = 0;

	int redo_cali = 0;


	double Ca[3][3] = {0};  // calibration matrix for accelerometer
	double  aofs[3] = {0};	// x,y,z offsets of accelerometer

	double Cg[3][3] = {0};  // calibration matrix for gyroscope
	double  gofs[3] = {0};// x,y,z gyro offsets are handled in Temp calibration

	wait_ms(1000);
	printf("\r\n----------------------\r\n");
	printf("IMU Calibration Helper\r\n");
	printf("----------------------\r\n");
	printf("Instructions Available on FCM LCD as well\r\n");
	printf("Accelerometer Calibration:\r\n"
		   "- Press ENTER to continue or press SKIP on FCM\r\n");
	fflush(stdout);


    myLcd->SetLine(0, (char*)"CALIB FOR IMU:", 0);
    myLcd->SetLine(1, (char*)"ACCEL CALIB:  ", 0);
    myLcd->SetLine(2, (char*)"ENTER TO CNT  ", 0);
    myLcd->SetLine(3, (char*)"OR SKIP       ", 0);
    myLcd->SetLine(4, (char*)"              ", 0);
	myLcd->SetLine(5, (char*)"ENTER     SKIP", 0);
    refresh_LCD(myLcd);

	while(1)
	{
		if( btnEnter->read() == 0 )
		{
			printf("\r\n--->Starting Accelerometer Calibration\r\n");
			wait_ms(1000);
			accel_done = accel_calib(mpu, myLcd, btnEnter, btnCancel, Ca, aofs);
			break;
		}
		else if( btnCancel->read() == 0 )
		{
			break;
		}
	}

	wait_ms(1000);

	printf("\r\n\r\nGyroscope Calibration:\r\n"
		   "- Press ENTER to continue or press SKIP on FCM\r\n");
	fflush(stdout);

    myLcd->SetLine(0, (char*)"CALIB FOR IMU ", 0);
    myLcd->SetLine(1, (char*)"GYRO CALIB:   ", 0);
    myLcd->SetLine(2, (char*)"ENTER TO CNT  ", 0);
    myLcd->SetLine(3, (char*)"OR SKIP       ", 0);
    myLcd->SetLine(4, (char*)"              ", 0);
	myLcd->SetLine(5, (char*)"ENTER     SKIP", 0);
    refresh_LCD(myLcd);

	while(1)
	{
		if( btnEnter->read() == 0 )
		{
			printf("\r\n--->Starting Gyroscope Gain Calibration\r\n");
			wait_ms(500);

			/* Configure Interrupt flag for IMU data- WARNING!!
			 * - set up flag to trigger when IMU data is ready
			 * - checked using mpu->getIntStatus() function
			 * WARNING! - must be set back to zero at end of function*/
			mpu->setIntDataReady(1);

			gyro_done = gyro_calib(mpu, myLcd, btnEnter, btnCancel, Cg, gofs);

			/*WARNING: MUST USE THESE LINES OF CODE!
			 * Revert back to defaults of MPU6050 IMU */
			mpu->setIntDataReady(0);

			break;
		}
		else if( btnCancel->read() == 0 )
		{
			break;
		}
	}

	wait_ms(1000);
	myLcd->SetLine(0, (char*)"EXITING       ", 0);
	myLcd->SetLine(1, (char*)"CALIBRATION   ", 0);
	myLcd->SetLine(2, (char*)"PROCEDURE...  ", 0);
	myLcd->SetLine(3, (char*)"THANK YOU!!!  ", 0);
	myLcd->SetLine(4, (char*)"              ", 0);
	myLcd->SetLine(5, (char*)"              ", 0);
	refresh_LCD(myLcd);
	wait_ms(1000);


	//Easter Egg for Geoff
	if( accel_done == 1 && gyro_done == 1 )
	{
		easter_egg( myLcd );
	}

	write_calib_file(accel_done, gyro_done, Ca, aofs, Cg, gofs);

	/* Prepare for Validation by making sure that the calibration
	 * matrix and offsets for accelerometer and gyroscope are loaded,
	 * that is, Ca, aofs, Cg and gofs are all set*/
	if( accel_done == 0 )
	{
		for(i = 2; i<=4; i++)
		{
			if( Load_Double("AccXGains", Ca[0], 3, accel_files[i])
			 && Load_Double("AccYGains", Ca[1], 3, accel_files[i])
			 && Load_Double("AccZGains", Ca[2], 3, accel_files[i])
			 && Load_Double("AccOffsets", aofs, 3, accel_files[i]) )
			{
				accel_done = i;
				break;
			}
		}
	}
	if( gyro_done == 0 )
	{
		for(i = 2; i<=4; i++)
		{
			if( Load_Double("GyroXGains", Cg[0], 3, gyro_files[i])
			 && Load_Double("GyroYGains", Cg[1], 3, gyro_files[i])
			 && Load_Double("GyroZGains", Cg[2], 3, gyro_files[i]) )
			{
				gyro_done = i;
				break;
			}
		}
	}

	wait_ms(1000);

	myLcd->SetLine(0, (char*)"DO YOU WANT TO", 0);
	myLcd->SetLine(1, (char*)"RUN VALIDATION", 0);
	myLcd->SetLine(2, (char*)"TESTS FOR THE ", 0);
	myLcd->SetLine(3, (char*)"ACCELEROMETER?", 0);
	myLcd->SetLine(4, (char*)"              ", 0);
	myLcd->SetLine(5, (char*)"YES         NO", 0);
	refresh_LCD(myLcd);

	printf("\r\nDo you want to run validation tests for accelerometer?\r\n"
			"Press YES or NO on FCM...\r\n");

	while(1)
	{
		if( btnEnter->read() == 0 )
		{
			if( accel_done == 0 )
			{
				myLcd->SetLine(0, (char*)"NO CALIB DATA ", 0);
				myLcd->SetLine(1, (char*)"AVAILABLE AND ", 0);
				myLcd->SetLine(2, (char*)"NONE IN ANY   ", 0);
				myLcd->SetLine(3, (char*)"CONFIG FILES..", 0);
				myLcd->SetLine(4, (char*)"EXITITING...  ", 0);
				myLcd->SetLine(5, (char*)"SEE YA!       ", 0);
				refresh_LCD(myLcd);
				wait_ms(5000);
				break;
			}

			myLcd->SetLine(0, (char*)"READING CALIB ", 0);
			myLcd->SetLine(1, (char*)"DATA FROM:    ", 0);
			sprintf(str,"%s", strtok(strdup(accel_files[accel_done]), "/local/") );
			myLcd->SetLine(2, str, 0);
			myLcd->SetLine(3, (char*)"PROCEED WITH  ", 0);
			myLcd->SetLine(4, (char*)"ACCELEROMETER ", 0);
			myLcd->SetLine(5, (char*)"VALIDATION    ", 0);
			refresh_LCD(myLcd);
			wait_ms(5000);

			printf("\r\n--->Starting Accelerometer Validation\r\n");
			accel_valid(mpu, myLcd, btnEnter, btnCancel, Ca, aofs);
			break;
		}
		else if( btnCancel->read() == 0 )
		{
			break;
		}
	}
	wait_ms(1000);


	myLcd->SetLine(0, (char*)"DO YOU WANT TO", 0);
	myLcd->SetLine(1, (char*)"RUN VALIDATION", 0);
	myLcd->SetLine(2, (char*)"TESTS FOR     ", 0);
	myLcd->SetLine(3, (char*)"GYROSCOPE?    ", 0);
	myLcd->SetLine(4, (char*)"              ", 0);
	myLcd->SetLine(5, (char*)"YES         NO", 0);
	refresh_LCD(myLcd);

	printf("\r\nDo you want to run validation tests for gyroscope?\r\n"
			"Press YES or NO on FCM...\r\n");

	while(1)
	{
		if( btnEnter->read() == 0 )
		{
			if( accel_done == 0 )
			{
				myLcd->SetLine(0, (char*)"NO CALIB DATA ", 0);
				myLcd->SetLine(1, (char*)"AVAILABLE AND ", 0);
				myLcd->SetLine(2, (char*)"NONE IN ANY   ", 0);
				myLcd->SetLine(3, (char*)"CONFIG FILES..", 0);
				myLcd->SetLine(4, (char*)"EXITITING...  ", 0);
				myLcd->SetLine(5, (char*)"SEE YA!       ", 0);
				refresh_LCD(myLcd);
				wait_ms(5000);
				break;
			}

			myLcd->SetLine(0, (char*)"READING CALIB ", 0);
			myLcd->SetLine(1, (char*)"DATA FROM:    ", 0);
			sprintf(str,"%s", strtok(strdup(gyro_files[gyro_done]), "/local/") );
			myLcd->SetLine(2, str, 0);
			myLcd->SetLine(3, (char*)"PROCEED WITH  ", 0);
			myLcd->SetLine(4, (char*)"GYROSCOPE     ", 0);
			myLcd->SetLine(5, (char*)"VALIDATION    ", 0);
			refresh_LCD(myLcd);
			wait_ms(5000);

			printf("\r\n--->Starting Gyroscope Validation\r\n");

			/* Configure Interrupt flag for IMU data- WARNING!!
			 * - set up flag to trigger when IMU data is ready
			 * - checked using mpu->getIntStatus() function
			 * WARNING! - must be set back to zero at end of function*/
			mpu->setIntDataReady(1);
			gyro_valid(mpu, myLcd, btnEnter, btnCancel, Cg, gofs);

			/*WARNING: MUST USE THESE LINES OF CODE!
			 * Revert back to defaults of MPU6050 IMU */
			mpu->setIntDataReady(0);

			break;
		}
		else if( btnCancel->read() == 0 )
		{
			break;
		}
	}

	wait_ms(1000);

	myLcd->SetLine(0, (char*)"DO YOU WANT TO", 0);
	myLcd->SetLine(1, (char*)"RECALIBRATE   ", 0);
	myLcd->SetLine(2, (char*)"THE IMU?      ", 0);
	myLcd->SetLine(3, (char*)"              ", 0);
	myLcd->SetLine(4, (char*)"              ", 0);
	myLcd->SetLine(5, (char*)"YES         NO", 0);
	refresh_LCD(myLcd);

	while(1)
	{
		if( btnEnter->read() == 0 )
		{
			redo_cali = 1;
			break;
		}
		else if( btnCancel->read() == 0 )
		{
			redo_cali = 0;
			break;
		}
	}

	wait_ms(1000);
	return redo_cali;
}

/*	Purpose: Prompt user to take measurements to calibrate Accelerometer,
 *           that is, get gains for each of the 6 axes and
 *           calculate the offsets.
 * Inputs:
 * - MPU6050* mpu: pointer to IMU object, used to collect data
 * - NokiaLcd* myLcd: pointer to LCD object, used to deliver instructions to user.
 * - DigitalIn* btnEnter, btnCancel: button objects for user
 * - double Ca[3][3]: Accelerometer calibration matrix
 * - double  aofs[3]: Accelerometer offsets for x,y,z
 * Outputs:
 * - double Ca[3][3]: matrix is calculated and assigned
 * - double  aofs[3]: offsets calculated and assigned
*/
static int accel_calib(MPU6050* mpu, NokiaLcd* myLcd,
		                DigitalIn* btnEnter, DigitalIn* btnCancel,
		                double Ca[3][3], double aofs[3])
{
	char str[40];
	int i;
	int row,col;
	float n_samples   = 3000.0f;

	float accRaw[3];
	float gyroRaw[3];
	float temp;

	//     [row][col]
	double Mp[3][3] = {0};  // measurements for +X, +Y, +Z axes facing up
	double Mn[3][3] = {0};  // measurements for -X, -Y, -Z axes facing up
	double  m[3][3] = {0};  // working matrix

	const char* dir[] = {"RIGHT","FRONT","TOP","LEFT","BACK","BOTTOM"};
	const char* sign[] = {"+","+","+","-","-","-"};
	const char* axis[] = {"X","Y","Z","X","Y","Z"};

	/*There are 6 column vectors to construct with measurements,
	 *-->One accelerometer column vector measured for each face.
	 *-->n_samples measured and summed up for each accel triplet*/
	for(col = 0; col < 6; col++)
	{
		printf("Measurement %d of 6:\r\n"
			   "1. Place the Accelerometer on a level and stationary\r\n"
			   "   surface with the %s side of IMU is facing UP.\r\n"
			   "   Watch the numbers on the FMC LCD and try different\r\n"
			   "   orientations until the %s axis shows %s1.\r\n"
			   "2. When ready, push ENTER on FCM, and avoid\r\n"
			   "   moving the IMU.\r\n"
			   "3. You can skip this measurement by pressing SKIP on FCM\r\n"
			   "\r\nPress ENTER on FCM when ready.\r\n",col+1, dir[col], axis[col], sign[col]);

		//			"12345678901234"
        sprintf(str,"ACCEL- %d OF 6", col+1);
		myLcd->SetLine(0, str, 0);
		sprintf(str,"1.PUT %s     ",dir[col]);
		myLcd->SetLine(1, str, 0);
		myLcd->SetLine(2, (char*)" SIDE FACE UP ", 0);
		myLcd->SetLine(3, (char*)"2.PRESS ENTER ", 0);
		myLcd->SetLine(5, (char*)"ENTER   CANCEL", 0);
		refresh_LCD(myLcd);
		wait_ms(1000);

		while(1)
		{
			mpu->readMotion7_blocking(accRaw, gyroRaw, &temp);
			sprintf(str,"%4.2f,%4.2f,%4.2f",accRaw[0],accRaw[1],accRaw[2]);
			myLcd->SetLine(4, str, 0);
			myLcd->Update();

			if( btnEnter->read() == 0 )
			{
				break;
			}
			else if( btnCancel->read() == 0 )
			{
				return 0;
			}
		}

		printf("--->Accelerometer: Doing measurements for %s axis.\r\n"
			   "                   DO NOT MOVE ACCELEROMETER!\r\n",axis[col]);

		wait_ms(1000);
        sprintf(str,"ACCEL- %d OF 6", col+1);
		myLcd->SetLine(0, str, 0);
		myLcd->SetLine(1,(char*)" MEASURING    ", 0);
		myLcd->SetLine(2,(char*)" DO NOT MOVE  ", 0);
		myLcd->SetLine(3,(char*)" ACCELEROMETER", 0);
		myLcd->SetLine(4,(char*)" !!!!!!       ", 0);
		myLcd->SetLine(5,(char*)"              ", 0);
		refresh_LCD(myLcd);
		wait_ms(1000);

		i = 0; //use 'i' to count number of samples

		// For each face, take n_samples
		while( i < n_samples )
		{
			mpu->readMotion7_blocking(accRaw, gyroRaw, &temp);

			if( col < 3 )
			{
				Mp[0][col] += accRaw[0];
				Mp[1][col] += accRaw[1];
				Mp[2][col] += accRaw[2];
			}
			else
			{
				Mn[0][col-3] += accRaw[0];
				Mn[1][col-3] += accRaw[1];
				Mn[2][col-3] += accRaw[2];
			}

			i++;

			wait_ms(1);
		}

        sprintf(str,"ACCEL- %d OF 6", col+1);
		myLcd->SetLine(0, str, 0);
		myLcd->SetLine(1, (char*)" MEASUREMENT", 0);
		myLcd->SetLine(2, (char*)" COMPLETE!  ", 0);
		myLcd->SetLine(3, (char*)"            ", 0);
		myLcd->SetLine(4, (char*)"            ", 0);
		myLcd->SetLine(5, (char*)"            ", 0);
		refresh_LCD(myLcd);

		printf("--->Accelerometer: Measurement %d of 6 COMPLETE!\r\n\r\n",col+1);

		wait_ms(1000);
	}

	/*Complete averaging of n_samples for each triad*/
	for(col = 0; col < 3; col++)
	{
		for(row = 0; row < 3; row++)
		{
			Mp[row][col] = Mp[row][col] / n_samples;
			Mn[row][col] = Mn[row][col] / n_samples;
		}
	}

	printf("\r\nMp = \r\n"); print_matrix3x3(Mp);
	printf("\r\nMn = \r\n"); print_matrix3x3(Mn);

	//m = Asp - Asn
	subtract_matrix3x3(Mp, Mn, m);
	printf("\r\nMp - Mn = \r\n"); print_matrix3x3(m);


	//m = (Asp - Asn)^(-1)
	inverse_matrix3x3(m);
	printf("\r\n(Mp - Mn)^-1 = \r\n"); print_matrix3x3(m);

	// Get calibration matrix: C = 2(Asp - Asn)^(-1)
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			Ca[i][j] = 2*m[i][j];
		}
	}

	// get the offset vector aOffset for each of the three axes
	add_matrix3x3(Mp, Mn, m);
	for(i = 0; i < 3; i++)
	{
		aofs[i] = ( m[i][0] + m[i][1] + m[i][2] ) / 6;
	}


	printf("\r\nAccOffsets");
	for(int i = 0; i < 3; i++)
	{
		printf("\t\t%f",aofs[i]);
	}

	printf("\r\nCa = \r\n"); print_matrix3x3(Ca);
	fflush(stdout);

	myLcd->SetLine(0, (char*)"ACCEL GAINS:", 0);
	sprintf(str, (char*)"%1.3f %1.3f %1.3f",Ca[0][0],Ca[0][1],Ca[0][2]);
	myLcd->SetLine(1, str, 0);
	sprintf(str, (char*)"%1.3f %1.3f %1.3f",Ca[1][0],Ca[1][1],Ca[1][2]);
	myLcd->SetLine(2, str, 0);
	sprintf(str, (char*)"%1.3f %1.3f %1.3f",Ca[2][0],Ca[2][1],Ca[2][2]);
	myLcd->SetLine(3, str, 0);
	myLcd->SetLine(4, (char*)"ACCEL OFFSETS:", 0);
	sprintf(str, (char*)"%2.2f %2.2f %2.2f",aofs[0],aofs[1],aofs[2]);
	myLcd->SetLine(5, str, 0);
	refresh_LCD(myLcd);

	wait_ms(1000);

	return 1;
}

/*	Purpose: Prompt user to take measurements to calibrate Gyroscope,
 *           that is, get gains for each rotational axis.
 *           Gyroscope offsets are handled by the temperature calibration
 *           since they vary with temperature
 * Inputs:
 * - MPU6050* mpu: pointer to IMU object, used to collect data
 * - NokiaLcd* myLcd: pointer to LCD object, used to deliver instructions to user.
 * - DigitalIn* btnEnter, btnCancel: button objects for user
 * - double Cg[3][3]: gyroscope calibration matrix
 * Outputs:
 * - double Cg[3][3]: matrix is calculated and assigned
*/
static int gyro_calib(MPU6050* mpu, NokiaLcd* myLcd,
		DigitalIn* btnEnter, DigitalIn* btnCancel,
		double Cg[3][3], double gofs[3])
{
	char str[80];
	int i, j;
	int col;
	float n_samples = 5000.0f;

	float accRaw[3];
	float gyroRaw[3];
	float temp;

	int ticks;
	float dT[3] = {0.0f};
	double fs            =  0.0;  // sample frequency of the gyroscope

	double omega_0[3][3] = {0};   // sampled angular velocities, triplet columns
	double omega_s[3][3] = {0};   // sampled angular velocities, triplet columns
	double   omega[3][3] = {0};   // calculated angular velocity matrix based on fs
	double       m[3][3] = {0};   // working matrix

	const char* dir[] = {"RIGHT","FRONT","TOP"};

	int rotations, rotation_samples = 0;
	DigitalIn rotate_sensor(p23); //S4 on FCM
	rotate_sensor.mode(PullUp);

	PwmOut rotation_ctrl(p24); //S3 on FCM
	int stop_pwm = 1500;
	int start_pwm = 1375;

	rotation_ctrl.period_us(3333); // 3.333ms period, 300Hz
	rotation_ctrl.pulsewidth_us(stop_pwm);


	printf("\r\n gyro sample frequency = %f\r\n",fs);

	printf("\r\nGyro Offset Measurement:\r\n"
		   "1. Place the GYRO on a level and stationary surface\r\n"
		   "2. Before pressing ENTER on FMC make sure that the\r\n"
		   "   GYRO is fastened and avoid any vibrations or\r\n"
		   "   disturbances during measurement.\r\n"
		   "3. When ready, push ENTER on FCM, and avoid\r\n"
		   "   moving the IMU.\r\n"
		   "4. You can skip this measurement by pressing SKIP on FCM\r\n"
		   "\r\nPress ENTER on FCM when ready.\r\n");

	//			"12345678901234"
	myLcd->SetLine(0, (char*)"GYRO OFFSET   ", 0);
	myLcd->SetLine(1, (char*)"1.DO NOT MOVE!", 0);
	myLcd->SetLine(2, (char*)"2.FASTEN GYRO ", 0);
	myLcd->SetLine(3, (char*)"3.PRESS ENTER ", 0);
	myLcd->SetLine(4, (char*)"              ", 0);
	myLcd->SetLine(5, (char*)"ENTER   CANCEL", 0);
	refresh_LCD(myLcd);

	while(1)
	{
		if( btnEnter->read() == 0 )
		{
			break;
		}
		else if( btnCancel->read() == 0 )
		{
		    return 0;
		}
	}

	printf("--->Gyroscope: Doing offset measurement.\r\n"
		   "               DO NOT MOVE GYROSCOPE!\r\n");

	myLcd->SetLine(1,(char*)" MEASURING", 0);
	myLcd->SetLine(2,(char*)" DO NOT MOVE", 0);
	myLcd->SetLine(3,(char*)" GYRO!", 0);
	refresh_LCD(myLcd);

	wait_ms(1000);

	i = 0;
	// Take n_samples while gyro is stationary
	while(i < n_samples)
	{
		if( mpu->getIntStatus() )
		{
			mpu->readMotion7_blocking(accRaw, gyroRaw, &temp);

			for(j = 0; j < 3; j++)
            {
                gofs[j] += gyroRaw[j];
            }

			i++;
		}
	}

	for(i = 0; i < 3; i++)
	{
	    gofs[i] = gofs[i] / n_samples;
	    for(j = 0; j < 3; j++)
	    {
	        omega_0[i][j] = gofs[i];
	    }
	}

    printf("\r\ngofs = [%+5.5f %+5.5f %+5.5f]\r\n\r\n",gofs[0],gofs[1],gofs[2]);
    printf("\r\nomega_0 = \r\n");
    print_matrix3x3(omega_0);

	myLcd->SetLine(0, (char*)"GYRO OFFSET", 0);
	myLcd->SetLine(1, (char*)"-MEASUREMENT", 0);
	myLcd->SetLine(2, (char*)" COMPLETE!  ", 0);
	myLcd->SetLine(3, (char*)"            ", 0);
	myLcd->SetLine(4, (char*)"            ", 0);
	refresh_LCD(myLcd);

	printf("--->Gyroscope: Data Collected and averaged for Gyro\r\n"
		   "               offset calculation!\r\n");


	wait_ms(1000);

	/* Loop for three different axis
	 Here the user is prompted to set up the gyroscope and
	 make measurements while rotating along a single axis */
	for(col = 0; col < 3; col++)
	{
		printf("\r\nGain Measurement %d of 3:\r\n"
			   "1. Place the Gyro on a level and stationary\r\n"
			   "   surface with the %s side of IMU facing UP and\r\n"
			   "   centered in the middle of the turn-table.\r\n"
			   "2. When ready, push ENTER\r\n"
			   "   -turn table will rotate counter-clockwise (CCW) until\r\n"
			   "    it reaches start point, that is, optical sensor,\r\n"
			   "    at which point it will begin to make measurements\r\n"
			   "   -once measurements begin, it will rotate three more times\r\n"
			   "    then stop.\r\n"
			   "*******************************************************************\r\n"
			   "WARNING: Please avoid disturbing the IMU with any additional motion.\r\n"
			   "*******************************************************************\r\n"
			   "\r\nPress ENTER on FCM when ready.\r\n",col+1,dir[col]);

		//			"12345678901234"
        sprintf(str,"GYRO- %d OF 3", col+1);
		myLcd->SetLine(0, str, 0);
		sprintf(str,"PUT %s SIDE",dir[col]);
		myLcd->SetLine(1, str, 0);
		myLcd->SetLine(2, (char*)"FACE UP,CENTER", 0);
		myLcd->SetLine(3, (char*)"ON TURN TABLE ", 0);
		myLcd->SetLine(4, (char*)"2.ENTER, AVOID", 0);
		myLcd->SetLine(5, (char*)"ENTER   CANCEL", 0);
		refresh_LCD(myLcd);

		while(1)
		{
			if( btnEnter->read() == 0 )
			{
				break;
			}
			else if( btnCancel->read() == 0 )
			{
				return 0;
			}
		}

		wait_ms(1000);
		myLcd->SetLine(1, (char*)" MEASURING!   ", 0);
		myLcd->SetLine(2, (char*)" STAND CLEAR!!", 0);
		myLcd->SetLine(3, (char*)" WILL STOP AFT", 0);
		myLcd->SetLine(4, (char*)" 3 ROTATIONS! ", 0);
		myLcd->SetLine(5, (char*)"              ", 0);
		refresh_LCD(myLcd);

		wait_ms(1000);
		rotation_ctrl.pulsewidth_us(start_pwm);
		wait_ms(1000);

		rotations = 0;
		rotation_samples = 0;
		n_samples = 0;

		// wait until I hit the sensor, that is my reference start point
		while(rotate_sensor.read() != 0 );

	    SysTick_Run();
		while(1)
		{
			if( mpu->getIntStatus() )
			{
				mpu->readMotion7_blocking(accRaw, gyroRaw, &temp);
				ticks = Ticks_us();
				dT[col] += (ticks*0.000001f);
				omega_s[0][col] += gyroRaw[0];
				omega_s[1][col] += gyroRaw[1];
				omega_s[2][col] += gyroRaw[2];
				rotation_samples++;
				n_samples++;
			}

			/*because the physical interrupter is large, the photo sensor
			 * reads many more than a single interrupt, so only count an
			 * interrupt as a rotation if more than 200 (arbitrarily large enough)
			 * samples are measured by the IMU*/
			if( (rotate_sensor.read() == 0) && (rotation_samples > 200) )
			{
				rotations++;
				rotation_samples = 0;
			}

			if( rotations == 3 )
			{
				break;
			}
		}
	    omega_s[0][col] = omega_s[0][col] / n_samples;
	    omega_s[1][col] = omega_s[1][col] / n_samples;
	    omega_s[2][col] = omega_s[2][col] / n_samples;

	    wait_ms(1000);
		rotation_ctrl.pulsewidth_us(stop_pwm);

        sprintf(str,"GYRO - %d OF 3", col+1);
		myLcd->SetLine(0, str, 0);
		myLcd->SetLine(1, (char*)"-MEASUREMENT  ", 0);
		myLcd->SetLine(2, (char*)" COMPLETE!    ", 0);
		myLcd->SetLine(3, (char*)"              ", 0);
		myLcd->SetLine(4, (char*)"              ", 0);
		refresh_LCD(myLcd);

		printf("--->Gyroscope: Measurement %d of 3 COMPLETE!\r\n\r\n",col+1);

		wait_ms(1000);
	}

	rotation_ctrl.pulsewidth_us(stop_pwm);

    /* The angular velocity can be approximated by taking the total
     * distance traveled and dividing by the time
     * total distance = 3 rotations = 1080 degrees */
    omega[0][0] = ( 1080.0f / dT[0] );
    omega[1][1] = ( 1080.0f / dT[1] );
    omega[2][2] = ( 1080.0f / dT[2] );

	printf("\r\n omega_s = \r\n");
    print_matrix3x3(omega_s);

	printf("\r\n omega_0 = \r\n");
	print_matrix3x3(omega_0);

	printf("\r\n omega = \r\n");
	print_matrix3x3(omega);


    subtract_matrix3x3(omega_s, omega_0, m);
    printf("\r\n omega_s - omega_0 = \r\n");
    print_matrix3x3(m);

    inverse_matrix3x3(m);
    printf("\r\n(omega_s - omega_0)^-1 = \r\n");
    print_matrix3x3(m);

	multiply_matrix3x3(omega,m,Cg);
	printf("\r\nCg = omega*(omega_s - omega_0)^-1\r\n");
	print_matrix3x3(Cg);

	fflush(stdout);

	myLcd->SetLine(0, (char*)"GYRO GAINS:", 0);
	sprintf(str, (char*)"%1.3f %1.3f %1.3f",Cg[0][0],Cg[0][1],Cg[0][2]);
	myLcd->SetLine(1, str, 0);
	sprintf(str, (char*)"%1.3f %1.3f %1.3f",Cg[1][0],Cg[1][1],Cg[1][2]);
	myLcd->SetLine(2, str, 0);
	sprintf(str, (char*)"%1.3f %1.3f %1.3f",Cg[2][0],Cg[2][1],Cg[2][2]);
	myLcd->SetLine(3, str, 0);
	myLcd->SetLine(4, (char*)"              ", 0);
	myLcd->SetLine(5, (char*)"              ", 0);
	refresh_LCD(myLcd);

	wait_ms(1000);

	return 1;
}


/*Purpose: Validation Test(s) for gyroscope*/
static void gyro_valid(MPU6050* mpu, NokiaLcd* myLcd, DigitalIn* btnEnter, DigitalIn* btnCancel, double Cg[3][3], double gofs[3])
{
	char str[40];
	const char* dir[] = {"RIGHT","FRONT","TOP"};

	int temp_compensation = 0;

	float accRaw[3];
	float temp;
	float gyroRaw[3];
	float gyroCal[3];

	double gofs_temp_coeffs[3][3] = {0};

	float degreesRaw[3][3] = {0};
	float degreesCal[3][3] = {0};
	int ticks;
	float dT;

	int rotations, rotation_samples = 0;
	DigitalIn rotate_sensor(p23);
	rotate_sensor.mode(PullUp);

	PwmOut rotation_ctrl(p24); //S1 on FCM
	int stop_pwm = 1500;
	int ccw_pwm = 1375;
	//int cw_pwm = 1625;

	rotation_ctrl.period_us(3333); // 20ms period, 50Hz
	rotation_ctrl.pulsewidth_us(stop_pwm);

	wait_ms(1000);
	myLcd->SetLine(0, (char*)"DO YOU WANT TO", 0);
	myLcd->SetLine(1, (char*)"USE THE TEMP  ", 0);
	myLcd->SetLine(2, (char*)"COMPENSATION  ", 0);
	myLcd->SetLine(3, (char*)"OR FIXED OFFS ", 0);
	myLcd->SetLine(4, (char*)"JUST MEASURED?", 0);
	myLcd->SetLine(5, (char*)"TEMP     FIXED", 0);
	refresh_LCD(myLcd);

	while(1)
	{
		if( btnEnter->read() == 0 )
		{
			temp_compensation = 1;
			break;
		}
		else if( btnCancel->read() == 0 )
		{
			temp_compensation = 0;
			break;
		}
	}

	mpu->readMotion7_blocking(accRaw, gyroRaw, &temp);
	if( temp_compensation )
	{
		if (   Load_Double("GyroTempOfsP", gofs_temp_coeffs[0], 3, "/local/config.txt")
		    && Load_Double("GyroTempOfsR", gofs_temp_coeffs[1], 3, "/local/config.txt")
			&& Load_Double("GyroTempOfsY", gofs_temp_coeffs[2], 3, "/local/config.txt") )
		{
			for(int j=0; j<3; j++)
			{
				gofs[j] = temp*( temp*gofs_temp_coeffs[j][0]   +
									  gofs_temp_coeffs[j][1] ) +
									  gofs_temp_coeffs[j][2];
			}
		}
	}

	myLcd->SetLine(0, (char*)"GYRO OFFSETS: ", 0);
	sprintf(str,"TEMP:%+2.1fdegC",temp);
	myLcd->SetLine(1, str, 0);
	sprintf(str,"X-OFFSET:%5.4f",gofs[0]);
	myLcd->SetLine(2, str, 0);
	sprintf(str,"Y-OFFSET:%5.4f",gofs[1]);
	myLcd->SetLine(3, str, 0);
	sprintf(str,"Z-OFFSET:%5.4f",gofs[2]);
	myLcd->SetLine(4, str, 0);
	myLcd->SetLine(5, (char*)"              ", 0);
	refresh_LCD(myLcd);
	wait_ms(6000);

	for(int i = 0; i < 3; i++)
	{
		printf("\r\nGyro Validation %d of 3:\r\n"
			   "1. Place the Gyro on a level and stationary\r\n"
			   "   surface with the %s side of IMU facing UP and\r\n"
			   "   centered in the middle of the turn-table.\r\n"
			   "2. When ready, push ENTER\r\n"
			   "*******************************************************************\r\n"
			   "WARNING: Please avoid disturbing the IMU with any additional motion.\r\n"
			   "*******************************************************************\r\n"
			   "\r\nPress ENTER on FCM when ready.\r\n",i+1,dir[i]);

		//			"12345678901234"
		sprintf(str,"GYRO- %d OF 3", i+1);
		myLcd->SetLine(0, str, 0);
		sprintf(str,"PUT %s SIDE",dir[i]);
		myLcd->SetLine(1, str, 0);
		myLcd->SetLine(2, (char*)"FACE UP,CENTER", 0);
		myLcd->SetLine(3, (char*)"ON TURN TABLE ", 0);
		myLcd->SetLine(4, (char*)"2.ENTER, AVOID", 0);
		myLcd->SetLine(5, (char*)"ENTER   CANCEL", 0);
		refresh_LCD(myLcd);

		wait_ms(1000);
		while(1)
		{
			if( btnEnter->read() == 0 )
			{
				break;
			}
			else if( btnCancel->read() == 0 )
			{
				return;
			}
		}

		myLcd->SetLine(1, (char*)" MEASURING!   ", 0);
		myLcd->SetLine(2, (char*)" STAND CLEAR!!", 0);
		myLcd->SetLine(3, (char*)" WILL STOP AFT", 0);
		myLcd->SetLine(4, (char*)" 3 ROTATIONS! ", 0);
		myLcd->SetLine(5, (char*)"              ", 0);
		refresh_LCD(myLcd);

		wait_ms(1000);
		rotation_ctrl.pulsewidth_us(ccw_pwm);
		wait_ms(1000);

		rotations = 0;
		rotation_samples = 0;

		// wait until I hit the sensor, that is my reference start point
		while(rotate_sensor.read() != 0 );

	    SysTick_Run();

		while(1)
		{
			if( mpu->getIntStatus() )
			{
				mpu->readMotion7_blocking(accRaw, gyroRaw, &temp);
				ticks = Ticks_us();
				dT = (float)ticks * 0.000001f;

				if( temp_compensation )
					for(int j=0; j<3; j++)
						gofs[j] = temp*( temp*gofs_temp_coeffs[j][0]   +
											  gofs_temp_coeffs[j][1] ) +
											  gofs_temp_coeffs[j][2];

				calibrate(gyroRaw,gyroCal,Cg,gofs);
				integrate_matrix3x3(gyroRaw,degreesRaw[i],dT);
				integrate_matrix3x3(gyroCal,degreesCal[i],dT);
				rotation_samples++;
			}

			/*because the physical interrupter is large, the photo sensor
			 * reads many more than a single interrupt, so only count an
			 * interrupt as a rotation if more than 200 (arbitrarily large enough)
			 * samples are measured by the IMU*/
			if( (rotate_sensor.read() == 0) && (rotation_samples > 200) )
			{
				rotations++;
				rotation_samples = 0;
			}

			if( rotations == 3 )
			{
				break;
			}
		}

		wait_ms(1000);
		rotation_ctrl.pulsewidth_us(stop_pwm);

		sprintf(str,"GYRO - %d OF 3", i+1);
		myLcd->SetLine(0, str, 0);
		myLcd->SetLine(1, (char*)"-MEASUREMENT  ", 0);
		myLcd->SetLine(2, (char*)" COMPLETE!    ", 0);
		myLcd->SetLine(3, (char*)"              ", 0);
		myLcd->SetLine(4, (char*)"              ", 0);
		myLcd->SetLine(5, (char*)"              ", 0);
		refresh_LCD(myLcd);

		printf("--->Gyroscope: Validation %d of 3 COMPLETE!\r\n\r\n",i+1);

		wait_ms(1000);

		sprintf(str,"GYRO %s SIDE", dir[i]);
		myLcd->SetLine(0, str, 0);
		sprintf(str,"DEGS = %d", rotations*360);
		myLcd->SetLine(1, str, 0);
		myLcd->SetLine(2, (char*)" RAW  ::  CAL ", 0);
		sprintf(str,"%6.1f::%6.1f",degreesRaw[i][0],degreesCal[i][0]);
		myLcd->SetLine(3, str, 0);
		sprintf(str,"%6.1f::%6.1f",degreesRaw[i][1],degreesCal[i][1]);
		myLcd->SetLine(4, str, 0);
		sprintf(str,"%6.1f::%6.1f",degreesRaw[i][2],degreesCal[i][2]);
		myLcd->SetLine(5, str, 0);
		refresh_LCD(myLcd);

		printf("--->Gyroscope: Validation %d of 3 COMPLETE!\r\n\r\n",i+1);

		while(1)
		{
			if( btnEnter->read() == 0 || btnCancel->read() == 0)
			{
				break;
			}
		}
	}
	wait_ms(1000);

	return;
}

/*Purpose: Validation Test(s) for accelerometer*/
static void accel_valid(MPU6050* mpu, NokiaLcd* myLcd, DigitalIn* btnEnter, DigitalIn* btnCancel, double Ca[3][3], double aofs[3])
{
	char str[40];

	int ticks;
	int counter = 0;

	float gyroRaw[3];
	float temp;

	float accRaw[3];
	float accCal[3];

	float orientRaw[3];
    float orientCal[3];

	wait_ms(1000);
    myLcd->SetLine(0, (char*)"ACCEL RAW::CAL", 0);

	while(1)
	{
		mpu->readMotion7_blocking(accRaw, gyroRaw, &temp);
		ticks = Ticks_us();
		counter += ticks;
		calibrate(accRaw,accCal,Ca,aofs);
		getAccelOrient(orientRaw,accRaw);
		getAccelOrient(orientCal,accCal);

		if(counter % 1000000 <= ticks)
		{
			sprintf(str,"%6.3f::%6.3f",accRaw[0],accCal[0]);
			myLcd->SetLine(1, str, 0);
			sprintf(str,"%6.3f::%6.3f",accRaw[1],accCal[1]);
			myLcd->SetLine(2, str, 0);
			sprintf(str,"%6.3f::%6.3f",accRaw[2],accCal[2]);
			myLcd->SetLine(3, str, 0);
            sprintf(str,"%6.3f::%6.3f",orientRaw[0]*R2D,orientCal[0]*R2D);
            myLcd->SetLine(4, str, 0);
            sprintf(str,"%6.3f::%6.3f",orientRaw[1]*R2D,orientCal[1]*R2D);
            myLcd->SetLine(5, str, 0);
			refresh_LCD(myLcd);
		}

		if( btnEnter->read() == 0 )
		{
			break;
		}
		else if( btnCancel->read() == 0 )
		{
			return;
		}
	}
}

static void subtract_matrix3x3( double m1[3][3], double m2[3][3], double m[3][3])
{
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			m[i][j]=m1[i][j]-m2[i][j];
	return;
}

static void add_matrix3x3( double m1[3][3], double m2[3][3], double m[3][3])
{
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			m[i][j]=m1[i][j]+m2[i][j];
	return;
}

static void inverse_matrix3x3( double m[3][3] )
{

	double m_inv[3][3];

	double det = m[0][0] * ( m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
	             m[0][1] * ( m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
	             m[0][2] * ( m[1][0] * m[2][1] - m[1][1] * m[2][0]);

	double invdet = 1 / det;

	// calculate the inverse of matrix m
	m_inv[0][0] = ( m[1][1] * m[2][2] - m[2][1] * m[1][2] ) * invdet;
	m_inv[0][1] = ( m[0][2] * m[2][1] - m[0][1] * m[2][2] ) * invdet;
	m_inv[0][2] = ( m[0][1] * m[1][2] - m[0][2] * m[1][1] ) * invdet;
	m_inv[1][0] = ( m[1][2] * m[2][0] - m[1][0] * m[2][2] ) * invdet;
	m_inv[1][1] = ( m[0][0] * m[2][2] - m[0][2] * m[2][0] ) * invdet;
	m_inv[1][2] = ( m[1][0] * m[0][2] - m[0][0] * m[1][2] ) * invdet;
	m_inv[2][0] = ( m[1][0] * m[2][1] - m[2][0] * m[1][1] ) * invdet;
	m_inv[2][1] = ( m[2][0] * m[0][1] - m[0][0] * m[2][1] ) * invdet;
	m_inv[2][2] = ( m[0][0] * m[1][1] - m[1][0] * m[0][1] ) * invdet;

	memcpy(m, &m_inv, sizeof(m[0])*3);
	return;
}

static void multiply_matrix3x3( double m1[3][3], double m2[3][3], double m[3][3])
{
    int i, j, k;
	for(i = 0; i < 3; i++)
		for(j = 0; j < 3; j++)
			for(k = 0; k < 3; k++)
				m[i][j] +=  m1[i][k] *  m2[k][j];
}

static void print_matrix3x3( double m[3][3] )
{
	int i;
	for( i = 0; i < 3; i++)
		printf("\t%f\t%f\t%f\r\n",m[i][0],m[i][1],m[i][2]);
}

static void calibrate(float data_raw[3], float data_cali[3], double C_matrix[3][3], double ofs[3])
{
	for(int i=0; i<3; i++)
	{
		data_cali[i] = (float)C_matrix[i][0]*( data_raw[0] - (float)ofs[0] )
			         + (float)C_matrix[i][1]*( data_raw[1] - (float)ofs[1] )
			         + (float)C_matrix[i][2]*( data_raw[2] - (float)ofs[2] );
	}
}

static void integrate_matrix3x3(float data[3], float data_int[3], float dT)
{
	for(int i = 0; i < 3; i++)
		data_int[i] += data[i] * dT;
}

static void easter_egg( NokiaLcd* myLcd )
{
	myLcd->SetLine(0, (char*)"EVERYTHING IS ", 0);
	myLcd->SetLine(1, (char*)"AWESOME!      ", 0);
	myLcd->SetLine(2, (char*)"EVERYTHING IS ", 0);
	myLcd->SetLine(3, (char*)"COOL WHEN U'RE", 0);
	myLcd->SetLine(4, (char*)"PART OF A TEAM", 0);
	myLcd->SetLine(5, (char*)"        -GEOFF", 0);
	refresh_LCD(myLcd);

	wait_ms(7777);
}

void getAccelOrient(float *AccelOrient, float *AccData)
{
//    float GyroRate[3];
    float micro = 0.02f;

    float sign = AccData[Z_AXIS] < 0 ? -1 : 1;
//    float temp1, temp2;

    /* Accelerometer xyz into Pitch and Roll relative to the horizon */
    AccelOrient[ROLL]  =  ATAN2fR(-AccData[X_AXIS], sign*sqrtf(AccData[Z_AXIS] * AccData[Z_AXIS] + micro*AccData[Y_AXIS] * AccData[Y_AXIS]));
    AccelOrient[PITCH] = (ATAN2fR(AccData[Y_AXIS], sqrtf(AccData[Z_AXIS] * AccData[Z_AXIS] + AccData[X_AXIS] * AccData[X_AXIS])));
}


/*Write Calibration file depending on which calibration was performed*/
static void write_calib_file(int accel_done, int gyro_done,
		double Ca[3][3], double aofs[3],
		double Cg[3][3], double gofs[3])
{
	int i;
	FILE *fh;

	if( accel_done == 1 || gyro_done == 1 )
	{
		if( accel_done == 1 && gyro_done == 0)
		{
			fh = fopen("/local/acc_gain.txt", "w");
		}
		else if( gyro_done == 1 && accel_done == 0)
		{
			fh = fopen("/local/gyro_gain.txt", "w");
		}
		else
		{
			fh = fopen("/local/gain_calib.txt", "w");
		}

		if( accel_done == 1 )
		{
			fprintf(fh,"# Accelerometer/gyro/compass calibration parameters\r\n"
					   "# AVI-X#_###");

			fprintf(fh,"\r\nAccOrient\t\t"
					"(X_orient) \t\t(Y_orient) \t\t(Z_orient) \t"
					"(X_invert) \t(Y_invert) \t(Z_invert)");

			fprintf(fh,"\r\nAccOffsets");
			for( i = 0; i < 3; i++)
				fprintf(fh,"\t\t %f",aofs[i]);

			fprintf(fh,"\r\nAccXGains");
			for( i = 0; i < 3; i++)
				fprintf(fh,"\t\t %f",Ca[0][i]);

			fprintf(fh,"\r\nAccYGains");
			for( i = 0; i < 3; i++)
				fprintf(fh,"\t\t %f",Ca[1][i]);

			fprintf(fh,"\r\nAccZGains");
			for( i = 0; i < 3; i++)
				fprintf(fh,"\t\t %f",Ca[2][i]);
		}

		if( gyro_done == 1 )
		{
			fprintf(fh,"\r\nGyroOrient\t\t"
					"(pitch_orient) \t\t(roll_orient) \t\t(yaw_orient) \t"
					"(pitch_invert) \t(roll_invert) \t(yaw_invert)");

			fprintf(fh,"\r\nGyroXGains");
			for( i = 0; i < 3; i++)
				fprintf(fh,"\t\t %f",Cg[0][i]);

			fprintf(fh,"\r\nGyroYGains");
			for( i = 0; i < 3; i++)
				fprintf(fh,"\t\t %f",Cg[1][i]);

			fprintf(fh,"\r\nGyroZGains");
			for( i = 0; i < 3; i++)
				fprintf(fh,"\t\t %f",Cg[2][i]);


			fprintf(fh,"\r\n");
		}

		fclose(fh);
	}

	return;
}

/* copies input filename file line into str, which starts with string name */
static char Search_file(const char *name, char *str, const char* filename)
{
	char *config_mem = NULL;
	int   config_size = 0;

	if (config_mem && config_size>0)
    {
        char *pstr = str;
        int ch = 0;
        int len = strlen(name);
        int i;

        for (i=0; i<config_size; i++)
        {
            if ((pstr-str)>=MAX_LINES)
            {
                printf("Config line too long, max %d\r\n", MAX_LINES);
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
        FILE *fp = fopen(filename, "r");
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

static char Load_Double(const char *name, double *value, int N, const char* filename)
{
    char str[MAX_LINES];
    char *pstr = str;
    if (Search_file(name, str, filename))
    {
        char str1[64];
        int i;
        sscanf(pstr, "%s", str1);
        printf("%s\t", str1);
        pstr = FindNext(pstr);
        for (i=0; i<N; i++)
        {
            sscanf(pstr, "%lf", value+i);
            printf("%8.4lf\t", value[i]);
            pstr = FindNext(pstr);
        }
        printf("\r\n");
        return 1;
    }
    return 0;
}

static char *FindNext(char *pstr)
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

static void refresh_LCD( NokiaLcd* myLcd )
{
	for(int i = 0; i < 84; i++)
		myLcd->Update();

}
