#include "mbed.h"
#include "MPU6050.h"
#include "IMU.h"
#include "utils.h"

static const float gyro_scale_factor[4] = {250.0f/32768.0f, 500.0f/32768.0f, 1000.0f/32768.0f, 2000.0f/32768.0f};
static const float acc_scale_factor[4]  = {  2.0f/32768.0f,   4.0f/32768.0f,    8.0f/32768.0f,   16.0f/32768.0f};
// Public Methods //////////////////////////////////////////////////////////////


// Constructor
MPU6050::MPU6050(I2Ci *m_i2c, int mux_reg_setting, int m_gyro_scale, int m_acc_scale)
{
    i2c = m_i2c;

    i2c_address = (MPU6050_ADDRESS_INTERNAL << 1);
    eeprom = NULL;

    if (mux_reg_setting >= 0) {
        mux_chan = mux_reg_setting;
        mux = true;
    }
    else {
        mux_chan = 0;
        mux = false;
    }

    gyro_scale = m_gyro_scale;
    acc_scale  = m_acc_scale;

    dT = 0;
    temp_imu = 0;

    serialNum   = 0;
    calib_day   = 0;
    calib_month = 0;
    calib_year  = 0;

    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            Mp[i][j] = 0;
            Mn[i][j] = 0;

            if (i==j) {
                Ca[i][j] = 1;
                Cg[i][j] = 1;
            }
            else {
                Ca[i][j] = 0;
                Cg[i][j] = 0;
            }

            omega_0[i][j] = 0;
            omega_s[i][j] = 0;
            omega[i][j] = 0;
        }

        aofs[i] = 0;
        gofs[i] = 0;
        gyroP_temp_coeffs[i] = 0;
        gyroR_temp_coeffs[i] = 0;
        gyroY_temp_coeffs[i] = 0;
    }

    rotation_angle = 0;
    n_samples = 0;

    buffer = 0;
    orient_index = 0;
}

bool MPU6050::is_ok()
{
    if (mux) {
        i2c->write_reg_blocking(MUX_ADDRESS, mux_chan);
    }

    int r = i2c->read_reg_blocking(i2c_address, MPU6050_WHO_AM_I);

    if (r == WHO_AM_I_VALUE_6050) {
        return true;
    }

    //printf("Unrecognized Who Am I: %x\r\n", r);
    return false;
}

int MPU6050::init(IMU_TYPE type, char lp, bool use_defaults)
{
    int result = 1;

    // Assume external IMU by default.
    EEPROM_TYPE ee_type = IMU_EEPROM;
    i2c_address = (MPU6050_ADDRESS_EXTERNAL << 1);

    if (type == INTERNAL) {
        i2c_address = (MPU6050_ADDRESS_INTERNAL << 1);
        ee_type = FCM_EEPROM;
    }

    eeprom = new EEPROM(i2c, ee_type);

    if (mux) {
        i2c->write_reg_blocking(MUX_ADDRESS, mux_chan);
        wait_ms(50);
    }

    if (!use_defaults) {
        if (read_eeprom() != 0) {
            result = -1;
        }
    }

    /* clock source and disable sleep */
    i2c->write_reg_blocking(i2c_address, MPU6050_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);

    /* config - sync input and LPF */
    i2c->write_reg_blocking(i2c_address, MPU6050_CONFIG, lp);

    /* gyro range */
    i2c->write_reg_blocking(i2c_address, MPU6050_GYRO_CONFIG, gyro_scale<<3);

    /* acc range */
    i2c->write_reg_blocking(i2c_address, MPU6050_ACCEL_CONFIG, acc_scale<<3);

    /* enable secondary I2C for HMC5883L */
    i2c->write_reg_blocking(i2c_address, MPU6050_RA_INT_PIN_CFG, 1<<MPU6050_INTCFG_I2C_BYPASS_EN_BIT);

    return result;
}

void MPU6050::ForceCalData()
{
    serialNum = 4;

#ifdef IMU_X4_02
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j<3; j++) {
            Ca[i][j] = dflt_acc_calib_matrix[i][j];
            Cg[i][j] = dflt_gyr_calib_matrix[i][j];
        }

        aofs[i] = dflt_aofs[i];

        gyroP_temp_coeffs[i] = dflt_gyroP_temp_coeffs[i];
        gyroR_temp_coeffs[i] = dflt_gyroR_temp_coeffs[i];
        gyroY_temp_coeffs[i] = dflt_gyroY_temp_coeffs[i];
    }
#endif
}

int MPU6050::read_eeprom()
{
    if (mux) {
        i2c->write_reg_blocking(MUX_ADDRESS, mux_chan);
    }

    if (eeprom->read_data() != 0) {
        return -1;
    }

    //printf("eeprom read status = %d\r\n",status);
    eeprom->getDate();

    serialNum = eeprom->data.serial_num;

    calib_year  = eeprom->calib_year;
    calib_month = eeprom->calib_month;
    calib_day   = eeprom->calib_day;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j<3; j++) {
            Ca[i][j] = eeprom->data.acc_calib_matrix[i][j];
            Cg[i][j] = eeprom->data.gyro_calib_matrix[i][j];
        }

        aofs[i] = eeprom->data.acc_offsets[i];

        gyroP_temp_coeffs[i] = eeprom->data.gyroP_temp_coeffs[i];
        gyroR_temp_coeffs[i] = eeprom->data.gyroR_temp_coeffs[i];
        gyroY_temp_coeffs[i] = eeprom->data.gyroY_temp_coeffs[i];
    }
    return 0;
}

void MPU6050::write_eeprom()
{
    if (mux) {
        i2c->write_reg_blocking(MUX_ADDRESS, mux_chan);
    }

    eeprom->data.serial_num = serialNum;

    eeprom->calib_year  = calib_year;
    eeprom->calib_month = calib_month;
    eeprom->calib_day   = calib_day;

    eeprom->saveDate();

    for(int i = 0; i < 3; i++){
        for(int j = 0; j<3; j++){
            eeprom->data.acc_calib_matrix[i][j]  = Ca[i][j];
            eeprom->data.gyro_calib_matrix[i][j] = Cg[i][j];
        }
        eeprom->data.acc_offsets[i] = aofs[i];
        eeprom->data.gyroP_temp_coeffs[i] = gyroP_temp_coeffs[i];
        eeprom->data.gyroR_temp_coeffs[i] = gyroR_temp_coeffs[i];
        eeprom->data.gyroY_temp_coeffs[i] = gyroY_temp_coeffs[i];
    }

    eeprom->write_data();

    return;
}

void MPU6050::print_eeprom()
{
    if (mux) {
        i2c->write_reg_blocking(MUX_ADDRESS, mux_chan);
    }
    eeprom->print_data();
}
void MPU6050::readMotion7_start()
{
    if (mux) {
        i2c->write_reg_blocking(MUX_ADDRESS, mux_chan);
    }
    i2c->read_regs_nb(i2c_address, MPU6050_ACCEL_XOUT_H, m7buffer, sizeof(m7buffer), &status);
}

bool MPU6050::readMotion7_finish(short int acc[3], short int gyro[3], short int *temp)
{
    int counter = 2000; // 2ms
    while(1)
    {
        if (status)
            break;
        /* timeout */
        counter--;
        if (counter<=0)
            return false;
        wait_us(1);
    }
    acc[0]  = (((short int)m7buffer[0]) << 8) | m7buffer[1];
    acc[1]  = (((short int)m7buffer[2]) << 8) | m7buffer[3];
    acc[2]  = (((short int)m7buffer[4]) << 8) | m7buffer[5];
    *temp   = (((short int)m7buffer[6]) << 8) | m7buffer[7];
    gyro[0] = (((short int)m7buffer[8]) << 8) | m7buffer[9];
    gyro[1] = (((short int)m7buffer[10])<< 8) | m7buffer[11];
    gyro[2] = (((short int)m7buffer[12])<< 8) | m7buffer[13];
    return true;
}

bool MPU6050::readMotion7f_finish(float acc[3], float gyro[3], float *temp)
{
    short int t;
    short int a[3];
    short int g[3];

    if (!readMotion7_finish(a, g, &t)) {
        return false;
    }

    acc[0]  = a[0]*acc_scale_factor[acc_scale];
    acc[1]  = a[1]*acc_scale_factor[acc_scale];
    acc[2]  = a[2]*acc_scale_factor[acc_scale];

    *temp   = t*0.00294117647f + 36.53f;

    gyro[0] = g[0]*gyro_scale_factor[gyro_scale];
    gyro[1] = g[1]*gyro_scale_factor[gyro_scale];
    gyro[2] = g[2]*gyro_scale_factor[gyro_scale];

    return true;
}


void PrintIMU_Data_Helper()
{

}
/***********************************************************
 * MMRI - Extra MPU6050 IMU functions used for calibration
 * Found at the following URL:
 * https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 *
 ************************************************************/

/** Read accelerometer, gyro and temperature data in I2C blocking mode */
bool MPU6050::readMotion7_blocking(float acc[3], float gyro[3], float *temp)
{
    short int t;
    short int a[3];
    short int g[3];

    if (mux) {
        i2c->write_reg_blocking(MUX_ADDRESS, mux_chan);
    }
	i2c->read_regs_blocking(i2c_address, MPU6050_ACCEL_XOUT_H, m7buffer, sizeof(m7buffer), &status);

    if (!readMotion7_finish(a, g, &t))
    {
    	//printf("readMotion7_blocking - I AM FALSE\r\n");
        return false;
    }
    acc[0]  = a[0]*acc_scale_factor[acc_scale];
    acc[1]  = a[1]*acc_scale_factor[acc_scale];
    acc[2]  = a[2]*acc_scale_factor[acc_scale];

    *temp   = t*0.00294117647f + 36.53f;

    gyro[0] = g[0]*gyro_scale_factor[gyro_scale];
    gyro[1] = g[1]*gyro_scale_factor[gyro_scale];
    gyro[2] = g[2]*gyro_scale_factor[gyro_scale];
    return true;
}

bool MPU6050::readMotion7_blocking(){
    short int t;
    short int a[3];
    short int g[3];

    if (mux) {
        i2c->write_reg_blocking(MUX_ADDRESS, mux_chan);
    }

    // i2c->read_regs_blocking(i2c_address, MPU6050_ACCEL_XOUT_H, m7buffer, sizeof(m7buffer,  &status);
	i2c->read_regs_blocking(i2c_address, MPU6050_ACCEL_XOUT_H, m7buffer, sizeof(m7buffer), &status);

    if (!readMotion7_finish(a, g, &t))
    {
    	//printf("I AM FALSE");
    	return false;
    }

    acc_raw_data[0]  = a[0]*acc_scale_factor[acc_scale];
    acc_raw_data[1]  = a[1]*acc_scale_factor[acc_scale];
    acc_raw_data[2]  = a[2]*acc_scale_factor[acc_scale];

    temp_imu   = t*0.00294117647f + 36.53f;

    gyro_raw_data[0] = g[0]*gyro_scale_factor[gyro_scale];
    gyro_raw_data[1] = g[1]*gyro_scale_factor[gyro_scale];
    gyro_raw_data[2] = g[2]*gyro_scale_factor[gyro_scale];
    return true;
}

/** Get Data Ready interrupt enabled setting.
 * This event occurs each time a write operation to all of the sensor registers
 * has been completed. Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_DATA_RDY_BIT
 */
uint8_t MPU6050::getIntDataReadyEnabled()
{
    if (mux) {
        i2c->write_reg_blocking(MUX_ADDRESS, mux_chan);
    }

    buffer = i2c->read_reg_blocking(i2c_address, MPU6050_RA_INT_ENABLE);
    return buffer;//(buffer & MPU6050_INTERRUPT_DATA_RDY_BIT);
}

/** Set Data Ready interrupt enabled status.
 * @param enable (1) or disable(0)  New interrupt enabled status
 * @see getIntDataReadyEnabled()
 */
void MPU6050::setIntDataReady(bool setbit)
{
    if (mux) {
        i2c->write_reg_blocking(MUX_ADDRESS, mux_chan);
    }

    buffer = getIntDataReadyEnabled();
    if(setbit) buffer = buffer | MPU6050_INTERRUPT_DATA_RDY_BIT;
    else buffer = buffer & (0xFF^MPU6050_INTERRUPT_DATA_RDY_BIT);
    i2c->write_reg_blocking(i2c_address, MPU6050_RA_INT_ENABLE, buffer);
}

// INT_STATUS register

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 */
uint8_t MPU6050::getIntStatus()
{
    if (mux) {
        i2c->write_reg_blocking(MUX_ADDRESS, mux_chan);
    }

    buffer = i2c->read_reg_blocking(i2c_address, MPU6050_RA_INT_STATUS);
    return (buffer & MPU6050_INTERRUPT_DATA_RDY_BIT);
}

void MPU6050::write_accel_data()
{
	//mpu[k]->readMotion7_blocking(accRaw, gyroRaw, &temp);
//	orient_index = determine_orient();

	if(orient_index < 3)
	{
		Mp[0][orient_index] += acc_raw_data[0];
		Mp[1][orient_index] += acc_raw_data[1];
		Mp[2][orient_index] += acc_raw_data[2];
	}
	else
	{
		Mn[0][orient_index-3] += acc_raw_data[0];
		Mn[1][orient_index-3] += acc_raw_data[1];
		Mn[2][orient_index-3] += acc_raw_data[2];
	}
}

void MPU6050::write_omegaS_data()
{
    if(orient_index == 0 || orient_index == 1 || orient_index == 2){

        for(int i = 0; i < 3; i++){
            omega_s[i][orient_index] += ABS(gyro_raw_data[i]);
        }

        rotation_angle = 1080.0f;
    }

    else
    if(orient_index == 3 || orient_index == 4 || orient_index == 5){ // gb

        for(int i = 0; i < 3; i++){
            omega_s[i][orient_index-3] += ABS(gyro_raw_data[i]);
        }

//        rotation_angle = -1080.0f;
        rotation_angle = 1080.0f;
    }
}

void MPU6050::average_omegaS() {

    if(orient_index == 0 || orient_index == 1 || orient_index == 2) {
        for(int i = 0; i < 3; i++) {
            omega_s[i][orient_index] = omega_s[i][orient_index] / n_samples;
        }
    }

    else
    if(orient_index == 3 || orient_index == 4 || orient_index == 5) { // gb
        for(int i = 0; i < 3; i++){
            omega_s[i][orient_index-3] = omega_s[i][orient_index-3] / n_samples;
        }
    }
}

void MPU6050::calculate_omega0()
{

    int i = 0;

    while(i < n_samples){
        if( getIntStatus() ){
            readMotion7_blocking();

            for(int j = 0; j < 3; j++){
                gofs[j] += ABS(gyro_raw_data[j]);
            }

            i++;
        }
    }

    for(i = 0; i < 3; i++){
        gofs[i] = gofs[i] / n_samples;
        for(int j = 0; j < 3; j++){
            omega_0[i][j] = gofs[i];
        }
    }
}

void MPU6050::calculate_omega(float *time_diff)
{
    omega[0][0] = ( rotation_angle / time_diff[0] );
    omega[1][1] = ( rotation_angle / time_diff[1] );
    omega[2][2] = ( rotation_angle / time_diff[2] );
}

void MPU6050::get_gyro_calib_data()
{
    double m[3][3] = {0};

    printf("\r\n\r\n");

    printf("\tOMEGA = \r\n");
    print_matrix3x3(omega);
    printf("\tOMEGA_0 = \r\n");
    print_matrix3x3(omega_0);
    printf("\tOMEGA_S = \r\n");
    print_matrix3x3(omega_s);

    subtract_matrix3x3(omega_s, omega_0, m);
    printf("\tSubtract: OMEGA_S - OMEGA_0\r\n");
    print_matrix3x3(m);

    inverse_matrix3x3(m);
    printf("\tInverse Matrix: (OMEGA_S - OMEGA_0)^-1\r\n");
    print_matrix3x3(m);

    multiply_matrix3x3(omega,m,Cg);
    printf("\tCg = OMEGA*(OMEGA_S - OMEGA_0)^-1: \r\n");
    print_matrix3x3(Cg);

    printf("\tGyro offsets = [");
    for(int i = 0; i < 3; i++) {
        printf(" %f ",gofs[i]);
    }
    printf("]\r\n\r\n\r\n");

}

void MPU6050::get_accel_calib_data(int avg_num_samples)
{
	double m[3][3] = {0};

	for(int col = 0; col < 3; col++){
		for(int row = 0; row < 3; row++){
			Mp[row][col] = Mp[row][col] / avg_num_samples;
			Mn[row][col] = Mn[row][col] / avg_num_samples;
		}
	}
	//m = Asp - Asn
	subtract_matrix3x3(Mp, Mn, m);
	//m = (Asp - Asn)^(-1)
	inverse_matrix3x3(m);

	// Get calibration matrix: C = 2(Asp - Asn)^(-1)
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			Ca[i][j] = 2*m[i][j];
		}
	}

	// get the offset vector aOffset for each of the three axes
	add_matrix3x3(Mp, Mn, m);
	for(int i = 0; i < 3; i++){
		aofs[i] = ( m[i][0] + m[i][1] + m[i][2] ) / 6;
	}
}

/*Determines which side of the IMU is facing down towards earth
 * This function just rounds the accelerometer data to zero's and one's*/
void MPU6050::determine_orient()
{
	const int num_avgs = 50;
	float sums_xyz[3] = {0,0,0};

	for(int i = 0; i < num_avgs; i++){
		readMotion7_blocking();
		for(int j = 0; j<3; j++){
			sums_xyz[j] += acc_raw_data[j];
		}
	}

	for(int i = 0; i<3; i++){
	    round_orient[i] = round(sums_xyz[i]/num_avgs);
	}


    //printf("ROUNDED ORIENT: ");
	//for(int i = 0; i<3; i++){
	//	printf(" %d", round_orient[i]);
	//}
	//printf("\r\n");

	if( (round_orient[0] == 1) &&
		(round_orient[1] == 0) &&
		(round_orient[2] == 0) ) {
		orient_index = 0;

	}
	else
	if( (round_orient[0] == 0) &&
		(round_orient[1] == 1) &&
		(round_orient[2] == 0) ) {
		orient_index = 1;
	}

	else
	if( (round_orient[0] == 0) &&
		(round_orient[1] == 0) &&
		(round_orient[2] == 1) ) {
		orient_index = 2;
	}

	else
	if( (round_orient[0] == -1) &&
		(round_orient[1] == 0) &&
		(round_orient[2] == 0) ) {
		orient_index = 3;
	}

	else
	if( (round_orient[0] == 0) &&
		(round_orient[1] == -1) &&
		(round_orient[2] == 0) ) {
		orient_index = 4;
	}

	else
	if( (round_orient[0] == 0) &&
		(round_orient[1] == 0) &&
		(round_orient[2] == -1) ) {
		orient_index = 5;
	}

	else {
		//printf("ERROR: UNKNOWN ROUND_ORIENT\r\n");
	}

	return;
}

void MPU6050::calibrate_accel(float data_raw[3], float data_cali[3])
{
    for(int i=0; i<3; i++){
        data_cali[i] = Ca[i][0]*( data_raw[0] - aofs[0] )
                     + Ca[i][1]*( data_raw[1] - aofs[1] )
                     + Ca[i][2]*( data_raw[2] - aofs[2] );
    }
}

void MPU6050::calibrate_gyro(float data_raw[3], float data_cali[3])
{
    for(int i=0; i<3; i++){
        data_cali[i] = (float)Cg[i][0]*( data_raw[0] - (float)gofs[0] )
                     + (float)Cg[i][1]*( data_raw[1] - (float)gofs[1] )
                     + (float)Cg[i][2]*( data_raw[2] - (float)gofs[2] );
    }
}

bool MPU6050::accel_error(float acc_raw_avg[3])
{
    float acc_cal_avg[3];
    float net_raw_error = 0;
    float net_calib_error = 0;

    calibrate_accel(acc_raw_avg, acc_cal_avg);

    for(int i=0; i<3; i++){
        net_raw_error += ABS(acc_raw_avg[i] - round_orient[i]) / 3;
    }

    for(int i=0; i<3; i++){
        net_calib_error += ABS(acc_cal_avg[i] - round_orient[i]) / 3;
    }

    if(net_calib_error <= net_raw_error){
        return true;
    }
    else if(net_calib_error > net_raw_error){
        return false;
    }
    else {
        return false;
    }
}
//void MPU6050::print_omegaS()
//{
//    print_matrix3x3(omega_s);
//}

