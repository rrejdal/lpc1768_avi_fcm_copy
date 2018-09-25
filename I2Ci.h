#include "mbed.h"

#ifndef _I2CI_H_
#define _I2CI_H_

#define I2C_BUFFER          10

#define I2C_FREQUENCY       400000


/** Library that allows interrupt driven communication with I2C devices
  *
  * Example:
  * @code
  * #include "mbed.h"
  * #include "MODI2C.h"
  * #include "MPU6050.h"
  * 
  * Serial pc(USBTX, USBRX); // tx, rx
  * MODI2C mod(p9, p10);
  * 
  * int main() {
  *     char registerAdd = MPU6050_WHO_AM_I_REG;
  *     char registerResult;
  *     int status;
  * 
  *     while (1) {
  *         mod.write(MPU6050_ADDRESS*2, &registerAdd, 1, true);
  *         mod.read_nb(MPU6050_ADDRESS*2, &registerResult, 1, &status);
  * 
  *         while (!status) wait_us(1);
  *         pc.printf("Register holds 0x%02X\n\r", registerResult);
  *         wait(2);
  *     }
  * }
  * @endcode
  */
class I2Ci
{
public:
    I2Ci(PinName sda, PinName scl);

    /**
    * Write to a register in an I2C device, blocking version
    *
    * @param address - I2C address of the slave (7 bit address << 1).
    * @param reg     - register number.
    * @param data    - data to write
    */
    void write_reg_blocking(int address, char reg, char data);
    
    /**
    * Write to register in an I2C device through I2C multiplexer, blocking version
	*
	* @param address - I2C address of the slave (7 bit address << 1).
	* @param reg     - register number
	*/

    void write_reg_blocking(int address, char data);

    /**
    * Write to multiple registers in an I2C device in blocking mode,
    * for IMU EEPROM: 24LC01B
    * only 1 byte needed to address EEPROM memory since the size is
    * only 1kbit = 128 bytes, address 0x00 to 0x7F
    *
    * @param address - I2C address of the slave (7 bit address << 1).
    * @param reg     - register number (0x00 to 0x7F)
    * @param *data   - pointer to first byte in data to be written
    * @param length  - number of bytes in "data"
    */
    void write_blocking(int address, char reg, char *data, int length);

    /**
    * Write to multiple registers in an I2C device in blocking mode,
    * for FCM EEPROM: 24AA256
    * 2 bytes are needed for the register since
    * this eeprom is so big 256kbit = 32k bytes, 0x0000 to 0x7D00
    * addressing register: first byte is MSB, second byte is LSB
    *
    * @param address - I2C address of the slave (7 bit address << 1).
    * @param *reg     - register number
    * @param reg_len  - number of bytes in "reg"
    * @param *data    - pointer to first byte in data to be written
    * @param data_len - number of bytes in "data"
    */
    void write_blocking(int address, char* reg, int reg_len, char *data, int data_len);

    /**
    * Write to a register in an I2C device, non-blocking versions. Data has to exist until the write is performed !!!!!!
    *
    * @param address - I2C address of the slave (7 bit address << 1).
    * @param data    - pointer to byte array that holds the data, first value is the register to write to, second is the value
    * @length        - number of bytes to write
    */
    void write_reg_nb(int address, volatile char data[2]);

    /**
    * Read single register - blocking
    *
    * @param address - I2C address of the slave (7 bit address << 1).
    * @param reg     - register number to read.
    * @param return  - returns a numgative number on an error, the value read otherwise
    */
    int read_reg_blocking(int address, char reg);

    /**
    * Read single register that has 2 byte register address - blocking
    * addressing register: first byte is MSB, second byte is LSB
    * for FCM EEPROM: 24AA256
    * 2 bytes are needed for the register address since
    * this eeprom is so big 256kbit = 32k bytes, 0x0000 to 0x7D00
    * addressing register: first byte is MSB, second byte is LSB
    *
    * @param address - I2C address of the slave (7 bit address << 1).
    * @param reg     - register number to read, reg number can span 2 bytes
    * @param reg_len - register number to read, length of reg number
    * @param return  - returns a numgative number on an error, the value read otherwise
    */
    int read_reg_blocking(int address, char* reg, int reg_len);

    /**
    * Read multiple registers - none-blocking
    *
    * @param address - I2C address of the slave (7 bit address << 1).
    * @param reg     - register number to read.
    * @param data    - pointer to a buffer receiving the data
    * @param length  - a number of bytes to read
    * @param status  - pointer to a completion flag, signaled on completion
    */

    void read_regs_nb(int address, char reg, volatile char *data, int length, volatile int *status);
    
    /**
    * Sets the I2C bus frequency
    *
    * @param hz - the bus frequency in herz
    */
    void frequency(int hz);

    /**
    * Returns the current number of commands in the queue (including one currently being processed)
    *
    * Note that this is the number of commands, not the number of bytes
    *
    * @param return - number of commands in queue
    */
    int getQueue( void );

    /* mmri: Read multiple registers - blocking
    *
    * @param address - I2C address of the slave (7 bit address << 1).
    * @param reg     - register number to read.
    * @param data    - pointer to a buffer receiving the data
    * @param length  - a number of bytes to read
    * @param status  - pointer to a completion flag, signaled on completion
    */
    void read_regs_blocking(int address, char reg, volatile char *data, int length, volatile int *status);

private:

    struct I2CData {
        I2Ci *caller;
        char address;
        volatile char *data;
        int  length;
        bool repeated;
        volatile int *status;
    };
    
    struct I2CBuffer {
        int queue;
        int count;
        I2CData Data[I2C_BUFFER];
        };
        
    //Settings:
    int duty;
    
    FunctionPointer callback;
    int IRQOp;   
    void runUserIRQ( volatile I2CData Data );

    //Remove later:
    LPC_I2C_TypeDef *I2CMODULE;

    //Whole bunch of static stuff, pretty much everything that is ever called from ISR
    volatile static I2CBuffer Buffer1;
    volatile static I2CBuffer Buffer2;
    
    static void IRQHandler(volatile I2CBuffer *Buffer, LPC_I2C_TypeDef *I2CMODULE);
    static void IRQ1Handler(void);
    static void IRQ2Handler(void);
    
    static void bufferHandler(LPC_I2C_TypeDef *I2CMODULE);
    static bool addBuffer(I2CData Data, LPC_I2C_TypeDef *I2CMODULE);
    static bool removeBuffer(LPC_I2C_TypeDef *I2CMODULE);
    static void startBuffer(LPC_I2C_TypeDef *I2CMODULE);

    volatile static int defaultStatus;
    
    static void _start(LPC_I2C_TypeDef *I2CMODULE);
    static void _stop(LPC_I2C_TypeDef *I2CMODULE);
    static void _clearISR( LPC_I2C_TypeDef *I2CMODULE );
    static void _setISR( LPC_I2C_TypeDef *I2CMODULE );

    int write_nb(int address, volatile char *data, int length, volatile int *status);
    int read_nb(int address, volatile char *data, int length, volatile int *status);
    
    void writeSettings( void );
    void writePinState( void );
    void setISR( void );
    void clearISR( void );
};


#endif
