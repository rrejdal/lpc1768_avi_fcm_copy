#include "I2Ci.h"

#define I2C_ENABLE          6
#define I2C_START           5
#define I2C_STOP            4
#define I2C_FLAG            3
#define I2C_ASSERT_ACK      2

#define IRQ_I2C_BOTH        0
#define IRQ_I2C_READ        1
#define IRQ_I2C_WRITE       2

volatile I2Ci::I2CBuffer I2Ci::Buffer1 = {0,0};        //Sets the initial buffer empty and count on zero
volatile I2Ci::I2CBuffer I2Ci::Buffer2 = {0,0};        //Sets the initial buffer empty
volatile int I2Ci::defaultStatus=0;

I2Ci::I2Ci(PinName sda, PinName scl)
{
    //Check which connection we are using, if not correct, go to error status
    if ((sda==p9) && (scl==p10))
        I2CMODULE = LPC_I2C1;
    else if ((sda==p28) && (scl==p27))
        I2CMODULE = LPC_I2C2;
    else
        error("I2Ci pins not valid");

    //Default settings:
    frequency(I2C_FREQUENCY);

    writePinState();
}

int I2Ci::write_nb(int address, volatile char *data, int length, volatile int *status)
{
    I2CData Data;
    //Store relevant information
    address &= 0xFE;
    Data.caller   = this;
    Data.address  = address;
//    Data.repeated = data[0]>=128 ? true : false;
    Data.repeated = false;
    Data.data     = data;
    Data.length   = length;
    Data.status   = status;

    while(!addBuffer(Data, I2CMODULE));

    return 0;
}

void I2Ci::write_reg_nb(int address, volatile char data[2])
{
    write_nb(address, data, 2, NULL);
}
    
void I2Ci::write_reg_blocking(int address, char reg, char data)
{
    char d[2] = {reg, data};
    int status;
    write_nb(address, d, 2, &status);
    while (!status) wait_us(1);
}

void I2Ci::write_reg_blocking(int address, char data) // function to talk with the mux, is of length 2 bytes only
{
	volatile char d = data;
	int status;
	write_nb(address, &d, 1, &status);
	while(!status) wait_us(1);
}

void I2Ci::write_blocking(int address, char reg, char *data, int length)
{
   char d[length+1];
   d[0] = reg;
   for(int i = 0; i<length; i++) {
       d[i+1] = data[i];
   }
   int status;
   write_nb(address, d, length+1, &status);
   while (!status) wait_ms(5);
}

void I2Ci::write_blocking(int address, char* reg, int reg_len, char *data, int data_len)
{
   char d[reg_len+data_len]; // register length is two bytes
   for(int i = 0; i<reg_len; i++) {
       d[i] = reg[i];
   }
   for(int i = 0; i<data_len; i++) {
       d[i+reg_len] = data[i];
   }
   int status;
   write_nb(address, d, reg_len+data_len, &status);
   while (!status) wait_ms(5);
}

int I2Ci::read_nb(int address, volatile char *data, int length, volatile int *status)
{
    //Store relevant information
    address |= 0x01;

    //isIdle check here
    I2CData Data;

    Data.caller   = this;
    Data.address  = address;
//    Data.repeated = length>1 ? true : false;
    Data.repeated = false;
    Data.data     = data;
    Data.length   = length;
    Data.status   = status;

    while(!addBuffer(Data, I2CMODULE));

    return 0;
}

void I2Ci::read_regs_nb(int address, char reg, volatile char *data, int length, volatile int *status)
{
    data[0] = reg;
    write_nb(address, data, 1, NULL);
    read_nb(address, data, length, status);
}

int I2Ci::read_reg_blocking(int address, char reg)
{
    volatile char d = reg;
    volatile int status;

    //isIdle check here
    I2CData Data;

    write_nb(address, &d, 1, &status);
    while (!status) wait_us(1);

    //Store relevant information
    address |= 0x01;

    Data.caller  = this;
    Data.address  = address;
    Data.repeated = false;
    Data.data     = &d;
    Data.length   = 1;
    Data.status   = &status;

    while(!addBuffer(Data, I2CMODULE));
    
    I2CBuffer *Buffer;
    if (I2CMODULE == LPC_I2C1)
        Buffer = (I2CBuffer*)&Buffer1;
    else
        Buffer = (I2CBuffer*)&Buffer2;
    
    while(Buffer->queue!=0)
        wait_us(1);
    
    if (status!=0x58)         //Return zero if ended correctly, otherwise return return code.
        return 0;

    return d;
}

int I2Ci::read_reg_blocking(int address, char* reg, int reg_len)
{
    char d[reg_len]; // register length is two bytes
    volatile int status;

    for(int i = 0; i<reg_len; i++) {
        d[i] = reg[i];
    }

    //isIdle check here
    I2CData Data;

    write_nb(address, d, reg_len, &status);
    while (!status) wait_us(1);

    //Store relevant information
    address |= 0x01;

    Data.caller  = this;
    Data.address  = address;
    Data.repeated = false;
    Data.data     = d;
    Data.length   = reg_len;
    Data.status   = &status;

    while(!addBuffer(Data, I2CMODULE));

    I2CBuffer *Buffer;
    if (I2CMODULE == LPC_I2C1)
        Buffer = (I2CBuffer*)&Buffer1;
    else
        Buffer = (I2CBuffer*)&Buffer2;

    while(Buffer->queue!=0)
        wait_us(1);

    if (status!=0x58)         //Return zero if ended correctly, otherwise return return code.
        return 0;

    return d[0];
}

void I2Ci::frequency(int hz) {
    //The I2C clock by default runs on quarter of system clock, which is 96MHz
    //So to calculate high/low count times, we do 96MHz/4/2/frequency
    duty = 96000000/8/hz;
    if (duty>65535)
        duty=65535;
    if (duty<4)
        duty=4;
}

int I2Ci::getQueue( void )
{
    volatile I2CBuffer *Buffer;
    if (I2CMODULE == LPC_I2C1)
        Buffer = &Buffer1;
    else
        Buffer = &Buffer2;
    return Buffer->queue;
}

void I2Ci::read_regs_blocking(int address, char reg, volatile char *data, int length, volatile int *status)
{
    data[0] = reg;
    write_nb(address, data, 1, NULL);
    while (!status);
    read_nb(address, data, length, status);
}


//*******************************************
//***********Internal functions**************
//*******************************************


void I2Ci::writeSettings( void )
{
    I2CMODULE->I2CONSET = 1<<I2C_ENABLE;     //Enable I2C
    I2CMODULE->I2CONCLR = I2C_STOP;
    I2CMODULE->MMCTRL = 0;                   //Disable monitor mode
    I2CMODULE->I2SCLH = duty;
    I2CMODULE->I2SCLL = duty;
}

void I2Ci::writePinState( void )
{
    if (I2CMODULE == LPC_I2C1)
    {
        LPC_PINCON->PINSEL0     |= 0x0000000F;   //Sets pins as I2C
        LPC_PINCON->PINMODE0    |= 0x0000000A;   //Neither pull up nor pull down
        LPC_PINCON->PINMODE_OD0 |= 0x00000003;   //Open drain mode enabled
    }
    else if (I2CMODULE == LPC_I2C2)
    {
        LPC_PINCON->PINSEL0 |= (1<<21)|(1<<23); //Same story, different register settings
        LPC_PINCON->PINMODE0 |= (1<<21)|(1<<23);
        LPC_PINCON->PINMODE_OD0 |= (1<<10)|(1<<11);
    }
}

void I2Ci::_start(LPC_I2C_TypeDef *I2CMODULE)
{
    if (!(I2CMODULE->I2CONSET & 1<<I2C_START))  //If already sent, skip
        I2CMODULE->I2CONSET = 1<<I2C_START;     //Send start condition
}

void I2Ci::_stop(LPC_I2C_TypeDef *I2CMODULE)
{
    I2CMODULE->I2CONSET = 1<<I2C_STOP;      //Send stop condition
    I2CMODULE->I2CONCLR = 1<<I2C_FLAG;
}

//Set interrupt vector
void I2Ci::setISR(void)
{
    _setISR(I2CMODULE);
}

void I2Ci::_setISR(LPC_I2C_TypeDef *I2CMODULE)
{
    if (I2CMODULE == LPC_I2C1) {
        NVIC_SetVector(I2C1_IRQn, (uint32_t)&IRQ1Handler);
        NVIC_EnableIRQ(I2C1_IRQn);
    } else if (I2CMODULE == LPC_I2C2) {
        NVIC_SetVector(I2C2_IRQn, (uint32_t)&IRQ2Handler);
        NVIC_EnableIRQ(I2C2_IRQn);
    }
}

void I2Ci::clearISR( void )
{
    _clearISR(I2CMODULE);
}

void I2Ci::_clearISR( LPC_I2C_TypeDef *I2CMODULE )
{
    if (I2CMODULE == LPC_I2C1)
        NVIC_DisableIRQ(I2C1_IRQn);
    else if (I2CMODULE == LPC_I2C2)
        NVIC_DisableIRQ(I2C2_IRQn);
}

void I2Ci::runUserIRQ(volatile I2CData Data)
{
    if (IRQOp==IRQ_I2C_BOTH)        //Always call if both
        callback.call();
    if ((IRQOp==IRQ_I2C_READ)&&(Data.address&0x01)) //Call if read and byte was read
        callback.call();
    if ((IRQOp==IRQ_I2C_WRITE)&&(!(Data.address&0x01))) //Call if write and byte was written
        callback.call();
}

void I2Ci::IRQ1Handler( void )
{
    IRQHandler(&Buffer1, LPC_I2C1);
}

void I2Ci::IRQ2Handler( void )
{
    IRQHandler(&Buffer2, LPC_I2C2);
}

void I2Ci::IRQHandler( volatile I2CBuffer *Buffer, LPC_I2C_TypeDef *I2CMODULE)
{
    volatile I2CData *Data = &Buffer->Data[0];

    //Depending on the status register it determines next action, see datasheet
    //This is also pretty much copy pasting the datasheet
    //General options
//    printf("%x ", I2CMODULE->I2STAT);
//    printf(" ");
//    wait_us(1);
    switch (I2CMODULE->I2STAT)
    {
        case(0x08):
        case(0x10):
            //Send Address
            I2CMODULE->I2DAT = Data->address;
            I2CMODULE->I2CONCLR = 1<<I2C_FLAG | 1<<I2C_START;
            break;

            //All master TX options

            //Address + W has been sent, ACK received
            //Data has been sent, ACK received
        case(0x18):
        case(0x28):
            if (Buffer->count==Data->length)
            {
                *Data->status=I2CMODULE->I2STAT;
                if (!Data->repeated)
                    _stop(I2CMODULE);
                else
                {
                    I2CMODULE->I2CONSET = 1<<I2C_START;
                    I2CMODULE->I2CONCLR = 1<<I2C_FLAG;
                }
                bufferHandler(I2CMODULE);
            }
            else
            {
                I2CMODULE->I2DAT = Data->data[Buffer->count];
                I2CMODULE->I2CONSET = 1<<I2C_ASSERT_ACK;        //I dont see why I have to enable that bit, but datasheet says so
                I2CMODULE->I2CONCLR = 1<<I2C_FLAG;
                Buffer->count++;
            }
            break;

            //Address + W has been sent, NACK received
            //Data has been sent, NACK received
        case(0x20):
        case(0x30):
            *Data->status=I2CMODULE->I2STAT;
            _stop(I2CMODULE);
            bufferHandler(I2CMODULE);
            break;

            //Arbitration lost (situation looks pretty hopeless to me if you arrive here)
        case(0x38):
            _start(I2CMODULE);
            break;


            //All master RX options

            //Address + R has been sent, ACK received
        case(0x40):
            //If next byte is last one, NACK, otherwise ACK
            if (Data->length <= Buffer->count + 1)
                I2CMODULE->I2CONCLR = 1<<I2C_ASSERT_ACK;
            else
                I2CMODULE->I2CONSET = 1<<I2C_ASSERT_ACK;
            I2CMODULE->I2CONCLR = 1<<I2C_FLAG;
            break;

            //Address + R has been sent, NACK received
        case(0x48):
            *Data->status=I2CMODULE->I2STAT;
            _stop(I2CMODULE);
            bufferHandler(I2CMODULE);
            break;

            //Data was received, ACK returned
        case(0x50):
            //Read data
            Data->data[Buffer->count]=I2CMODULE->I2DAT;
            Buffer->count++;

            //If next byte is last one, NACK, otherwise ACK
            if (Data->length == Buffer->count + 1)
                I2CMODULE->I2CONCLR = 1<<I2C_ASSERT_ACK;
            else
                I2CMODULE->I2CONSET = 1<<I2C_ASSERT_ACK;

            I2CMODULE->I2CONCLR = 1<<I2C_FLAG;
            break;

            //Data was received, NACK returned (last byte)
        case(0x58):
            //Read data
            *Data->status=I2CMODULE->I2STAT;
            Data->data[Buffer->count]=I2CMODULE->I2DAT;
            if (!Data->repeated)
                _stop(I2CMODULE);
            else
            {
                I2CMODULE->I2CONSET = 1<<I2C_START;
                I2CMODULE->I2CONCLR = 1<<I2C_FLAG;
            }
            bufferHandler(I2CMODULE);
            break;

        default:
            *Data->status=I2CMODULE->I2STAT;
            bufferHandler(I2CMODULE);
            break;
    }
}


//**********************************************************
//*********************COMMAND BUFFER***********************
//**********************************************************

void I2Ci::bufferHandler(LPC_I2C_TypeDef *I2CMODULE)
{
    volatile I2CBuffer *Buffer;
    if (I2CMODULE == LPC_I2C1)
        Buffer = &Buffer1;
    else
        Buffer = &Buffer2;

    //Start user interrupt
//    Buffer->Data[0].caller->runUserIRQ(Buffer->Data[0]);

    removeBuffer(I2CMODULE);

    if (Buffer->queue!=0)
        startBuffer(I2CMODULE);
    else
        _clearISR(I2CMODULE);
}

//Returns true if succeeded, false if buffer is full
bool I2Ci::addBuffer(I2CData Data, LPC_I2C_TypeDef *I2CMODULE)
{
    I2CBuffer *Buffer;
    if (I2CMODULE == LPC_I2C1)
        Buffer = (I2CBuffer*)&Buffer1;
    else
        Buffer = (I2CBuffer*)&Buffer2;
    if (Buffer->queue<I2C_BUFFER)
    {
        if(Data.status == NULL)
        {
            Data.status = &defaultStatus;
            wait_us(1);     //I blame the compiler that this is needed
        }
        *Data.status = 0;
          
        Buffer->Data[Buffer->queue]=Data;
        Buffer->queue++;

        //If queue was empty, set I2C settings, start conversion
        if (Buffer->queue==1)
        {
//          printf("addBuffer %d %d %d %d %d  ", Data.address, Data.data[0], Data.length, Data.repeated, Data.status[0]);
            startBuffer(I2CMODULE);
        }

        return true;
    }
    else
        return false;
}

//Returns true if buffer still has data, false if empty
bool I2Ci::removeBuffer(LPC_I2C_TypeDef *I2CMODULE)
{
    I2CBuffer *Buffer;
    if (I2CMODULE == LPC_I2C1)
        Buffer = (I2CBuffer*)&Buffer1;
    else
        Buffer = (I2CBuffer*)&Buffer2;

    if (Buffer->queue>0)
    {
        for (int i =0; i<Buffer->queue-1; i++)
            Buffer->Data[i]=Buffer->Data[i+1];
        Buffer->queue--;
    }
    if (Buffer->queue>0)
        return true;
    else
        return false;
}

//Starts new conversion
void I2Ci::startBuffer(LPC_I2C_TypeDef *I2CMODULE)
{
    I2CBuffer *Buffer;
    if (I2CMODULE == LPC_I2C1)
        Buffer = (I2CBuffer*)&Buffer1;
    else
        Buffer = (I2CBuffer*)&Buffer2;

    //Write settings
    Buffer->Data[0].caller->writeSettings();
    Buffer->count=0;

    //Send Start
    _start(I2CMODULE);

    //Start ISR (when buffer wasnt empty this wont do anything)
    _setISR(I2CMODULE);
}
