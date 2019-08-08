/***** Includes *****/
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/I2C.h>

/* For usleep() */
//#include <unistd.h>
#include <stddef.h>

/* Board Header files */
#include "Board.h"

/* Application Header files */


/***** Defines *****/
#define I2C_TASK_STACK_SIZE 512
#define I2C_TASK_PRIORITY   3

#define delay_ms(i) Task_sleep( ((i) * 1000) / Clock_tickPeriod )
#define Board_I2C0      0

/* Local variables */
unsigned int i;
uint16_t temperatureCount;
static double temperature;
static uint8_t txBuffer[1];
static uint8_t rxBuffer[2];
//uint8_t registerAddressBuf[1] ;

static I2C_Handle i2c;
static I2C_Params i2cParams;
static I2C_Transaction i2cTransaction;

/* Task Variable declarations */
static Task_Params i2cTaskParams;
Task_Struct i2cTask;    /* Not static so you can see in ROV */
static uint8_t i2cTaskStack[I2C_TASK_STACK_SIZE];

// Semaphore handle
//static Semaphore_Handle sem_i2c;
extern Semaphore_Handle  semaphore0;
static Semaphore_Params semI2cParams;
static Semaphore_Struct sem_i2c;

/* Global variables initialization*/
uint16_t gTemperature;
uint16_t gHumidity;
uint16_t gC = 0xFF;

/* Function Prototypes  */
static void I2cTaskFunction();

// I2C interface functions
I2C_Handle i2cBegin(uint_least8_t slaveAddress);
int i2cWrite(I2C_Handle i2c, uint_least8_t slaveAddress, void* txbuffer, size_t writeCount);
int i2cRead(I2C_Handle i2c,  uint_least8_t slaveAddress, void * txBuffer, void *rxBuffer, size_t readCount);
int i2cWriteRead(I2C_Handle i2c, uint_least8_t slaveAddress, void * txBuffer, void *rxBuffer,size_t writeCount, size_t readCount);
double getHDC1000Humidity_ver2( I2C_Handle i2c, uint8_t* rxBuffer);


// HDC1000
double getHDC1000Temperature( I2C_Handle i2c, uint8_t* rxBuffer);
double getHDC1000Humidity( I2C_Handle i2c, uint8_t* rxBuffer);


/***** Function definitions *****/
void I2cTask_create(void)
{
    /* Create the i2c task */
      Task_Params_init(&i2cTaskParams);
      i2cTaskParams.stackSize = I2C_TASK_STACK_SIZE;
      i2cTaskParams.stack = &i2cTaskStack;
      i2cTaskParams.priority = 1;
      Task_construct(&i2cTask, I2cTaskFunction, &i2cTaskParams, NULL);
      System_printf("I2C Task is created .....\n");
}

static void I2cTaskFunction()
{

    /* Semaphore */
    Semaphore_Params_init(&semI2cParams);
    semI2cParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&sem_i2c, 1, &semI2cParams);

    // open i2c
    i2c = i2cBegin(Board_I2C0);
    if (i2c == NULL) {
        System_printf("I2C IS NOT OPENED !!!!!!!.\n");
        }
        else {
            System_printf("I2C is open.\n");
        }


        while (1) {
            System_printf("semaphore pend....\n");
            Semaphore_pend(Semaphore_handle(&sem_i2c),  BIOS_WAIT_FOREVER);
            getHDC1000Temperature(i2c,rxBuffer);
            delay_ms(500);
        }

        /* Deinitialized I2C */
        I2C_close(i2c);
}


// Get temperature
double getHDC1000Temperature( I2C_Handle i2c, uint8_t* rxBuffer )
{
    uint16_t count;
    int status = 0;
    double temperature = 0.0;
    uint8_t i2c_base_address = 0x40;
    uint8_t dataRegister = 0x00;
    uint8_t writeCount = 1;
    uint8_t readCount = 2;
    uint8_t txBuffer[1];

    // A write command must be issued before executing a read command
    txBuffer[0]= dataRegister;
    status = i2cWrite(i2c, i2c_base_address, txBuffer,writeCount);
    delay_ms(15); // wait for conversion at least 15 ms

    status = i2cRead(i2c,i2c_base_address, txBuffer, rxBuffer,readCount);
    //usleep(1000000L/1000 * 15);  // 15 ms
    //count = (rxBuffer[0] << 8) | (rxBuffer[1]);
    //temperature = ((double)count / 65536) * 165 - 40;
    /* CALLBACK IS USED !!! */

    return temperature;
}

// Get humidity
double getHDC1000Humidity( I2C_Handle i2c, uint8_t* rxBuffer)
{
    uint16_t count;
    double humidity;
    uint8_t i2c_base_address = 0x40;
    uint8_t dataRegister = 0x01;
    uint8_t writeCount = 1;
    uint8_t readCount = 2;

    // A write command must be issued before executing a read command
    txBuffer[0]= dataRegister;
    i2cWrite(i2c, i2c_base_address, txBuffer,writeCount);
    //usleep(1000000L/1000 * 10); // 10 ms
    delay_ms(15);

    i2cRead(i2c, i2c_base_address, txBuffer, rxBuffer,readCount);
    //usleep(1000000L/1000 * 10);  // 10 ms

    count = (rxBuffer[0] << 8) | (rxBuffer[1]);
    humidity = ((double)count / 65536) * 100;

    return humidity;
}

/* Write and Read at a one shot operation  */
/* still Not working ... though            */
double getHDC1000Humidity_ver2( I2C_Handle i2c, uint8_t* rxBuffer)
{
    uint16_t count;
    double humidity;
    uint8_t i2c_base_address = 0x40;
    uint8_t dataRegister = 0x01;
    uint8_t writeCount = 1;
    uint8_t readCount = 2;
    int status = 0;

    // A write command must be issued before executing a read command
    txBuffer[0]= dataRegister;
    i2cWriteRead(i2c,i2c_base_address,txBuffer,rxBuffer,writeCount,readCount);

    //usleep(1000000L/1000 * 10);  // 10 ms
    //sleep(1);

    count = (rxBuffer[0] << 8) | (rxBuffer[1]);
    humidity = ((double)count / 65536) * 100;
    return humidity;
}


/* I2C CallBack Function */
void UserCallbackFxn(I2C_Handle handle, I2C_Transaction *msg, Bool transfer) {
    const char NumOfCallBack = 2;
    static int count = NumOfCallBack; // process data every 2 callbacks
    static int logCount =0;
    uint16_t rawTemp;
    double temp;
    if (msg->arg != NULL) {
 //Semaphore_post((Semaphore_Handle)(msg->arg));
   //  gC = 0xBBBB;
 }

 if(!(--count))
 {
     rawTemp = (rxBuffer[0] << 8) | (rxBuffer[1]);
     temp = ((double)rawTemp / 65536) * 165 - 40; // temperature
     //gC = ((double)gC / 65536) * 100;
     System_printf("i2c Callback is called:%d, temp:%d degCx10 \n", ++logCount,(int)(temp*10));

     // reset count
     count =  NumOfCallBack;
 }



 Semaphore_post(Semaphore_handle(&sem_i2c));
}

I2C_Handle i2cBegin(uint_least8_t I2C_Board)
{
    I2C_Params i2cParams;
    I2C_Handle i2c;
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cParams.transferMode = I2C_MODE_CALLBACK;
    i2cParams.transferCallbackFxn = UserCallbackFxn;
    I2C_init();
    i2c = I2C_open( I2C_Board, &i2cParams);
    return i2c;
}

/* I2C WRITE to address */
int i2cWrite(I2C_Handle i2c, uint_least8_t slaveAddress, void * txBuffer, size_t writeCount)
{
    int status;
    //uint8_t * rxBuf = 0;  // NULL rxBuf pointer but you need it for i2cWrite to work
    I2C_Transaction i2cTransaction;
    i2cTransaction.slaveAddress = slaveAddress;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = writeCount;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    i2cTransaction.arg = "temp";
    status = I2C_transfer(i2c, &i2cTransaction);
   // while(!status);
    return status; // 1 if successful otherwise 0.
}

int i2cRead(I2C_Handle i2c, uint_least8_t slaveAddress, void * txBuffer, void *rxBuffer, size_t readCount)
{
    int status;
    I2C_Transaction i2cTransaction;
    i2cTransaction.slaveAddress = slaveAddress;
    i2cTransaction.writeBuf = NULL;
    i2cTransaction.writeCount = 0;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = readCount;
    status = I2C_transfer(i2c, &i2cTransaction);
    //while(!status);
    return status; // 1 if successful otherwise 0.
}

int i2cWriteRead(I2C_Handle i2c, uint_least8_t slaveAddress, void * txBuffer, void *rxBuffer,size_t writeCount, size_t readCount)
{

    int status;
    I2C_Transaction i2cTransaction;
    i2cTransaction.slaveAddress = slaveAddress;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = writeCount;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = readCount;
    status = I2C_transfer(i2c, &i2cTransaction);
    return status;
}

/*void SensorTag_blinkLed(uint8_t led, uint8_t nBlinks)
{
  uint8_t i;

  for (i=0; i<nBlinks; i++)
  {
    PIN_setOutputValue(hGpioPin, led, Board_LED_ON);
    delay_ms(BLINK_DURATION);
    PIN_setOutputValue(hGpioPin, led, Board_LED_OFF);
    delay_ms(BLINK_DURATION);
  }
} */