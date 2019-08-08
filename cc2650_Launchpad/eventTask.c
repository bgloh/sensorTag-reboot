// event task


/*******************************************************************************
 * INCLUDES
 */
#include <string.h>
#include <stdio.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/BIOS.h>

// code I added
#include <ti/drivers/UART.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>


#include "hci.h"
#include "sensortag_io.h"



// Task configuration
#define ST_TASK_PRIORITY                      1
#define ST_TASK_STACK_SIZE                    700
#define UART_MESSAGE                          Event_Id_01

static Task_Struct myTask; // The code i added
static Char myTaskStack[ST_TASK_STACK_SIZE];

static uint16_t Myevent;
static Clock_Struct myPeriodicClock0, myPeriodicClock1;  // Clock instances for internal periodic events.
Event_Handle evt;

//UART
UART_Handle uart;

const char uartMessage[] = "event is posted !\r\n";
uint8_t txBuf[2] = {0x61, 0x62};
uint8_t rxBuf[10];


/* function prototypes */
static void readCallback(UART_Handle handle, void *rxBuf, size_t size);


static void my_clockHandler0(UArg arg)
{
    Event_post(evt,Event_Id_00 );
    Util_startClock(&myPeriodicClock0);
}

static void my_clockHandler1(UArg arg)
{
    static int i=0;
   // Event_post(evt,Event_Id_01 );
    Util_startClock(&myPeriodicClock1);
}

void myTaskFxn()
{
    #define MY_EVENT0 6
    #define MY_EVENT1 7
    const int MYCLOCK0_PERIOD = 1000;
    const int MYCLOCK1_PERIOD = 1000;

    // initialize
    UART_init();

    // open uart
    Uart_begin();

    // create Event
    evt = Event_create(NULL,NULL);

    // Create one-shot clocks for internal periodic events.
      Util_constructClock(&myPeriodicClock0,my_clockHandler0,MYCLOCK0_PERIOD,0,false, MY_EVENT0);
      Clock_Handle h_myClock0 = Clock_handle(&myPeriodicClock0);
      Clock_start(h_myClock0);

      Util_constructClock(&myPeriodicClock1,my_clockHandler1,MYCLOCK1_PERIOD,0,false, MY_EVENT1);
      Clock_Handle h_myClock1 = Clock_handle(&myPeriodicClock1);
      Clock_start(h_myClock1);


    uint16_t events;

    // start uart read with callback Fxn
    UART_read(uart,rxBuf,1);

    while(1) {
        /* Wait for ANY of the ISR events to be posted */

         events = Event_pend(evt, Event_Id_NONE,
         Event_Id_00 + UART_MESSAGE + Event_Id_02,
         BIOS_WAIT_FOREVER);

       /* The following C code blocks on an event. It wakes the task only when both events 0
       and 1 have occurred. It sets the andMask to enable both Event_Id_00 and Event_Id_06. It sets the
       orMask to Event_Id_NONE */


        if (events & Event_Id_00) {
          // SensorTagIO_blinkLed(IOID_6, 1);
            UART_write(uart, txBuf, sizeof(txBuf));


         }
        if (events & UART_MESSAGE) {
            SensorTagIO_blinkLed(IOID_6, 1);
            UART_read(uart,rxBuf,1);
        }
    }
}

void myTask_createTask(void)
{
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stack = myTaskStack;
    taskParams.stackSize = ST_TASK_STACK_SIZE;
    taskParams.priority = ST_TASK_PRIORITY;
    Task_construct(&myTask, myTaskFxn, &taskParams, NULL);
}

// Uart initialization

Void Uart_begin()
{

  //  UART_Handle uart;
    UART_Params uartParams;


    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode =  UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = readCallback;
    uartParams.baudRate = 9600;
    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
           System_printf("Error opening the UART");
           System_flush();
       }
    UART_write(uart,txBuf, sizeof(txBuf));
}

// Callback function
static void readCallback(UART_Handle handle, void *rxBuf, size_t size)
{
   uint8_t data;
   // read received data
   data = *((uint8_t*)rxBuf);
   // print to system console
   System_printf("data:%d \n", data);
   System_flush();
   // post 'UART_MESSAGE' event
   Event_post(evt,UART_MESSAGE);
}