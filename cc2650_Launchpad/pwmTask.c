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
#include <ti/drivers/PWM.h>

/* For usleep() */
//#include <unistd.h>
//#include <stddef.h>
#include "SensorUtil.h"

/* Board Header files */
#include "Board.h"

/* Application Header files */

//////


/***** Defines *****/
#define PWM_TASK_STACK_SIZE 512
#define PWM_TASK_PRIORITY   3

/***** Task Variable declarations *****/
static Task_Params pwmTaskParams;
static Task_Struct pwmTask;    /* Not static so you can see in ROV */
static uint8_t pwmTaskStack[PWM_TASK_STACK_SIZE];

/* pwm handle and params declarations */
static PWM_Handle pwm1 = NULL;
static PWM_Params params;

/* Period and duty in microseconds */
static uint16_t   pwmPeriod    = 3000;
static uint16_t   duty         = 0;
static uint16_t   dutyInc      = 100;

/* Sleep time in miliseconds */
static uint32_t   timeSleep = 50;

/***** Prototypes *****/

static void pwmTaskFunction(UArg arg0, UArg arg1);


/***** Function definitions *****/
void PwmTask_create(void)
{

          /* Initialize pwm  */
          PWM_Params_init(&params);
          params.dutyUnits = PWM_DUTY_US;
          params.dutyValue = 0;
          params.periodUnits = PWM_PERIOD_US;
          params.periodValue = pwmPeriod;

          /* Create the pwm task */
          Task_Params_init(&pwmTaskParams);
          pwmTaskParams.stackSize = PWM_TASK_STACK_SIZE;
          pwmTaskParams.priority = PWM_TASK_PRIORITY;
          pwmTaskParams.stack = &pwmTaskStack;
          Task_construct(&pwmTask, pwmTaskFunction, &pwmTaskParams, NULL);
}

static void pwmTaskFunction(UArg arg0, UArg arg1)
{

    PWM_init();

    pwm1 = PWM_open(Board_PWM1, &params);

    if (pwm1 == NULL) {
        /* Board_PWM0 did not open */
        //while (1);
    }

    /* start pwm */
    PWM_start(pwm1);

    /* Loop forever incrementing the PWM duty */
    while (1) {
        PWM_setDuty(pwm1, duty);
        duty = (duty + dutyInc);
        if (duty == pwmPeriod || (!duty)) {
            dutyInc = - dutyInc;
        }
       // usleep(timeSleep);
        DELAY_MS(timeSleep);
    }
}
