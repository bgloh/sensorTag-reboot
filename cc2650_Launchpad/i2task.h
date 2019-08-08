#ifndef I2CTASK_H_
#define I2CTASK_H_

/* Global variables*/
extern uint16_t gC;
extern uint16_t gHumidity;

/* Initializes the PwmLed Task and creates all TI-RTOS objects */
extern void I2cTask_create(void);

#endif /* I2CTASK_H_ */