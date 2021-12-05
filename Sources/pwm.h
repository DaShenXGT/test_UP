#ifndef _STEPPER_DRIVER_H_
#define _STEPPER_DRIVER_H_

//#include "MC9S08LG32.h"
//#include "stepper_int.h"

//macros defining the mode in which the stepper operates
#define MODE_2TPM_2GPIO 1

//4 Micro steps in each partial step and 3 partial steps in each full step
#define 	STEP0				4
#define 	STEP1				8
#define 	STEP2				12
#define 	STEP3				16
#define 	STEP4				20
#define 	STEP5				24

//macros to tell the direction of movement of motor
#define   STEPPER_MOVE_CLKWISE  0
#define   STEPPER_MOVE_ANTICLKWISE  1

//macros for controlling the current driver IC chip enable
#define CURRENT_DRIVER_IC_ON  0
#define CURRENT_DRIVER_IC_OFF  1

/****************************************************************************
 * Function Name: InitStepper
 *
 * Agruments:
 * NONE
 *
 * Return Type: VOID
 *
 * Description:
 *    It initializes the working mode of Stepper as PIN1-TPM1CH0, PIN2-PTI0
 * , PIN3-PTI1, PIN4-TPM1CH1. It initializes all the required TPM and GPIO
 *  registers, but does not enables the TPM clock.
 *
 * 
 ****************************************************************************/
void InitStepper(void);

/****************************************************************************
 * Function Name: MotorCurrentDriverCE_Toggle
 *
 * Agruments:
 * 1.currentdriver: It holds the enable/disable value for CE of the driver IC
 *
 * Return Type: VOID
 *
 * Description:
 *    It toggles the ChipSelect signal(Active low) for the current driver IC
 * , which is required to operate the stepper motor.
 *
 * 
 ****************************************************************************/
void MotorCurrentDriverCE_Toggle(unsigned char currentdriver);

/****************************************************************************
 * Function Name: DriveStepper_to_zero
 *
 * Agruments:
 * NONE
 *
 * Return Type: VOID
 *
 * Description:
 *    It follows a specified process to get stepper to initial ZERO position
 * This function might be different for different stepper motors. It
 * basically follows a predefined movement to avoid jitter and noisy movement
 * to initial ZERO position.
 *
 * 
 ****************************************************************************/
void DriveStepper_to_zero(void);

/****************************************************************************
 * Function Name: DriveStepper_to_Max
 *
 * Agruments:
 * NONE
 *
 * Return Type: VOID
 *
 * Description:
 *    It follows a specified process to get stepper to MAX position
 * This function might be different for different stepper motors. It
 * basically follows a predefined movement to avoid jitter and noisy movement
 * to MAX position.
 *
 * 
 ****************************************************************************/
void DriveStepper_to_Max(void);

/****************************************************************************
 * Function Name: move_motor_microstep
 *
 * Agruments:
 * 1.num_of_steps: It holds the number of steps to be moved in stepper motor
 * 2.direction: It holds the direction in which the specified steps are moved
 *
 * Return Type: VOID
 *
 * Description:
 *    It is the function providing movement of stepper motor in microsteps
 * that is 1/12 degree in each micro step
 *
 * 
 ****************************************************************************/
void move_motor_microstep(unsigned int num_of_steps, unsigned char direction);

/****************************************************************************
 * Function Name: move_motor_partialstep
 *
 * Agruments:
 * 1.num_of_steps: It holds the number of steps to be moved in stepper motor
 * 2.direction: It holds the direction in which the specified steps are moved
 *
 * Return Type: VOID
 *
 * Description:
 *    It is the function providing movement of stepper motor in partialsteps
 * that is 1/3 degree in each partial step
 *
 * 
 ****************************************************************************/
void move_motor_partialstep(unsigned int num_of_steps, unsigned char direction);

/****************************************************************************
 * Function Name: move_motor_llstep
 *
 * Agruments:
 * 1.num_of_steps: It holds the number of steps to be moved in stepper motor
 * 2.direction: It holds the direction in which the specified steps are moved
 *
 * Return Type: VOID
 *
 * Description:
 *    It is the function providing movement of stepper motor in fullsteps
 * that is 1 degree in each full step
 *
 * 
 ****************************************************************************/
void move_motor_fullstep(unsigned int num_of_steps, unsigned char direction);

#endif /* #ifndef _STEPPER_DRIVER_H_ */