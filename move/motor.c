#include <stdlib.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <gpio.h>
#include <motor.h>

#define TIMER_CLOCK         84000000
#define TIMER_FREQ          100000 // [Hz]
#define PRESCALER			TIMER_CLOCK/TIMER_FREQ

#define MOTOR_SPEED_LIMIT   13 // [cm/s]
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13 // [cm]

//timers to use for the motors
#define MOTOR_RIGHT_TIMER       TIM6
#define MOTOR_RIGHT_TIMER_EN    RCC_APB1ENR_TIM6EN
#define MOTOR_RIGHT_IRQHandler  TIM6_DAC_IRQHandler
#define MOTOR_RIGHT_IRQ         TIM6_DAC_IRQn

#define MOTOR_LEFT_TIMER        TIM7
#define MOTOR_LEFT_TIMER_EN     RCC_APB1ENR_TIM7EN
#define MOTOR_LEFT_IRQHandler   TIM7_IRQHandler
#define MOTOR_LEFT_IRQ          TIM7_IRQn

//control signals A,B,C,D for motor 1 (left)
#define MOTOR_LEFT_A	GPIOE, 9
#define MOTOR_LEFT_B	GPIOE, 8
#define MOTOR_LEFT_C	GPIOE, 11
#define MOTOR_LEFT_D	GPIOE, 10

//control signals A,B,C,D for motor 2 (right)
#define MOTOR_RIGHT_A	GPIOE, 13
#define MOTOR_RIGHT_B	GPIOE, 12
#define MOTOR_RIGHT_C	GPIOE, 14
#define MOTOR_RIGHT_D	GPIOE, 15


static void timer_left_init(void){
	// Enable TIM clock
	RCC->APB1ENR |= MOTOR_LEFT_TIMER_EN;

	// Enable interrupt vector
	NVIC_EnableIRQ(MOTOR_LEFT_IRQ);

	// Configure the left timer
	MOTOR_LEFT_TIMER->PSC = PRESCALER - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
	MOTOR_LEFT_TIMER->ARR = 0;					// Note: Disables the counter since the ARR is never overstepped
	MOTOR_LEFT_TIMER->DIER |= TIM_DIER_UIE;  	// Enable update interrupt
	MOTOR_LEFT_TIMER->CR1 |= TIM_CR1_CEN;    	// Enable timer
}

static void timer_right_init(void){
	// Enable TIM clock
	RCC->APB1ENR |= MOTOR_RIGHT_TIMER_EN;

	// Enable interrupt vector
	NVIC_EnableIRQ(MOTOR_RIGHT_IRQ);

	//configure the right timer
	MOTOR_RIGHT_TIMER->PSC = PRESCALER - 1;     // Note: final timer clock  = timer clock / (prescaler + 1)
	MOTOR_RIGHT_TIMER->ARR = 0;					// Note: Disables the counter since the ARR is never overstepped
	MOTOR_RIGHT_TIMER->DIER |= TIM_DIER_UIE;  	// Enable update interrupt
	MOTOR_RIGHT_TIMER->CR1 |= TIM_CR1_CEN;    	// Enable timer
}

// the state in which the motors stop
static const uint8_t step_halt[NB_OF_PHASES] = {0, 0, 0, 0};	// state {A, B, C, D}
// the 4 states according to the 4 phases of the electrical motor
// they don't allow a short cut by never having 1,1 on positions 0 and 1 or on 2 and 3
static const uint8_t step_table[NSTEP_ONE_EL_TURN][NB_OF_PHASES] = {
    {0, 1, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 0, 1},
    {1, 0, 1, 0},
};

//contain the current position of the robot since the last motor_set_position call
static float pos_right = 0;
static float pos_left = 0;

//the distace in cm the wheels make with one step
static float step = 0.013;

//contain the target position that is given by the motor_set_position function
static float target_pos_right = 0;
static float target_pos_left = 0;

//contain the phase the motor is currently in
static uint8_t phase_right = 0;
static uint8_t phase_left = 0;

//initiates all the pins and timers needed for the operation of the motors
//but doesn't start the timers. That is done in the motor_set_speed function
void motor_init(void)
{
	//configure the motor control pins as output, and set the output to 0
	gpio_config_output_opendrain(MOTOR_LEFT_A);
	gpio_clear(MOTOR_LEFT_A);
	gpio_config_output_opendrain(MOTOR_LEFT_B);
	gpio_clear(MOTOR_LEFT_B);
	gpio_config_output_opendrain(MOTOR_LEFT_C);
	gpio_clear(MOTOR_LEFT_C);
	gpio_config_output_opendrain(MOTOR_LEFT_D);
	gpio_clear(MOTOR_LEFT_D);

	gpio_config_output_opendrain(MOTOR_RIGHT_A);
	gpio_clear(MOTOR_RIGHT_A);
	gpio_config_output_opendrain(MOTOR_RIGHT_B);
	gpio_clear(MOTOR_RIGHT_B);
	gpio_config_output_opendrain(MOTOR_RIGHT_C);
	gpio_clear(MOTOR_RIGHT_C);
	gpio_config_output_opendrain(MOTOR_RIGHT_D);
	gpio_clear(MOTOR_RIGHT_D);

	//configure the timers 6 + 7
	timer_right_init();
	timer_left_init();
}

// sets the position of the IO pins according to the values in the array out
static void right_motor_update(const uint8_t *out)
{
	// the positions are defined in the step_table constant.
	// the order in which the pins are updated guarantees that there is never a short cut during a transition
	if(gpio_is_set(MOTOR_RIGHT_A)){
		if(out[0]) gpio_set  (MOTOR_RIGHT_A);
		else 	   gpio_clear(MOTOR_RIGHT_A);
		if(out[1]) gpio_set  (MOTOR_RIGHT_B);
		else 	   gpio_clear(MOTOR_RIGHT_B);
	}
	else {
		if(out[1]) gpio_set  (MOTOR_RIGHT_B);
		else 	   gpio_clear(MOTOR_RIGHT_B);
		if(out[0]) gpio_set  (MOTOR_RIGHT_A);
		else 	   gpio_clear(MOTOR_RIGHT_A);
	}
	if(gpio_is_set(MOTOR_RIGHT_C)){
		if(out[2]) gpio_set  (MOTOR_RIGHT_C);
		else 	   gpio_clear(MOTOR_RIGHT_C);
		if(out[3]) gpio_set  (MOTOR_RIGHT_D);
		else 	   gpio_clear(MOTOR_RIGHT_D);
	}
	else {
		if(out[3]) gpio_set  (MOTOR_RIGHT_D);
		else 	   gpio_clear(MOTOR_RIGHT_D);
		if(out[2]) gpio_set  (MOTOR_RIGHT_C);
		else 	   gpio_clear(MOTOR_RIGHT_C);
	}

		if(out[2]) gpio_set  (MOTOR_RIGHT_C);
		else 	   gpio_clear(MOTOR_RIGHT_C);
		if(out[3]) gpio_set  (MOTOR_RIGHT_D);
		else 	   gpio_clear(MOTOR_RIGHT_D);
}

// sets the position of the IO pins according to the values in the array out
static void left_motor_update(const uint8_t *out)
{
	// the positions are defined in the step_table constant.
	if(out[0]) gpio_set  (MOTOR_LEFT_A);
	else 	   gpio_clear(MOTOR_LEFT_A);
	if(out[1]) gpio_set  (MOTOR_LEFT_B);
	else 	   gpio_clear(MOTOR_LEFT_B);
	if(out[2]) gpio_set  (MOTOR_LEFT_C);
	else 	   gpio_clear(MOTOR_LEFT_C);
	if(out[3]) gpio_set  (MOTOR_LEFT_D);
	else 	   gpio_clear(MOTOR_LEFT_D);
}

//stops the motor by setting all the GPIOs to zero and disables the timers
void motor_stop(void)
{
	// set all the control signals to 0
	left_motor_update(step_halt);
	right_motor_update(step_halt);

	//disable the timers
	MOTOR_RIGHT_TIMER->ARR = 0;
	MOTOR_LEFT_TIMER->ARR = 0;
}

// sets the speed of the motors to the passed value, by changing the ARR register of the timers accordingly
void motor_set_speed(float speed_r, float speed_l)
{
	// disables the target positions to allow for infinite driving
	//only the motor_set_position uses the target positions
	target_pos_left = 10000;
	target_pos_right = 10000;
	//check if the given speed doesn't exceed the speed limit
	if(speed_r < MOTOR_SPEED_LIMIT && speed_l < MOTOR_SPEED_LIMIT){
		MOTOR_RIGHT_TIMER->ARR = TIMER_FREQ * WHEEL_PERIMETER / (NSTEP_ONE_TURN * speed_r);
		MOTOR_LEFT_TIMER->ARR  = TIMER_FREQ * WHEEL_PERIMETER / (NSTEP_ONE_TURN * speed_l);
	}
}

void motor_set_position(float position_r, float position_l, float speed_r, float speed_l)
{
	//lets the motors drive to the given position with the given speed and then stop

	//reset the previous position
	pos_right = 0;
	pos_left = 0;
	motor_set_speed(speed_r, speed_l);
	//since the motor_set_speed function disables the target positions they have to be defined after the function is called
	target_pos_left = position_l;
	target_pos_right = position_r;
}

void MOTOR_RIGHT_IRQHandler(void)
{
	/*
    *
    *   BEWARE !!
    *   Based on STM32F40x and STM32F41x Errata sheet - 2.1.13 Delay after an RCC peripheral clock enabling
    *
    *   As there can be a delay between the instruction of clearing of the IF (Interrupt Flag) of corresponding register (named here CR) and
    *   the effective peripheral IF clearing bit there is a risk to enter again in the interrupt if the clearing is done at the end of ISR.
    *
    *   As tested, only the workaround 3 is working well, then read back of CR must be done before leaving the ISR
    *
    */

	if(target_pos_right-step < pos_right && pos_right < target_pos_right+step){
		right_motor_update(step_halt);
	}
	else {
		phase_right += 1;
		if(phase_right >= NB_OF_PHASES) phase_right = 0;

		//the right motor needs to turn the inverse way, because the motors are not mirrored by default
		right_motor_update(step_table[NB_OF_PHASES-1-phase_right]);
		pos_right += step;
	}

	// Clear interrupt flag
	MOTOR_RIGHT_TIMER->SR &= ~TIM_SR_UIF;
	MOTOR_RIGHT_TIMER->SR;	// Read back in order to ensure the effective IF clearing
}

void MOTOR_LEFT_IRQHandler(void)
{
	/*
    *
    *   BEWARE !!
    *   Based on STM32F40x and STM32F41x Errata sheet - 2.1.13 Delay after an RCC peripheral clock enabling
    *
    *   As there can be a delay between the instruction of clearing of the IF (Interrupt Flag) of corresponding register (named here CR) and
    *   the effective peripheral IF clearing bit there is a risk to enter again in the interrupt if the clearing is done at the end of ISR.
    *
    *   As tested, only the workaround 3 is working well, then read back of CR must be done before leaving the ISR
    *
    */

	//check if the target position is reached, if yes stop
	if(target_pos_left-step < pos_left && pos_left < target_pos_left+step){
		left_motor_update(step_halt);
	}
	else {
		//increment one phase
		phase_left += 1;
		if(phase_left >= NB_OF_PHASES) phase_left = 0;
		//advance the motor one step
		left_motor_update(step_table[phase_left]);
		pos_left += step;
	}

	// Clear interrupt flag
    MOTOR_LEFT_TIMER->SR &= ~TIM_SR_UIF;
    MOTOR_LEFT_TIMER->SR;	// Read back in order to ensure the effective IF clearing
}

