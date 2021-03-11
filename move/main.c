#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>
#include <timer.h>
#include <motor.h>
#include <selector.h>

#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

// Init function required by __libc_init_array
void _init(void) {}

// Simple delay function
void delay_seconds(unsigned int n)
{
	n *= 4000000;
    while (n--) {
        __asm__ volatile ("nop");
    }
}


int main(void)
{
    SystemClock_Config();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    gpio_config_output_opendrain(FRONT_LED);
    gpio_clear(FRONT_LED);

    //motor_init();
    //motor_set_speed(13, 13);
    //delay_seconds(10);
    //motor_set_position(10, 10, 5, 5);

    //motor_set_position(4, 4, 3.0, 3.0);
    //delay_seconds(5);
    //motor_set_position(8, 8, 3, 3);

    //gpio_config_output_af_pushpull(FRONT_LED);
    //gpio_config_select_af(FRONT_LED, 2);
    //init_selector();
    //timer4_start();
    //pwm(TIM4, 3, 20);

    while (1) {
    	;
    }
}

