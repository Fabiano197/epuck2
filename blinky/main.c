#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <timer.h>
#include <main.h>

// Init function required by __libc_init_array
void _init(void) {}

int main(void)
{
    SystemClock_Config();

    // Enable GPIOD peripheral clock
    RCC->AHB1ENR    |= (RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);

    /*Task 1
    // LED used init
    gpio_config_output_opendrain(LED7);
    gpio_clear(LED7);

    while (1) {
        for(int i = 0; i < 4000000; i++){
        	;
        }
        gpio_toggle(LED7);
    }*/


    /* Task 2
    gpio_config_output_pushpull(FRONT_LED);
    gpio_clear(FRONT_LED);


	while (1) {
	for(int i = 0; i < 4000000; i++){
			;
		}
		gpio_toggle(FRONT_LED);
	}*/


    /* Task 3
	gpio_config_output_pushpull(BODY_LED);
	gpio_clear(BODY_LED);


	while (1) {
	for(int i = 0; i < 4000000; i++){
			;
		}
		gpio_toggle(BODY_LED);
	}*/

    /* Task 4
    gpio_config_input_pushpull(SELECTOR1);
    gpio_config_input_pushpull(SELECTOR2);
    gpio_config_input_pushpull(SELECTOR3);
    gpio_config_output_opendrain(LED1);
    gpio_config_output_opendrain(LED3);
    gpio_config_output_opendrain(LED5);
    gpio_config_output_opendrain(LED7);
    int count = 0;

    while (1) {
    	for(int i = 0; i < 4000000; i++){
    		;
    	}
    	count %=8;
    	if(gpio_get(SELECTOR3)){
    		switch (count) {
				case 0: gpio_clear(LED1); break;
				case 1: gpio_clear(LED3); break;
				case 2: gpio_clear(LED5); break;
				case 3: gpio_clear(LED7); break;
				case 4: gpio_set(LED1); break;
				case 5: gpio_set(LED3); break;
				case 6: gpio_set(LED5); break;
				case 7: gpio_set(LED7); break;
			}
    	}
    	else{
    		switch (count) {
				case 0: gpio_clear(LED1); break;
				case 2: gpio_clear(LED3); break;
				case 4: gpio_clear(LED5); break;
				case 6: gpio_clear(LED7); break;
				case 1: gpio_set(LED1); break;
				case 3: gpio_set(LED3); break;
				case 5: gpio_set(LED5); break;
				case 7: gpio_set(LED7); break;
			}
    	}

    	count++;
    }
    */

    // Task 5 and 6
    timer7_start();
    gpio_config_output_opendrain(LED7);
    gpio_clear(LED7);
    while (1) {
    	;
    }

}
