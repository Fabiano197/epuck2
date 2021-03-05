#include <stm32f4xx.h>
#include <gpio.h>
#include <main.h>

#define TIMER_CLOCK 840000    	// TODO: configure APB1 clock
#define PRESCALER   84       // TODO: configure timer frequency
#define COUNTER_MAX 10000       // TODO: configure timer max counter

void timer6_start(void)
{
    // Enable TIM6 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Enable TIM6 interrupt vector
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    // Configure TIM6
    TIM6->PSC = PRESCALER - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
    TIM6->ARR = COUNTER_MAX - 1;	// Note: timer reload takes 1 cycle, thus -1
    TIM6->DIER |= TIM_DIER_UIE;  	// Enable update interrupt
    TIM6->CR1 |= TIM_CR1_CEN;    	// Enable timer
}

void timer7_start(void)
{
    // Enable TIM7 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    // Enable TIM7 interrupt vector
    NVIC_EnableIRQ(TIM7_IRQn);

    // Configure TIM7
    TIM7->PSC = PRESCALER - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
    TIM7->ARR = COUNTER_MAX - 1;	// Note: timer reload takes 1 cycle, thus -1
    TIM7->DIER |= TIM_DIER_UIE;  	// Enable update interrupt
    TIM7->CR1 |= TIM_CR1_CEN;    	// Enable timer
}

void timer4_start(void)
{
    // Enable TIM4 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Enable TIM4 interrupt vector
    // NVIC_EnableIRQ(TIM4_IRQn);

    // Configure TIM4
    TIM4->PSC = PRESCALER - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
    TIM4->ARR = COUNTER_MAX - 1;	// Note: timer reload takes 1 cycle, thus -1
    //TIM4->DIER |= TIM_DIER_UIE;  	// Enable update interrupt
    TIM4->CR1 |= TIM_CR1_CEN;    	// Enable timer
}

void pwm(TIM_TypeDef* timer, unsigned int channel, unsigned int duty_cycle){
	//Capture Compare Enable Register for channel #channel
	timer->CCER |= 1 << ((channel-1)*4);
	if(channel < 3){
		timer->CCMR1 |= (7 << ((channel*8)-4));
	}
	else{
		timer->CCMR2 |= (7 << (((channel - 2)*8)-4));
	}
	switch (channel) {
		case 1:
			timer->CCR1 = (timer->ARR*duty_cycle)/100;
			break;
		case 2:
			timer->CCR2 = (timer->ARR*duty_cycle)/100;
			break;
		case 3:
			timer->CCR3 = (timer->ARR*duty_cycle)/100;
			break;
		case 4:
			timer->CCR4 = (timer->ARR*duty_cycle)/100;
			break;
	}
}

// Timer 7 Interrupt Service Routine
void TIM7_IRQHandler_old(void)
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

	gpio_toggle(LED7);

    // Clear interrupt flag
    TIM7->SR &= ~TIM_SR_UIF;
    TIM7->SR;	// Read back in order to ensure the effective IF clearing
}
