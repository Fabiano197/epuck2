#ifndef TIMER_H
#define TIMER_H

void timer6_start(void);
void timer7_start(void);
void timer4_start(void);
void pwm(TIM_TypeDef* timer, unsigned int channel, unsigned int duty_cycle);


#endif /* TIMER_H */
