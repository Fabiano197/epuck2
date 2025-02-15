#ifndef GPIO_H
#define GPIO_H

#include <stm32f407xx.h>

void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin);
void gpio_config_output_pushpull(GPIO_TypeDef *port, unsigned int pin);
void gpio_config_input_pushpull(GPIO_TypeDef *port, unsigned int pin);
int  gpio_get(GPIO_TypeDef *port, unsigned int pin);
void gpio_set(GPIO_TypeDef *port, unsigned int pin);
void gpio_clear(GPIO_TypeDef *port, unsigned int pin);
void gpio_toggle(GPIO_TypeDef *port, unsigned int pin);

#endif /* GPIO_H */
