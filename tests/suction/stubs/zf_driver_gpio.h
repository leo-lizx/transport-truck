#ifndef SUCTION_TEST_STUB_GPIO_H
#define SUCTION_TEST_STUB_GPIO_H

#include <stdint.h>

/* Only C15 is provided so an unintended dependency on another pin fails. */
typedef enum { C15 = 32 * 2 + 15 } gpio_pin_enum;
typedef enum { GPI = 0, GPO = 1 } gpio_dir_enum;
enum { GPIO_LOW = 0, GPIO_HIGH = 1 };
#define GPI_PULL_UP (0x100U)

void gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, uint8_t value,
               uint32_t pinconf);
uint8_t gpio_get_level(gpio_pin_enum pin);

#endif
