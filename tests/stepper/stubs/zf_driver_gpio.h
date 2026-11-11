#ifndef STEPPER_TEST_STUB_GPIO_H
#define STEPPER_TEST_STUB_GPIO_H

#include <stdint.h>

/* 只提供 K1–K4，应用若意外依赖其他引脚会直接编译失败。 */
typedef enum
{
    C12 = 32 * 2 + 12,
    C13 = 32 * 2 + 13,
    C14 = 32 * 2 + 14,
    C15 = 32 * 2 + 15
} gpio_pin_enum;
typedef enum { GPI = 0, GPO = 1 } gpio_dir_enum;
enum { GPIO_LOW = 0, GPIO_HIGH = 1 };
#define GPI_PULL_UP (0x100U)

void gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, uint8_t value,
               uint32_t pinconf);
uint8_t gpio_get_level(gpio_pin_enum pin);

#endif
