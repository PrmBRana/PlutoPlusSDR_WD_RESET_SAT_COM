#include "stm32wlxx.h"
#include "cmsis_gcc.h"

extern void cpu2_uart_panic(const char *tag);

void Error_Handler(void)
{
    __disable_irq();
    cpu2_uart_panic("ERROR_HANDLER");

    while (1)
    {
        __NOP();
    }
}