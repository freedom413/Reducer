#include <stdarg.h>
#include <string.h>
#include "stdio.h"
#include "usart.h"
#include "dbg.h"

/*  简易串口printf函数（支持%d, %u, %x, %s, %c）相比c库printf函数，节省了20kb 的空间 */
void dbg_printf(const char *fmt, ...)
{
    char buffer[256];
    va_list args;
    
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    
    if (len > 0) {
        HAL_UART_Transmit(DBG_UART, (uint8_t *)buffer, len, HAL_MAX_DELAY);
    }
}
