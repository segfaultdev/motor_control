#include "printk.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

extern UART_HandleTypeDef huart2;

static xSemaphoreHandle xPrintkMutex = NULL;

void printk_init()
{
    if (xPrintkMutex == NULL)
    {
        xPrintkMutex = xSemaphoreCreateMutex();
    }
}

void printk(const char *fmt, ...)
{
    va_list args;
    char buffer[256];
    int len;

    xSemaphoreTake(xPrintkMutex, portMAX_DELAY);

    va_start(args, fmt);
    len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len > 0)
    {
        buffer[len] = '\0';
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    }

    xSemaphoreGive(xPrintkMutex);
}