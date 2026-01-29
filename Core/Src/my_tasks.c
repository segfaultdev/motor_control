#include "my_tasks.h"
#include "btn.h"
#include "main.h"
#include "ui.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "printk.h"

extern UART_HandleTypeDef huart2;

/* UART RX buffer */
#define RX_BUFFER_SIZE 128
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* Helper: Check if character is available */
static uint8_t uart_available(void)
{
    return (rx_head != rx_tail);
}

/* Helper: Read one character */
static uint8_t uart_read(void)
{
    uint8_t c = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    return c;
}

/* IRQ: Called when UART receives data */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* Move to next buffer position */
        uint16_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
        
        /* Check for buffer overflow */
        if (next_head != rx_tail)
        {
            rx_head = next_head;
        }
        
        /* Continue receiving into the next position */
        HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_head], 1);
    }
}

void UITask(void *arg)
{
    UI_Init();

    for (;;)
    {
        Button_t btn = xGetBtnPress();

        if (btn == BTN_UP)
        {
            UI_NavigateUp();
        }
        else if (btn == BTN_DOWN)
        {
            UI_NavigateDown();
        }
        else if (btn == BTN_UP_DBL)
        {
            UI_TogglePage();
        }
        else if (btn == BTN_OK)
        {
            /* Send command based on current page */
            char tx[12];

            if (UI_GetPage() == PAGE_PRODUCT)
            {
                /* Send product selection only */
                uint8_t p = UI_GetProduct();
                sprintf(tx, "P%d\r\n", p);
            }
            else
            {
                /* Send model selection only */
                uint8_t m = UI_GetModel();
                sprintf(tx, "M%d\r\n", m);
            }

            /* Temporarily disable RX interrupt to avoid conflicts */
            __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
            HAL_UART_Transmit(&huart2, (uint8_t *)tx, strlen(tx), 100);
            __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

            UI_SetResponse("COMMAND SENT");
            UI_IncrementCount();
        }

        UI_Update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void MotorControlTask(void *arguments)
{
}

void xLDRReadTask(void *arguments)
{
    /* Start UART interrupt receive */
    HAL_UART_Receive_IT(&huart2, &rx_buffer[0], 1);
    
    static char line_buffer[64];
    static uint8_t line_idx = 0;
    
    printk("UART RX Task started\n");
    
    while (1)
    {
        if (uart_available())
        {
            char c = uart_read();
            
            if (c == '\n' || c == '\r')
            {
                if (line_idx > 0)
                {
                    line_buffer[line_idx] = '\0';
                    
                    printk("RX: %s\n", line_buffer);
                    
                    /* Parse incoming messages */
                    if (strncmp(line_buffer, "R=", 2) == 0)
                    {
                        /* Result message: R=GOOD_BOTTLE */
                        UI_SetResponse(&line_buffer[2]);
                        printk("Set response: %s\n", &line_buffer[2]);
                    }
                    else if (strncmp(line_buffer, "C=", 2) == 0)
                    {
                        /* Confidence message: C=95 */
                        int conf = atoi(&line_buffer[2]);
                        UI_SetConfidence((float)conf);
                        printk("Set confidence: %d\n", conf);
                    }
                    
                    line_idx = 0;
                }
            }
            else if (line_idx < sizeof(line_buffer) - 1)
            {
                line_buffer[line_idx++] = c;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}