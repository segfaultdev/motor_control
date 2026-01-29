#include "btn.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBOUNCE_MS 30
#define DOUBLE_PRESS_MS 350

typedef struct
{
    uint8_t last_state;
    uint32_t last_change;
} BtnDebounce_t;

static BtnDebounce_t up = {1, 0};
static BtnDebounce_t down = {1, 0};
static BtnDebounce_t ok = {1, 0};

/* For double press */
static uint8_t up_waiting_second = 0;
static uint32_t up_first_time = 0;

static uint8_t Debounce(GPIO_TypeDef *port, uint16_t pin, BtnDebounce_t *b)
{
    uint8_t raw = HAL_GPIO_ReadPin(port, pin);
    uint32_t now = xTaskGetTickCount();

    if (raw != b->last_state)
    {
        if (now - b->last_change > pdMS_TO_TICKS(DEBOUNCE_MS))
        {
            b->last_state = raw;
            b->last_change = now;

            if (raw == GPIO_PIN_RESET) // pressed (active low)
                return 1;
        }
    }

    return 0;
}

Button_t xGetBtnPress(void)
{
    uint32_t now = xTaskGetTickCount();

    /* ---- UP button with double press ---- */
    /* ---- UP button ---- */
    if (Debounce(UP_BTN_GPIO_Port, UP_BTN_Pin, &up))
    {
        uint32_t now = xTaskGetTickCount();

        // Always emit single press immediately
        Button_t ret = BTN_UP;

        // If previous press was recent, emit double instead
        if ((now - up_first_time) < pdMS_TO_TICKS(DOUBLE_PRESS_MS))
        {
            ret = BTN_UP_DBL;
        }

        up_first_time = now;
        return ret;
    }

    if (up_waiting_second && (now - up_first_time > pdMS_TO_TICKS(DOUBLE_PRESS_MS)))
    {
        up_waiting_second = 0;
        return BTN_UP;
    }

    /* ---- Other buttons normal ---- */
    if (Debounce(DOWN_BTN_GPIO_Port, DOWN_BTN_Pin, &down))
        return BTN_DOWN;

    if (Debounce(OK_BTN_GPIO_Port, OK_BTN_Pin, &ok))
        return BTN_OK;

    return BTN_NONE;
}
