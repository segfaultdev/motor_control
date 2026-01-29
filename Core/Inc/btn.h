#ifndef __BTN_H__
#define __BTN_H__

typedef enum {
    BTN_NONE = 0,
    BTN_UP,
    BTN_DOWN,
    BTN_OK,
    BTN_UP_DBL   // NEW
} Button_t;


Button_t xGetBtnPress();

#endif