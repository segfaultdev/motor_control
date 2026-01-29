#ifndef UI_H
#define UI_H

#include <stdint.h>
#include "ili9488.h"

/* Colors */
#define COLOR_BG       ILI9488_BLACK
#define COLOR_TEXT     ILI9488_WHITE
#define COLOR_TITLE    ILI9488_WHITE
#define COLOR_CARD_BG  0x1082
#define COLOR_BORDER   0x4208
#define COLOR_ACCENT   0x07E0
#define COLOR_SUBTLE   0x8410

typedef enum {
    PAGE_PRODUCT,
    PAGE_MODEL
} UI_Page;

/* UI control */
void UI_Init(void);
void UI_Update(void);
void UI_NavigateUp(void);
void UI_NavigateDown(void);
void UI_TogglePage(void);

/* Status updates */
void UI_SetResponse(char *msg);
void UI_SetConfidence(float conf);
void UI_IncrementCount(void);

/* Access current selection */
uint8_t UI_GetProduct(void);
uint8_t UI_GetModel(void);
UI_Page UI_GetPage(void);

/* Get names for UART */
const char* UI_GetProductName(uint8_t i);
const char* UI_GetModelName(uint8_t i);

#endif
