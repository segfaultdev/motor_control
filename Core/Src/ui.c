#include "ui.h"
#include <stdio.h>

/* ---------------- Data ---------------- */

static char *product_items[] = {
    "BOTTLE",
    "BEARING"};

static char *model_items[] = {
    "RESNET50",
    "MOBILENETV2",
    "EFFICIENTNETB0"};

#define PRODUCT_COUNT (sizeof(product_items) / sizeof(product_items[0]))
#define MODEL_COUNT (sizeof(model_items) / sizeof(model_items[0]))

/* ---------------- State ---------------- */

static UI_Page page = PAGE_PRODUCT;

static uint8_t product_sel = 0;
static uint8_t model_sel = 0;

static uint8_t last_product_sel = 255;
static uint8_t last_model_sel = 255;

static char response[32] = "Waiting...";
static int confidence = 0;
static uint16_t count = 0;

/* Dirty flags */
#define DIRTY_PAGE (1 << 0)
#define DIRTY_MENU (1 << 1)
#define DIRTY_PANEL (1 << 2)

static volatile uint8_t dirty = DIRTY_PAGE;

/* ---------------- Internals ---------------- */
static void Draw_Product(void);
static void Draw_Model(void);
static void Draw_Bottom(void);

/* ---------------- Public ---------------- */

void UI_Init(void)
{
    fillScreen(COLOR_BG);
    dirty = DIRTY_PAGE;
}

void UI_Update(void)
{
    if (dirty & DIRTY_PAGE)
    {
        fillScreen(COLOR_BG);
        last_product_sel = 255;
        last_model_sel = 255;
        dirty |= DIRTY_MENU | DIRTY_PANEL;
    }

    if (dirty & DIRTY_MENU)
    {
        if (page == PAGE_PRODUCT)
            Draw_Product();
        else
            Draw_Model();
    }

    if (dirty & DIRTY_PANEL)
    {
        Draw_Bottom();
    }

    dirty = 0;
}

void UI_NavigateUp(void)
{
    if (page == PAGE_PRODUCT && product_sel > 0)
    {
        product_sel--;
        dirty |= DIRTY_MENU;
    }

    if (page == PAGE_MODEL && model_sel > 0)
    {
        model_sel--;
        dirty |= DIRTY_MENU;
    }
}

void UI_NavigateDown(void)
{
    if (page == PAGE_PRODUCT && product_sel < PRODUCT_COUNT - 1)
    {
        product_sel++;
        dirty |= DIRTY_MENU;
    }

    if (page == PAGE_MODEL && model_sel < MODEL_COUNT - 1)
    {
        model_sel++;
        dirty |= DIRTY_MENU;
    }
}

void UI_TogglePage(void)
{
    page = (page == PAGE_PRODUCT) ? PAGE_MODEL : PAGE_PRODUCT;
    dirty |= DIRTY_PAGE;
}

void UI_SetResponse(char *msg)
{
    if (strcmp(response, msg) != 0)
    {
        strncpy(response, msg, sizeof(response) - 1);
        response[sizeof(response) - 1] = '\0';
        dirty |= DIRTY_PANEL;
    }
}

void UI_SetConfidence(float conf)
{
    /* Only update if difference is significant (avoid float precision issues) */
    if (conf != confidence)
    {
        confidence = conf;
        dirty |= DIRTY_PANEL;
    }
}

void UI_IncrementCount(void)
{
    count++;
    dirty |= DIRTY_PANEL;
}

/* ---- getters ---- */

uint8_t UI_GetProduct(void) { return product_sel; }
uint8_t UI_GetModel(void) { return model_sel; }
UI_Page UI_GetPage(void) { return page; }

const char *UI_GetProductName(uint8_t i) { return product_items[i]; }
const char *UI_GetModelName(uint8_t i) { return model_items[i]; }

/* ---------------- Drawing ---------------- */

static void DrawItem(uint8_t idx, uint8_t sel, char *text)
{
    uint16_t y = 60 + idx * 55;
    uint16_t bg = sel ? COLOR_ACCENT : COLOR_CARD_BG;
    uint16_t txt = sel ? COLOR_BG : COLOR_TEXT;

    fillRect(15, y, 290, 42, bg);

    if (!sel)
    {
        drawFastHLine(15, y, 290, COLOR_BORDER);
        drawFastHLine(15, y + 41, 290, COLOR_BORDER);
    }

    ILI9488_printText(text, 30, y + 12, txt, bg, 2);
}

static void Draw_Product(void)
{
    ILI9488_printText("PRODUCT SELECTION", 15, 20, COLOR_TITLE, COLOR_BG, 2);
    drawFastHLine(15, 45, 290, COLOR_BORDER);

    /* Show current model selection */
    char info[40];
    sprintf(info, "MODEL: %s", model_items[model_sel]);
    ILI9488_printText(info, 15, 250, COLOR_SUBTLE, COLOR_BG, 1);

    if (last_product_sel == 255)
    {
        for (int i = 0; i < PRODUCT_COUNT; i++)
            DrawItem(i, (i == product_sel), product_items[i]);
    }
    else
    {
        DrawItem(last_product_sel, 0, product_items[last_product_sel]);
        DrawItem(product_sel, 1, product_items[product_sel]);
    }

    last_product_sel = product_sel;
}

static void Draw_Model(void)
{
    ILI9488_printText("MODEL SELECTION", 15, 20, COLOR_TITLE, COLOR_BG, 2);
    drawFastHLine(15, 45, 290, COLOR_BORDER);

    /* Show current product selection */
    char info[40];
    sprintf(info, "PRODUCT: %s", product_items[product_sel]);
    ILI9488_printText(info, 15, 250, COLOR_SUBTLE, COLOR_BG, 1);

    if (last_model_sel == 255)
    {
        for (int i = 0; i < MODEL_COUNT; i++)
            DrawItem(i, (i == model_sel), model_items[i]);
    }
    else
    {
        DrawItem(last_model_sel, 0, model_items[last_model_sel]);
        DrawItem(model_sel, 1, model_items[model_sel]);
    }

    last_model_sel = model_sel;
}

static void Draw_Bottom(void)
{
    fillRect(0, 300, 320, 180, COLOR_CARD_BG);
    drawFastHLine(0, 300, 320, COLOR_BORDER);

    char buf[50];

    /* Show current command that will be sent */
    sprintf(buf, "CMD: P%dM%d", product_sel, model_sel);
    ILI9488_printText(buf, 15, 310, COLOR_ACCENT, COLOR_CARD_BG, 2);

    sprintf(buf, "%s | %s", product_items[product_sel], model_items[model_sel]);
    ILI9488_printText(buf, 15, 335, COLOR_SUBTLE, COLOR_CARD_BG, 1);

    sprintf(buf, "COUNT: %d", count);
    ILI9488_printText(buf, 15, 365, COLOR_TEXT, COLOR_CARD_BG, 2);

    sprintf(buf, "CONF: %d", confidence);
    ILI9488_printText(buf, 170, 365, COLOR_TEXT, COLOR_CARD_BG, 2);

    ILI9488_printText("STATUS:", 15, 400, COLOR_SUBTLE, COLOR_CARD_BG, 1);
    ILI9488_printText(response, 15, 425, COLOR_TEXT, COLOR_CARD_BG, 2);
}
