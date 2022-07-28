#ifndef ILI_9341_H
#define ILI_9341_H

#include <stdint.h>

#define DC_PIN GPIO_PIN_1
#define DC_GPIO_PORT GPIOA
#define DC_GPIO_CLK RCU_GPIOA

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
} rect_t;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} color_t;

void ili9341_initialize();
void ili9341_fill();
uint8_t ili9341_read_id(void);
void ili9341_shutdown();
void ili9341_set_rotation(uint8_t m);
void ili9341_draw_h_line(uint16_t y, const uint8_t *data, const uint8_t *palette);

#endif

