#ifndef ILI9341_DRIVER_H
#define ILI9341_DRIVER_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdint.h>

#define BLACK 0x0000
#define WHITE 0xFFFF

#define ILI9341_WIDTH 320
#define ILI9341_HEIGHT 240

void ILI9341_Init(void);
void ILI9341_Clear(uint16_t color);
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t colour);
void ILI9341_drawHLine(uint16_t y, uint16_t colour);
void ILI9341_drawVLine(uint16_t x, uint16_t colour);
void ILI9341_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
uint16_t ILI9341_MapADCtoY(uint16_t adc_value);
void ILI9341_PlotWaveform(uint16_t *buffer, uint16_t length, uint16_t colour);
void ILI9341_DrawAxis(void);


#endif //ILI9341_DRIVER_H
