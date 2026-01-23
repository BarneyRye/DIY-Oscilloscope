#ifndef ILI9341_DRIVER_H
#define ILI9341_DRIVER_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <math.h>

#define BLACK 0x0000
#define WHITE 0xFFFF
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define YELLOW 0xFFE0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define ORANGE 0xFC00


#define ILI9341_WIDTH 320
#define ILI9341_HEIGHT 240

void ILI9341_Init(void);
void ILI9341_Clear(uint16_t color);
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t colour);
void ILI9341_drawHLine(uint16_t y, uint16_t colour);
void ILI9341_drawVLine(uint16_t x, uint16_t colour);
void ILI9341_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ILI9341_DrawAxis(uint8_t ydivs, uint8_t xdivs);
float ILI9341_MapADCtoY(float Vin, float yDIV);
uint16_t ILI9341_MAPTimetoX(uint16_t t, uint16_t points);
void ILI9341_PlotWaveform(float *buffer, float xDIV, float yDIV);


#endif //ILI9341_DRIVER_H
