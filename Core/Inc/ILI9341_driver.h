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

#define UI_TOP_Y 220
#define UI_BOTTOM_Y 239
#define UI_LEFT_X 20
#define UI_RIGHT_X 300

#define FONT_WIDTH 5
#define FONT_HEIGHT 7
#define FONT_SPACING 1

void ILI9341_Init(void);
void ILI9341_Clear(uint16_t color);
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ILI9341_DrawHSpan(uint16_t y, uint16_t x0, uint16_t x1, uint16_t color);
void ILI9341_DrawVSpan(uint16_t x, uint16_t y0, uint16_t y1, uint16_t color);
void ILI9341_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ILI9341_DrawAxis(uint8_t ydivs, uint8_t xdivs);
float ILI9341_MapADCtoY(float Vin, float yDIV);
uint16_t ILI9341_MapTimetoX(uint16_t t, uint16_t points);
void ILI9341_PlotWaveform(float *buffer, float xDIV, float yDIV, uint16_t color);
void BuildWavefromBuffer(float *buffer, uint16_t points, float yDIV);
void ILI9341_UpdateVals(float xDIV, float yDIV, float Trig_Voltage);
void ILI9341_ClearWaveform(void);
void ILI9341_ClearUI(void);
void ILI9341_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bgcolor);
void ILI9341_DrawString(uint16_t x, uint16_t y, char *str, uint16_t color, uint16_t bgcolor);



#endif //ILI9341_DRIVER_H
