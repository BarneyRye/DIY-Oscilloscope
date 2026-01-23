#include "ILI9341_driver.h"

extern SPI_HandleTypeDef hspi1;

static void ILI9341_WriteCommand(uint8_t cmd) {
	HAL_GPIO_WritePin(LCD_DCRS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &cmd, 1 , HAL_MAX_DELAY);
	HAL_GPIO_WritePin(LCD_CS_PIN, GPIO_PIN_SET);
}

static void ILI9341_WriteData(uint8_t *data, uint16t_t size) {
	HAL_GPIO_WritePin(LCD_DCRS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, data, size , HAL_MAX_DELAY);
	HAL_GPIO_WritePin(LCD_CS_PIN, GPIO_PIN_SET);
}

//Reset Display
void ILI9341_Reset(void) {
	HAL_GPIO_WritePin(LCD_RESET_PIN, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(LCD_RESET_PIN, GPIO_PIN_SET);
	HAL_Delay(120);
}

//Initialise Display
void ILI9341_Init(void) {
	ILI9341_Reset();
	ILI9341_WriteCommand(0x028); //Display OFF

	uint8_t data = 0x55; //16-bit colour
	ILI9341_WriteCommand(0x3A); //Pixel Format Set
	ILI9341_WriteData(&data, 1);

	ILI9341_WriteCommand(0x29); //Display On
}

//Set Address Window
void ILI9341_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
	uint8_t data[4];

	ILI9341_WriteCommand(0x2A); //Column Address Set
	data[0] = x0 >> 8;
	data[1] = x0 & 0xFF;
	data[2] = x1 >> 8;
	data[3] = x1 & 0xFF;
	ILI9341_WriteData(data, 4);

	ILI9341_WriteCommand(0x2B); //Page Address Set
	data[0] = y0 >> 8;
	data[1] = y0 & 0xFF;
	data[2] = y1 >> 8;
	data[3] = y1 & 0xFF;
	ILI9341_WriteData(data, 4);

	ILI9341_WriteCommand(0x2C); //Memory Write
}

//Draw Pixel
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t colour) {
	ILI9341_SetAddressWindow(x, y, x, y);
	uint8_t data[2] { colour >> 8, colour & 0xFF };
	ILI9341_WriteData(data, 2);
}

//Clear Display
void ILI9341_Clear(uint16_t color) {
	ILI9341_SetAddressWindow(0, 0, ILI9341_WIDTH - 1, ILI9341_HEIGHT - 1);
	uint8_t data[2] { color >> 8, color & 0xFF };

	for (uint32_t i = 0; i < ILI9341_WIDTH * ILI9341_HEIGHT; i++) {
		HAL_WriteData(data, 2);
	}
}

//Draw Lines
void ILI9341_drawHLine(uint16_t y, uint16_t colour) {
	for (uint16_t x = 0; x < ILI9341_WIDTH; x++) {
		ILI9341_DrawPixel(x, y, colour);
	}
}

void ILI9341_drawVLine(uint16_t x, uint16_t colour) {
	for (uint16_t y = 0; y < ILI9341_HEIGHT; y++) {
		ILI9341_DrawPixel(x, y, colour);
	}
}

//Draw Grid
void ILI9341_DrawAxis(uint8_t ydivs, uint8_t xdivs) {
	ILI9341_drawHLine(ILI9341_HEIGHT / 2, BLUE); //Horizontal Centre Line
	for (uint8_t i=1; i<=ydivs; i++){
		ILI9341_drawHLine((ILI9341_HEIGHT/2) + 100/ydivs, WHITE); // Adds y-grid lines
		ILI9341_drawHLine((ILI9341_HEIGHT/2) - 100/ydivs, WHITE);
	}
	ILI9341_drawVLine(20, BLUE);  //Vertical x=0 Line
	for (uint8_t i=1; i<=xdivs; i++){
		ILI9341_drawVLine(20+(280/xdivs), WHITE);  //Adds x-grid lines
	}
}

//Map ADC value to Y coordinate
uint32_t ILI9341_MapADCtoY(float Vin, float yDIV) {
	//Between pixels 20->220
	//Centre is 120 i.e. 0v
	//220 at 0v + yDIV*no.Divs
	//20 at 0v - yDIV*no.Divs
	float Vin_Bot = -yDIV*Y_DIVISIONS;
	float Vin_Top = yDIV*Y_DIVISIONS;
	float yf = 20.0f + ((Vin-Vin_Bot)/(Vin_Top-Vin_Bot) * (220.0f-20.0f));
	return (uint32_t)yf;
}

uint16_t ILI9341_MAPTimetoX(uint16_t t, uint16_t points) {
	//Between 20 -> 280
	//20 at t=0
	//280 at t=points-1
	return 20 + ((t-0)/(points-1-0) * (280-20));
}

//Plot Waveform
void ILI9341_PlotWaveform(float *buffer, float xDIV, float yDIV, uint16_t colour) {
	uint32_t time = (uint32_t)xDIV * (uint32_t)X_DIVISIONS;
	uint16_t points = time/(uint16_t)Sample_period;
	for (uint16_t i=0; i<points; i++) {
		uint16_t y = ILI9341_MapADCtoY(buffer[i], yDIV);
		uint16_t x = ILI9341_MAPTimetoX(i, points);
		ILI9341_DrawPixel(x, y, colour);
	}
}

