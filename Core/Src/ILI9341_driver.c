#include "ILI9341_driver.h"

extern SPI_HandleTypeDef hspi1;
uint16_t wave_y_min[SCREEN_WIDTH];
uint16_t wave_y_max[SCREEN_WIDTH];
uint16_t old_wave_y_min[SCREEN_WIDTH];
uint16_t old_wave_y_max[SCREEN_WIDTH];

static void ILI9341_WriteCommand(uint8_t cmd) {
	HAL_GPIO_WritePin(LCD_DCRS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &cmd, 1 , HAL_MAX_DELAY);
	HAL_GPIO_WritePin(LCD_CS_PIN, GPIO_PIN_SET);
}

static void ILI9341_WriteData(uint8_t *data, uint16_t size) {
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
	uint8_t data[2] = { colour >> 8, colour & 0xFF };
	ILI9341_WriteData(data, 2);
}

//Clear Display
void ILI9341_Clear(uint16_t color) {
	ILI9341_SetAddressWindow(0, 0, ILI9341_WIDTH - 1, ILI9341_HEIGHT - 1);
	uint8_t data[2] = { color >> 8, color & 0xFF };

	for (uint32_t i = 0; i < ILI9341_WIDTH * ILI9341_HEIGHT; i++) {
		ILI9341_WriteData(data, 2);
	}
}

//Draw Lines
void ILI9341_DrawVSpan(uint16_t x, uint16_t y0, uint16_t y1, uint16_t colour) {
	if (y0>y1) {
		uint16_t temp = y0;
		y0 = y1;
		y1 = temp;
	}
	if (y1>=ILI9341_HEIGHT) {y1 = ILI9341_HEIGHT-1;};
	ILI9341_SetAddressWindow(x, y0, x, y1);
	uint8_t data[2] = { colour >> 8, colour & 0xFF };
	for (uint16_t y=y0; y<=y1; y++) {
		ILI9341_WriteData(data, 2);
	}
}

void ILI9341_DrawHSpan(uint16_t y, uint16_t x0, uint16_t x1, uint16_t colour) {
	if (x0>x1) {
		uint16_t temp = x0;
		x0 = x1;
		x1 = temp;
	}
	if (x1>=ILI9341_WIDTH) {x1 = ILI9341_WIDTH-1;};
	ILI9341_SetAddressWindow(x0, y, x1, y);
	uint8_t data[2] = { colour >> 8, colour & 0xFF };
	for (uint16_t x=x0; x<=x1; x++) {
		ILI9341_WriteData(data, 2);
	}
}

//Draw Grid
void ILI9341_DrawAxis(uint8_t ydivs, uint8_t xdivs) {
	ILI9341_DrawHSpan(ILI9341_HEIGHT / 2, 20, ILI9341_WIDTH - 20, BLUE); //Horizontal Centre Line
	for (uint8_t i=1; i<=ydivs; i++){
		uint16_t dy = (uint16_t)(100.0f/ydivs)*i;
		ILI9341_DrawHSpan((ILI9341_HEIGHT/2) + dy, 20, ILI9341_WIDTH - 20, WHITE); // Adds y-grid lines
		ILI9341_DrawHSpan((ILI9341_HEIGHT/2) - dy, 20, ILI9341_WIDTH - 20, WHITE);
	}
	ILI9341_DrawVSpan(20, 0, ILI9341_HEIGHT, BLUE);  //Vertical x=0 Line
	for (uint8_t i=1; i<=xdivs; i++){
		ILI9341_DrawVSpan(20+((280.0f/xdivs)*i), 0, ILI9341_HEIGHT, WHITE);  //Adds x-grid lines
	}
}

//Map ADC value to Y coordinate
uint32_t ILI9341_MapADCtoY(float Vin, float yDIV) {
	float vpp = 2.0f * yDIV * Y_DIVISIONS; //Total volts displayed on screen
	float vtop = vpp / 2.0f; //Voltage at top of screen
	float vbot = -vpp / 2.0f; //Voltage at bottom of screen
	float vloc = (Vin - vbot) / (vtop - vbot); //Location of Vin between vtop and vbot (0 to 1)

	float y = 220.0f - (vloc * (220.0f - 20.0f)); //Map to Y coordinate (20 to 220)

	if (y < 20.0f) {y = 20.0f;}
	if (y > 220.0f) {y = 220.0f;}
	return (uint16_t)y;
}

uint16_t ILI9341_MapTimetoX(uint16_t t, uint16_t points) {
	//Between 20 -> 280
	//20 at t=0
	//280 at t=points-1
	return 20 + (uint16_t)((float)t * (280.0f - 20.0f) / (float)(points - 1));

}

void BuildWavefromBuffer(float *buffer, uint16_t points, float yDiv) {
	for (uint16_t i=20; i<280; i++) {
		wave_y_min[i] = 0xFFFF;
		wave_y_max[i] = 0;
	}
	for (uint16_t i=0; i<points; i++) {
		uint16_t x = ILI9341_MapTimetoX(i, points);
		uint16_t y = ILI9341_MapADCtoY(buffer[i], yDiv); //Using 0.5V/div for min/max calculation
		if (y < wave_y_min[x]) {
			wave_y_min[x] = y;
		}
		if (y > wave_y_max[x]) {
			wave_y_max[x] = y;
		}
	}
}

//Plot Waveform
void ILI9341_PlotWaveform(float *buffer, float xDIV, float yDIV, uint16_t colour) {
	uint32_t time = (uint32_t)xDIV * (uint32_t)X_DIVISIONS;
	uint16_t points = (uint16_t)((float)time/Sample_period);
	BuildWavefromBuffer(buffer, points, yDIV);
	for (uint16_t x=20; x<280; x++) {
		if (wave_y_max[x] >= wave_y_min[x]) {
			ILI9341_DrawVSpan(x, wave_y_min[x], wave_y_max[x], colour);
		}
		old_wave_y_min[x] = wave_y_min[x];
		old_wave_y_max[x] = wave_y_max[x];
		wave_y_min[x] = 0xFFFF;
		wave_y_max[x] = 0;
	}
}

void ILI9341_ClearWaveform(void){
	for (uint16_t x=20; x<280; x++) {
		if (old_wave_y_max[x] >= old_wave_y_min[x]) {
			ILI9341_DrawVSpan(x, old_wave_y_min[x], old_wave_y_max[x], BLACK);
		}
	}
}

void ILI9341_ClearUI(void){
	ILI9341_SetAddressWindow(UI_LEFT_X, UI_TOP_Y, UI_RIGHT_X, UI_BOTTOM_Y);
	uint8_t data[2] = { BLACK >> 8, BLACK & 0xFF };

	for (uint32_t i = 0; i < (UI_RIGHT_X - UI_LEFT_X + 1) * (UI_BOTTOM_Y - UI_TOP_Y + 1); i++) {
		ILI9341_WriteData(data, 2);
	}
}

void ILI9341_UpdateVals(float xDIV, float yDIV, float Trig_Voltage){
	//Address window for text area (Top Bar)
	// 20<=x<=300
	// 220<=Y<=239
	//Write xDIV=##, yDIV=##, Trig=##V
	char buffer[32];
	ILI9341_ClearUI();
	//X_DIV
	sprintf(buffer, "X: %.2fus/div", xDIV*1e6f);
	ILI9341_DrawString(25,225, buffer, WHITE, BLACK);
	//Y_DIV
	sprintf(buffer, "Y: %.2fV/div", yDIV);
	ILI9341_DrawString(130,225, buffer, WHITE, BLACK);
	//TRIG_VOLTAGE
	sprintf(buffer, "Trig: %.2fV", Trig_Voltage);
	ILI9341_DrawString(230,225, buffer, WHITE, BLACK);
}

void ILI9341_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bgcolor) {
	if (c < 0x20 || c>0x7E) {return;} //Unsupported character
	const uint8_t *char_bitmap = font5x7[c - 0x20];

	for (uint8_t col=0; col<FONT_WIDTH; col++) {
		uint8_t line = char_bitmap[col];
		for (uint8_t row=0; row<FONT_HEIGHT; row++) {
			if (line & (1 << row)) {
				ILI9341_DrawPixel(x + col, y + row, color);
			}
			else {
				ILI9341_DrawPixel(x + col, y + row, bgcolor);
			}
		}
	}
}

void ILI9341_DrawString(uint16_t x, uint16_t y, char *str, uint16_t color, uint16_t bgcolor) {
	uint16_t cursor_x = x;
	while (*str) {
		ILI9341_DrawChar(cursor_x, y, *str, color, bgcolor);
		cursor_x += FONT_WIDTH + FONT_SPACING;
		str++;
	}
}

