#ifndef __SSD1306_H
#define __SSD1306_H

#include "main.h"
#include <string.h>
#include <stdint.h>

#define SSD1306_ADDR         0x3C
#define SSD1306_CMD_MODE     0x00
#define SSD1306_DATA_MODE    0x40
#define SSD1306_DISPLAY_OFF  0xAE
#define SSD1306_DISPLAY_ON   0xAF

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   8

/* I2C transport - these need the handle */
void SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_WriteCmd(I2C_HandleTypeDef *hi2c, uint8_t cmd);
void SSD1306_WriteData(I2C_HandleTypeDef *hi2c, uint8_t *data, uint16_t len);
void SSD1306_UpdateDisplay(I2C_HandleTypeDef *hi2c);

/* Buffer operations - no I2C needed */
void SSD1306_Clear(void);
void SSD1306_SetPixel(uint8_t x, uint8_t y, uint8_t on);
void SSD1306_DrawChar(uint8_t x, uint8_t y, char c);
void SSD1306_DrawString(uint8_t x, uint8_t y, const char *str);
void SSD1306_DrawInt(uint8_t x, uint8_t y, int32_t value);

/* Big font (3x scaled) - each char is 15x21 px, spacing 16 px */
void SSD1306_DrawCharBig(uint8_t x, uint8_t y, char c);
void SSD1306_DrawStringBig(uint8_t x, uint8_t y, const char *str);
void SSD1306_DrawAngle(float angle);

#endif
