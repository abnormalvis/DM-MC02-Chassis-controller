#include "headfile.h"
#include "oledfont.h" // Font tables

extern I2C_HandleTypeDef hi2c1;

static void OLED_I2C1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00D049F7;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
}

uint8_t CMD_Data[] = {
	0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F,

	0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD8, 0x05, 0xD9, 0xF1, 0xDA, 0x12,

	0xD8, 0x30, 0x8D, 0x14, 0xAF}; // SSD1306 init command sequence

void WriteCmd(void)
{
	uint8_t i = 0;
	for (i = 0; i < 27; i++)
	{
		HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, CMD_Data + i, 1, 0x100);
	}
}
// Write one command byte
void OLED_WR_CMD(uint8_t cmd)
{
	HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 0x100);
}
// Write one data byte
void OLED_WR_DATA(uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, &data, 1, 0x100);
}
// Initialize OLED
void OLED_Init(void)
{
	OLED_I2C1_Init();
	HAL_Delay(200);

	WriteCmd();
}
// Clear screen
void OLED_Clear(void)
{
	uint8_t i, n;
	for (i = 0; i < 8; i++)
	{
		OLED_WR_CMD(0xb0 + i);
		OLED_WR_CMD(0x00);
		OLED_WR_CMD(0x10);
		for (n = 0; n < 128; n++)
			OLED_WR_DATA(0);
	}
}
// Turn OLED display on
void OLED_Display_On(void)
{
	OLED_WR_CMD(0X8D); // Set charge pump
	OLED_WR_CMD(0X14); // DCDC ON
	OLED_WR_CMD(0XAF); // DISPLAY ON
}
// Turn OLED display off
void OLED_Display_Off(void)
{
	OLED_WR_CMD(0X8D); // Set charge pump
	OLED_WR_CMD(0X10); // DCDC OFF
	OLED_WR_CMD(0XAE); // DISPLAY OFF
}
void OLED_Set_Pos(uint8_t x, uint8_t y)
{
	OLED_WR_CMD(0xb0 + y);
	OLED_WR_CMD(((x & 0xf0) >> 4) | 0x10);
	OLED_WR_CMD(x & 0x0f);
}

void OLED_On(void)
{
	uint8_t i, n;
	for (i = 0; i < 8; i++)
	{
		OLED_WR_CMD(0xb0 + i); // Page address 0~7
		OLED_WR_CMD(0x00);	   // Column low nibble
		OLED_WR_CMD(0x10);	   // Column high nibble
		for (n = 0; n < 128; n++)
			OLED_WR_DATA(1);
	} // Fill all pixels
}
unsigned int oled_pow(uint8_t m, uint8_t n)
{
	unsigned int result = 1;
	while (n--)
		result *= m;
	return result;
}

// Display one character at (x, y)
// x:0~127
// y:0~63
// mode is not used in this implementation
// size: 16 or 8
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size)
{
	unsigned char c = 0, i = 0;
	c = chr - ' '; // Font table starts from space
	if (x > 128 - 1)
	{
		x = 0;
		y = y + 2;
	}
	if (Char_Size == 16)
	{
		OLED_Set_Pos(x, y);
		for (i = 0; i < 8; i++)
			OLED_WR_DATA(F8X16[c * 16 + i]);
		OLED_Set_Pos(x, y + 1);
		for (i = 0; i < 8; i++)
			OLED_WR_DATA(F8X16[c * 16 + i + 8]);
	}
	else
	{
		OLED_Set_Pos(x, y);
		for (i = 0; i < 6; i++)
			OLED_WR_DATA(F6x8[c][i]);
	}
}

void OLED_ShowNum(uint8_t x, uint8_t y, unsigned int num, uint8_t len, uint8_t size2)
{
	uint8_t t, temp;
	uint8_t enshow = 0;
	for (t = 0; t < len; t++)
	{
		temp = (num / oled_pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				OLED_ShowChar(x + (size2 / 2) * t, y, ' ', size2);
				continue;
			}
			else
				enshow = 1;
		}
		OLED_ShowChar(x + (size2 / 2) * t, y, temp + '0', size2);
	}
}

// Display a null-terminated string
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t Char_Size)
{
	unsigned char j = 0;
	while (chr[j] != '\0')
	{
		OLED_ShowChar(x, y, chr[j], Char_Size);
		x += 8;
		if (x > 120)
		{
			x = 0;
			y += 2;
		}
		j++;
	}
}
// Display one Chinese glyph from Hzk table
// hzk index maps to a 16x16 bitmap entry
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
	uint8_t t, adder = 0;
	OLED_Set_Pos(x, y);
	for (t = 0; t < 16; t++)
	{
		OLED_WR_DATA(Hzk[2 * no][t]);
		adder += 1;
	}
	OLED_Set_Pos(x, y + 1);
	for (t = 0; t < 16; t++)
	{
		OLED_WR_DATA(Hzk[2 * no + 1][t]);
		adder += 1;
	}
}
