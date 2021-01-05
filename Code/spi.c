#include "spi.h"
// 外部声明
extern SPI_HandleTypeDef hspi1;
/*
	实现四个函数:
	// 1、GPIO 模拟 SPI 端口初始化
	2、向寄存器地址中写入一个 Byte 数据
	3、写命令
	4、写数据
*/

// SPI_HandleTypeDef hspi;

// 引脚初始化(都置为输出)
void OLED_SPI_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = CS_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RES_GPIO_Port, &GPIO_InitStruct);
	OLED_RES(1);
}

/*
	data 表示要写入的数据/命令
	cmd  数据/命令标志，0 表示命令，1 表示数据
*/
void SPI_WriteByte(unsigned char data, unsigned char cmd)
{
	OLED_DC(cmd);
	OLED_CS(0);
	// 硬件 SPI
	HAL_SPI_Transmit(&hspi1, &data, 1, 10);
	OLED_CS(1);
	OLED_DC(1);
}

/* 写命令 */
void WriteCmd(unsigned char cmd)
{
	SPI_WriteByte(cmd, OLED_CMD);
}

/* 写数据 */
void WriteDat(unsigned char data)
{
	SPI_WriteByte(data, OLED_DATA);
}