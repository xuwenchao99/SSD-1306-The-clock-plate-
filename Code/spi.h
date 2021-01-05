#ifndef _SPI_H
#define _SPI_H

#include "stdint.h"
#include "main.h"
#include "stm32f1xx.h"

#define u8 uint8_t
#define OLED_CMD 0  // 命令声明
#define OLED_DATA 1 // 数据声明

#define OLED_RES(x)     x ? HAL_GPIO_WritePin(GPIOA, RES_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA, RES_Pin, GPIO_PIN_RESET) // RES复位
#define OLED_DC(x)      x ? HAL_GPIO_WritePin(GPIOA, DC_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA, DC_Pin, GPIO_PIN_RESET)   // 0 表示传输命令，1 表示传输数据
#define OLED_CS(x)      x ? HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_RESET)   // CS 片选信号，低电平有效


void OLED_SPI_Init(void);                       // 配置 MCU 的SPI
void SPI_WriteByte(unsigned char data, unsigned char cmd); // 向寄存器地址写一个 Byte 的数据
void WriteCmd(unsigned char cmd);               // 写命令     
void WriteDat(unsigned char date);              // 写数据

#endif