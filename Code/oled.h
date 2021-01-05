#ifndef _OLED_H
#define _OLED_H

#include "stdint.h"
#define u8 uint8_t
#define u32 uint32_t

void OLED_Init(void);                                       // 初始化OLED
void OLED_Refresh_Gram(void);                               // 更新显存至OLED（图像刷新）
void OLED_Clear(void);                                      // 清屏
void OLED_DrawPoint(u8 x, u8 y);                            // 画点
void OLED_ClearPoint(u8 x, u8 y);                           // 清点
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size);            // 显示字符
void OLED_DrawCircle(u8 x, u8 y, u8 r);                     // 画圆（以(x, y)为圆心，r 为半径） 
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size);         // 显示字符串
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size);    // 显示多个数字
u32 OLED_Pow(u8 m, u8 n);                                   // m^n   

#endif