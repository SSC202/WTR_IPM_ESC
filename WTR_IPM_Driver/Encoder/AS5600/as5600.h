/**
 * @file    STM32 AS5600 磁编码器驱动程序
 * @brief   AS5600 是 14Bit 的单圈绝对角度检测传感器,拥有 PWM/I2C 等多种信息输出方式,这里采用 IIC 进行数据输出
 * @attention   使用前请选择驱动方式(硬件/软件IIC)
 *              使用时使用AS5600_Init()进行初始化
 *              在一个循环的程序中定时运行AS5600_Get_Angle()
 * @author  SSC
 */
#ifndef __AS5600_H
#define __AS5600_H

#include "stm32g4xx.h"

/************************ 驱动方式选择 ********************************/

#define AS5600_SoftWare_IIC 1 // 使用软件IIC
#define AS5600_HardWare_IIC 0 // 使用硬件IIC

/*************************** 端口定义 *********************************/

#if (AS5600_SoftWare_IIC == 1)

#define AS5600_SCL_Port GPIOA
#define AS5600_SDA_Port GPIOA
#define AS5600_SCL_Pin  GPIO_PIN_7
#define AS5600_SDA_Pin  GPIO_PIN_6

#endif

/************************** 寄存器定义 ********************************/

#define AS5600_ANGLE_REG 0x0C // 角度寄存器

#define AS5600_ADDR      0X36 // 设备地址

/*************************** 函数定义 *********************************/
void AS5600_Init(void);
void AS5600_Get_Angle(float *encoder_angle);

#endif
