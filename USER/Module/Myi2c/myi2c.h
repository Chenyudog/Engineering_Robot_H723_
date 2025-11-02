#ifndef _BSP_SOFT_I2C_H
#define _BSP_SOFT_I2C_H
#include "stm32h7xx_hal.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F7开发板
//IIC驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/11/30
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////
// IO方向设置（针对PA2作为SDA）
#define SDA_IN()  {GPIOA->MODER &= ~(3 << (2 * 2)); GPIOA->MODER |= 0 << (2 * 2);}  // PA2输入模式（清除原模式，设置为输入）
#define SDA_OUT() {GPIOA->MODER &= ~(3 << (2 * 2)); GPIOA->MODER |= 1 << (2 * 2);}  // PA2输出模式（清除原模式，设置为推挽输出）
#define READ_SDA    HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)  // 读取SDA电平：PA2
// IO操作宏（SCL=PA0，SDA=PA2）
#define IIC_SCL(n)  (n ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET))  // SCL引脚：PA0
#define IIC_SDA(n)  (n ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET))  // SDA引脚：PA2

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);
#endif


