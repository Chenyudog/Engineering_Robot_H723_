#ifndef MYI2C_H
#define MYI2C_H
#include "stm32h7xx_hal.h"

//
// Created by 张荣开 on 25-11-2.
//

/*-----------------------配置软件iic引脚-----------------*/
#define IIC_SCL_PORT    GPIOA
#define IIC_SCL_PIN     GPIO_PIN_0
#define IIC_SDA_PORT    GPIOA
#define IIC_SDA_PIN     GPIO_PIN_2
/*-----------------------配置软件iic引脚-----------------*/

#define SDA_PIN_NUM     (31 - __CLZ((uint32_t)IIC_SDA_PIN))
#define SDA_IN()  do { \
    IIC_SDA_PORT->MODER &= ~(3 << (2 * SDA_PIN_NUM)); \
    IIC_SDA_PORT->MODER |= 0 << (2 * SDA_PIN_NUM); \
} while(0)
#define SDA_OUT() do { \
    IIC_SDA_PORT->MODER &= ~(3 << (2 * SDA_PIN_NUM)); \
    IIC_SDA_PORT->MODER |= 1 << (2 * SDA_PIN_NUM); \
} while(0)
#define READ_SDA        HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN)
#define IIC_SCL(n)      HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, n ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define IIC_SDA(n)      HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, n ? GPIO_PIN_SET : GPIO_PIN_RESET)



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


