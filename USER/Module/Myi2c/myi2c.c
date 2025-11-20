#include "myi2c.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "drv_dwt.h"
//
// Created by 张荣开 on 25-11-2.
//


//IIC初始化
void IIC_Init(void)
{

    IIC_SDA(1);
    IIC_SCL(1);
}

//产生IIC起始信号
void IIC_Start(void)
{
    SDA_OUT();//SDA设置为输出模式
    IIC_SDA(1);
    IIC_SCL(1);
    dwt_delay_us(1);
    IIC_SDA(0);
    dwt_delay_us(1);
    IIC_SCL(0);
}

//产生IIC停止信号
void IIC_Stop(void)
{
    SDA_OUT();//SDA设置为输出模式
    IIC_SCL(0);
    IIC_SDA(0);
    dwt_delay_us(1);
    IIC_SCL(1);
    dwt_delay_us(1);
    IIC_SDA(1);
    dwt_delay_us(1);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
    uint8_t ucErrTime=0;
    SDA_IN();
    IIC_SDA(1);dwt_delay_us(1);
    IIC_SCL(1);dwt_delay_us(1);
    while(READ_SDA)
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL(0);
    return 0;
}

//产生ACK应答
void IIC_Ack(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(0);
    dwt_delay_us(1);
    IIC_SCL(1);
    dwt_delay_us(1);
    IIC_SCL(0);
}

//不产生ACK应答
void IIC_NAck(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(1);
    dwt_delay_us(1);
    IIC_SCL(1);
    dwt_delay_us(1);
    IIC_SCL(0);
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    IIC_SCL(0);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
        IIC_SDA((txd&0x80)>>7);
        txd<<=1;
        dwt_delay_us(1);   
        IIC_SCL(1);
        dwt_delay_us(1);
        IIC_SCL(0);
        dwt_delay_us(1);
    }
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
    {
        IIC_SCL(0);
        dwt_delay_us(1);
        IIC_SCL(1);
        receive<<=1;
        if(READ_SDA)receive++;
        dwt_delay_us(1);
    }
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK
    return receive;
}



