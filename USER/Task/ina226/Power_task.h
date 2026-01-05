#ifndef __BSP_INA226_H__
#define __BSP_INA226_H__

#include "stm32h7xx_hal.h"


// ************************** 测量参数（根据校准值调整） **************************
#define INA226_CURRENT_LSB      0.001f  // 电流LSB：1mA/bit              //通过具体需求修改
#define INA226_BUS_V_LSB        1.25f   // 总线电压LSB：1.25mV/bit        //固定不用修改
#define INA226_SHUNT_V_LSB      2.5f    // 分流电压LSB：2.5uV/bit         //固定不用修改
#define INA226_POWER_LSB        (INA226_CURRENT_LSB*25.0f)  // 功率LSB：25mW/bit（=25*电流LSB） //半固定不用修改


// ************************** 初始化参数配置 **************************
// 配置寄存器
#define INA226_CONFIG_DEFAULT   0x0727   //对照文件配置   //0x0327：平均64次 1.1ms 1.1ms 连续测分流和总线

// 校准寄存器
#define INA226_CALIB_DEFAULT    0x1400   //对照文件配置    //0x0400 A_LSB取1mA  //这个宏定义就是CAL转为十六进制


// ************************** INA226 I2C地址配置 **************************
//A0、A1由原理图可知都接地
#define INA226_I2C_ADDR         0x40

// ************************** 寄存器地址定义 **************************
#define INA226_REG_CONFIG       0x00  // 配置寄存器
#define INA226_REG_SHUNT_V      0x01  // 分流电压寄存器
#define INA226_REG_BUS_V        0x02  // 总线电压寄存器
#define INA226_REG_POWER        0x03  // 功率寄存器
#define INA226_REG_CURRENT      0x04  // 电流寄存器
#define INA226_REG_CALIB        0x05  // 校准寄存器

#define INA226_REG_MAN_ID       0xFE  // 制造商ID寄存器（固定为0x5449）
#define INA226_REG_DEV_ID       0xFF  // 设备ID寄存器（固定为0x2260）



// ************************** 外部变量声明 **************************
extern float ina226_bus_voltage;    // 总线电压（V）
extern float ina226_shunt_voltage;  // 分流电压（mV）
extern float ina226_current;        // 电流（A）
extern float ina226_power;          // 功率（W）
extern float power_update_timestamp ;//检测功率是否更新，用于rls算法

// ************************** 函数声明 **************************
void INA226_Init(void);                           // 初始化INA226
uint16_t INA226_ReadRegister(uint8_t reg_addr);    // 读指定寄存器
HAL_StatusTypeDef INA226_WriteRegister(uint8_t reg_addr, uint16_t data);  // 写指定寄存器
void INA226_UpdateData(void);                     // 更新所有测量数据
uint8_t INA226_CheckID(void);                     // 检查芯片ID（验证通信）

#endif