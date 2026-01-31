#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "drv_dwt.h"
#include "Power_task.h"
#include "myi2c.h"
#include "i2c.h"

/* -------------------------------- 读取功率相关 --------------------------------- */
float ina226_bus_voltage = 0.0f;    // 总线电压（V）
float ina226_shunt_voltage = 0.0f;  // 分流电压（mV）
float ina226_current = 0.0f;        // 电流（A）
float ina226_power = 0.0f;          // 功率（W）
uint16_t reg;//寄存器内容
float power_update_timestamp = 0.0f;//检测功率是否更新，用于rls算法

/* -------------------------------- 读取功率相关 --------------------------------- */

/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t power_task_dwt = 0;   // 毫秒监测
static float power_task_dt = 0;       // 线程实际运行时间dt
static float power_task_delta = 0;    // 监测线程运行时间
static float power_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */
void PowerTask_Entry(void const * argument)
{
    power_task_dt = dwt_get_delta(&power_task_dwt);
    power_task_start_dt = dwt_get_time_ms();
    INA226_Init();//初始化ina226模块，初始化失败则读不到数据

    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        power_task_delta = dwt_get_time_ms() - power_task_start_dt;
        power_task_start_dt = dwt_get_time_ms();
        power_task_dt = dwt_get_delta(&power_task_dwt);
    /* -------------------------------- 调试监测线程调度 --------------------------------- */
        INA226_UpdateData();//更新功率
        vTaskDelay(1);
        //reg = INA226_ReadRegister(0x00);//用于测试有没有初始化成功
    }
}


/**
 * @brief  初始化INA226（含IIC初始化+寄存器配置）
 * @param  无
 * @retval 无
 */
void INA226_Init(void) {
    IIC_Init();  // 初始化软件IIC引脚

    // 配置寄存器
    INA226_WriteRegister(INA226_REG_CONFIG, INA226_CONFIG_DEFAULT);
    vTaskDelay(100);

    // 校准寄存器（关键：决定电流/功率计算精度）
    INA226_WriteRegister(INA226_REG_CALIB, INA226_CALIB_DEFAULT);
    vTaskDelay(100);
}

/**
 * @brief  向INA226寄存器写入数据
 * @param  reg_addr：寄存器地址
 * @param  data：16位数据
 * @retval HAL_StatusTypeDef：成功返回HAL_OK，失败返回HAL_ERROR
 */
HAL_StatusTypeDef INA226_WriteRegister(uint8_t reg_addr, uint16_t data) {
    IIC_Start();  // 发送起始信号

    // 发送从机地址（写操作：7位地址左移1位 + 0）
    IIC_Send_Byte((INA226_I2C_ADDR << 1) | 0x00);
    if (IIC_Wait_Ack() != 0) {  // 等待应答
        IIC_Stop();
        return HAL_ERROR;
    }

    // 发送寄存器地址
    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return HAL_ERROR;
    }

    // 发送高8位数据
    IIC_Send_Byte((data >> 8) & 0xFF);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return HAL_ERROR;
    }

    // 发送低8位数据
    IIC_Send_Byte(data & 0xFF);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return HAL_ERROR;
    }

    IIC_Stop();  // 发送停止信号
    return HAL_OK;
}

/**
 * @brief  从INA226寄存器读取数据
 * @param  reg_addr：寄存器地址
 * @retval 16位数据（失败返回0xFFFF）
 */
uint16_t INA226_ReadRegister(uint8_t reg_addr) {
    uint8_t rx_high, rx_low;

    IIC_Start();  // 发送起始信号

    // 发送从机地址（写操作：选择寄存器）
    IIC_Send_Byte((INA226_I2C_ADDR << 1) | 0x00);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0xFFFF;
    }

    // 发送要读取的寄存器地址
    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0xFFFF;
    }

    // 重复起始信号（切换到读操作）
    IIC_Start();
    IIC_Send_Byte((INA226_I2C_ADDR << 1) | 0x01);  // 读操作：地址左移1位 + 1
    if (IIC_Wait_Ack() != 0) {
        IIC_Stop();
        return 0xFFFF;
    }

    // 读取高8位（需应答）和低8位（无需应答）
    rx_high = IIC_Read_Byte(1);  // 读高8位，发送ACK
    rx_low  = IIC_Read_Byte(0);  // 读低8位，发送NACK
    IIC_Stop();

    // 组合16位数据（大端模式）
    return (rx_high << 8) | rx_low;
}

/**
 * @brief  更新所有测量数据（总线电压/分流电压/电流/功率）
 * @param  无
 * @retval 无
 */
void INA226_UpdateData(void) {
    uint16_t raw_data;
    // 读取功率（无符号，依赖校准值）
    raw_data = INA226_ReadRegister(INA226_REG_POWER);
    if (raw_data != 0xFFFF)
    {
        ina226_power = (float)raw_data * INA226_POWER_LSB;
        power_update_timestamp = power_task_start_dt;

    }

    // 读取总线电压（无符号）
//    raw_data = INA226_ReadRegister(INA226_REG_BUS_V);
//    if (raw_data != 0xFFFF) {
//        // 总线电压 = 原始值 * 1.25mV → 转换为V（除以1000）
//        ina226_bus_voltage = (float)raw_data * INA226_BUS_V_LSB / 1000.0f;
//    }
//    dwt_delay_us(1);


    // 读取分流电压（有符号）
//    raw_data = INA226_ReadRegister(INA226_REG_SHUNT_V);
//    if (raw_data != 0xFFFF) {
//        signed_data = (int16_t)raw_data;  // 转换为有符号数（补码）
//        // 分流电压 = 原始值 * 2.5uV → 转换为mV（除以1000）
//        ina226_shunt_voltage = (float)signed_data * INA226_SHUNT_V_LSB / 1000.0f;
//    }
//    dwt_delay_us(1);

    // 读取电流（有符号，依赖校准值）
//    raw_data = INA226_ReadRegister(INA226_REG_CURRENT);
//    if (raw_data != 0xFFFF) {
//        signed_data = (int16_t)raw_data;
//        ina226_current = (float)signed_data * INA226_CURRENT_LSB;
//    }


}

/**
 * @brief  检查INA226设备ID（验证I2C通信）
 * @param  无
 * @retval 1：ID匹配（通信正常），0：失败
 */
uint8_t INA226_CheckID(void) {
    uint16_t man_id, dev_id;
    man_id = INA226_ReadRegister(INA226_REG_MAN_ID);  // 制造商ID：0x5449
    dev_id = INA226_ReadRegister(INA226_REG_DEV_ID);  // 设备ID：0x2260
    return (man_id == 0x5449 && dev_id == 0x2260) ? 1 : 0;
}






//--------------------------------------------------------硬件iic------------------------------------------------//
///**
// * @brief  初始化INA226（含IIC初始化+寄存器配置）
// * @param  无
// * @retval 无
// */
//void INA226_Init(void) {
//
//    // 配置寄存器
//    if(INA226_WriteRegister(INA226_REG_CONFIG, INA226_CONFIG_DEFAULT) == HAL_OK)
//    {
//        power_task_state = 1;//配置配置器完毕
//    }
//    vTaskDelay(1);
//    // 校准寄存器（关键：决定电流/功率计算精度）
//    if(INA226_WriteRegister(INA226_REG_CALIB, INA226_CALIB_DEFAULT) == HAL_OK)
//    {
//        power_task_state = 2;//配置校准器完毕
//    }
//    vTaskDelay(1);
//}
//
//
///**
// * @brief  向INA226寄存器写入数据
// * @param  reg_addr：寄存器地址
// * @param  data：16位数据
// * @retval HAL_StatusTypeDef：成功返回HAL_OK，失败返回HAL_ERROR
// */
//HAL_StatusTypeDef INA226_WriteRegister(uint8_t reg_addr, uint16_t data)
//{
//    i2c_tx_buffer[0] = reg_addr;        // 寄存器地址
//    i2c_tx_buffer[1] = (data >> 8) & 0xFF;  // 高8位
//    i2c_tx_buffer[2] = data & 0xFF;     // 低8位
//    return HAL_I2C_Mem_Write_DMA(&hi2c1,
//                                 INA226_I2C_ADDR << 1,  // 7位地址左移1位
//                                 reg_addr,              // 内存地址
//                                 I2C_MEMADD_SIZE_8BIT,  // 8位地址
//                                 i2c_tx_buffer,         // 数据缓冲区
//                                 3);
//}
//
///**
// * @brief  从INA226寄存器读取数据
// * @param  reg_addr：寄存器地址
// * @retval 16位数据（失败返回0xFFFF）
// */
//HAL_StatusTypeDef INA226_ReadRegister(uint8_t reg_addr) {
//    return HAL_I2C_Mem_Read_DMA(&hi2c1,
//                                INA226_I2C_ADDR << 1,  // 7位地址左移1位
//                                reg_addr,              // 内存地址
//                                I2C_MEMADD_SIZE_8BIT,  // 8位地址
//                                i2c_rx_buffer,         // 数据缓冲区
//                                2);
//}
//
///**
// * @brief  更新所有测量数据（总线电压/分流电压/电流/功率）
// * @param  无
// * @retval 无
// */
//void INA226_UpdateData(void) {
//    static uint16_t raw_data = 0;
//
//    // 读取功率（无符号，依赖校准值）
//    if (power_task_state == 3)
//    {
//        raw_data = i2c_rx_buffer[0] << 8 | i2c_rx_buffer[1];
//        if (raw_data != 0xFFFF) {
//            ina226_power = (float)raw_data * INA226_POWER_LSB;
//            power_update_timestamp = power_task_start_dt;
//        }
//        power_task_state = 4;
//    }
//    if (power_task_state == 2 || power_task_state == 4)
//    {
//        INA226_ReadRegister(INA226_REG_POWER);
//    }
//}



///**
// * @brief  检查INA226设备ID（验证I2C通信）
// * @param  无
// * @retval 1：ID匹配（通信正常），0：失败
// */
//uint8_t INA226_CheckID(void) {
//    uint16_t man_id, dev_id;
//    man_id = INA226_ReadRegister(INA226_REG_MAN_ID);  // 制造商ID：0x5449
//    dev_id = INA226_ReadRegister(INA226_REG_DEV_ID);  // 设备ID：0x2260
//    return (man_id == 0x5449 && dev_id == 0x2260) ? 1 : 0;
//}
//
//void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//    if(hi2c == &hi2c1)
//    {
//        if (power_task_state == 0) power_task_state = 1;
//        if (power_task_state == 1) power_task_state = 2;
//    }
//}
//
//void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//    if(hi2c == &hi2c1)
//    {
//        power_task_state = 3;
//    }
//}
///**



//--------------------------------------------------------硬件iic------------------------------------------------//