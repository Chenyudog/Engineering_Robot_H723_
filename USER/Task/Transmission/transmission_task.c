/**
  ******************************************************************************
  * @file    algorithm_task.c
  * @author  Liu JiaJun(187353224@qq.com) and RockZhang(2431952330@qq.com)
  * @version V2.0.0
  * @date    2025-10-16
  * @brief   机器人算法任务线程，处理复杂算法，避免在其他线程中计算造成阻塞
  ******************************************************************************
  * @attention
  *
  * 本代码遵循GPLv3开源协议，仅供学习交流使用
  * 未经许可不得用于商业用途
  *
  ******************************************************************************
  */
#include "transmission_task.h"
#include "cmsis_os.h"
#include "drv_dwt.h"
#include "adc.h"
#include "msg_freertos.h"
#include "robot_task.h"
#include "usbd_cdc_if.h"
// 建议添加明确的宏定义，确保一致性
#define MAX_USB_BUF_LEN     42
#define HEAD_BUF_LEN        4       // 帧头长度（0-3字节）

#define USB_INS_DATA_LEN    36      // 数据部分长度（9个int32_t）
#define INS_BUF_LEN    42      //4+36+2

#define USB_CMD_CHASSIS_DATA_LEN    12      // 数据部分长度（3个int32_t）
#define CMD_CHASSIS_BUF_LEN    18      //4+12+2


/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
static struct ins_msg transmission_subscribe_ins_data;
static struct cmd_chassis_msg transmission_subscribe_cmd_chassis_data;

static subscriber_t *subscribe_ins_topic;
static subscriber_t *subscribe_cmd_chassis_topic;

static uint8_t usb_txbuffer[MAX_USB_BUF_LEN] = {0};

extern uint8_t USB_Received_Data[APP_RX_DATA_SIZE];//接收usb数据缓冲区
extern uint32_t USB_Received_Len;
extern volatile uint8_t USB_Data_Ready_Flag;

static void transmission_topic_publish_init(void);
static void transmission_topic_subscribe_init(void);
static void transmission_topic_publish_push(void);
static void transmission_topic_subscribe_pull(void);
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t transmission_task_dwt = 0;   // 毫秒监测
static float transmission_task_dt = 0;       // 线程实际运行时间dt
static float transmission_task_delta = 0;    // 监测线程运行时间
static float transmission_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

void InsDataPack(void);
void CmdChassisDataPack(void);
void set_data(uint8_t rx_byte);
void UnpackPCData(void);

/* -------------------------------- 线程入口 ------------------------------- */
void TransmissionTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
        transmission_topic_subscribe_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        transmission_task_dt = dwt_get_delta(&transmission_task_dwt);
        transmission_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        transmission_task_delta = dwt_get_time_ms() - transmission_task_start_dt;
        transmission_task_start_dt = dwt_get_time_ms();
        transmission_task_dt = dwt_get_delta(&transmission_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */

/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
        transmission_topic_subscribe_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
        //接收上位机数据并解析
        if (USB_Data_Ready_Flag == 1)
        {
            UnpackPCData();
            USB_Data_Ready_Flag = 0;
        }
        //发送数据给上位机
        InsDataPack();
        CmdChassisDataPack();
/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(1);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
static void transmission_topic_publish_init(void)
{

}

static void transmission_topic_subscribe_init(void)
{
    subscribe_ins_topic = sub_register("ins_pub", sizeof(struct ins_msg));
    subscribe_cmd_chassis_topic = sub_register("cmd_ch_pub", sizeof(struct cmd_chassis_msg));
}

static void transmission_topic_publish_push(void)
{

}

static void transmission_topic_subscribe_pull(void)
{
    sub_get_msg(subscribe_ins_topic, &transmission_subscribe_ins_data);
    sub_get_msg(subscribe_cmd_chassis_topic, &transmission_subscribe_cmd_chassis_data);
}


/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */

/* -------------------------------- 线程间通讯数据包相关 ------------------------------- */


void InsDataPack()
{
    // 初始化缓冲区，避免脏数据
    memset(usb_txbuffer, 0, MAX_USB_BUF_LEN);

    // 填充帧头信息
    usb_txbuffer[0] = 0xFF;  // 帧头
    usb_txbuffer[1] = 0x05;  // 地址
    usb_txbuffer[2] = 0x13;  // 命名ID
    usb_txbuffer[3] = USB_INS_DATA_LEN;  // 数据长度

    const float scale = 10000.0f;
    int32_t chassis_imu_eul_yaw = (int32_t)(transmission_subscribe_ins_data.yaw * scale);
    int32_t chassis_imu_eul_pitch = (int32_t)(transmission_subscribe_ins_data.pitch * scale);
    int32_t chassis_imu_eul_roll = (int32_t)(transmission_subscribe_ins_data.roll * scale);
    int32_t chassis_imu_angle_x = (int32_t)(transmission_subscribe_ins_data.gyro[0] * scale);
    int32_t chassis_imu_angle_y = (int32_t)(transmission_subscribe_ins_data.gyro[1] * scale);
    int32_t chassis_imu_angle_z = (int32_t)(transmission_subscribe_ins_data.gyro[2] * scale);
    int32_t chassis_imu_accel_x = (int32_t)(transmission_subscribe_ins_data.accel[0] * scale);
    int32_t chassis_imu_accel_y = (int32_t)(transmission_subscribe_ins_data.accel[1] * scale);
    int32_t chassis_imu_accel_z = (int32_t)(transmission_subscribe_ins_data.accel[2] * scale);

    uint32_t offset = HEAD_BUF_LEN;  // 从帧头后开始
    memcpy(&usb_txbuffer[offset], &chassis_imu_eul_yaw, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_eul_pitch, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_eul_roll, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_angle_x, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_angle_y, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_angle_z, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_accel_x, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_accel_y, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_accel_z, sizeof(int32_t)); offset += sizeof(int32_t);

    // 计算校验码（覆盖0到最后一个数据字节）
    uint32_t sum_check = 0;
    uint32_t addr_check = 0;
    for (int i = 0; i < (HEAD_BUF_LEN + USB_INS_DATA_LEN); i++) {
        sum_check += usb_txbuffer[i];
        addr_check += sum_check;
    }
    usb_txbuffer[offset++] = sum_check & 0xFF;
    usb_txbuffer[offset] = addr_check & 0xFF;

    // 发送数据，检查返回值确保发送成功
    if (CDC_Transmit_HS(usb_txbuffer, INS_BUF_LEN) != USBD_OK) {

    }
}


void CmdChassisDataPack(void)
{
    // 初始化缓冲区，避免脏数据
    memset(usb_txbuffer, 0, MAX_USB_BUF_LEN);

    // 填充帧头信息
    usb_txbuffer[0] = 0xFF;  // 帧头
    usb_txbuffer[1] = 0x05;  // 地址
    usb_txbuffer[2] = 0x10;  // 命名ID    //参考木鸢战队通信协议
    usb_txbuffer[3] = USB_CMD_CHASSIS_DATA_LEN;  // 数据长度

    const float scale = 10000.0f;
    int32_t cmd_chassis_vx = (int32_t)(transmission_subscribe_cmd_chassis_data.vx * scale);
    int32_t cmd_chassis_vy = (int32_t)(transmission_subscribe_cmd_chassis_data.vx * scale);
    int32_t cmd_chassis_vw = (int32_t)(transmission_subscribe_cmd_chassis_data.vw * scale);


    uint32_t offset = HEAD_BUF_LEN;  // 从帧头后开始
    memcpy(&usb_txbuffer[offset], &cmd_chassis_vx, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &cmd_chassis_vy, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &cmd_chassis_vw, sizeof(int32_t)); offset += sizeof(int32_t);


    // 计算校验码（覆盖0到最后一个数据字节）
    uint8_t sum_check = 0;
    uint8_t addr_check = 0;
    for (int i = 0; i < (HEAD_BUF_LEN + USB_CMD_CHASSIS_DATA_LEN); i++) {
        sum_check += usb_txbuffer[i];
        addr_check += sum_check;
    }
    usb_txbuffer[offset++] = sum_check & 0xFF;
    usb_txbuffer[offset] = addr_check & 0xFF;

    // 发送数据，检查返回值确保发送成功
    if (CDC_Transmit_HS(usb_txbuffer, CMD_CHASSIS_BUF_LEN) != USBD_OK) {

    }
}

uint8_t Rx_data[64];
uint8_t Data_len=0;
uint8_t num = 0;
int32_t sum_check = 0;
int32_t addr_check = 0;
void UnpackPCData(void)
{
    uint8_t rx_data_state = 0;
    for(num = 0; num < USB_Received_Len ;num++)
    {
        if(USB_Received_Data[num] == 0xFF && rx_data_state == 0) //等待帧头
        {
            set_data(USB_Received_Data[num]);
            Data_len++;
            rx_data_state=1;
        }
        else if(USB_Received_Data[num] == 0x05 && rx_data_state == 1) //等待地址
        {
            set_data(USB_Received_Data[num]);
            Data_len++;
            rx_data_state=2;
        }
        else if(rx_data_state == 2)
        {
            set_data(USB_Received_Data[num]);
            Data_len++;
            rx_data_state = 3;
        }
        else if(rx_data_state == 3)
        {
            if(Data_len <= Rx_data[2])
            {
                set_data(USB_Received_Data[num]);
            }
            else
            {
                rx_data_state = 4;
            }
        }
        else if(rx_data_state == 4)
        {
            if(sum_check == USB_Received_Data[num])
            {
                rx_data_state = 5;
            }
        }
        else if(rx_data_state == 5)
        {
            if(addr_check == USB_Received_Data[num])
            {

            }
        }
    }
}


void set_data(uint8_t rx_byte)
{
    Rx_data[Data_len]=rx_byte;
    Data_len++;
    sum_check += rx_byte;
    addr_check += sum_check;
}
/* -------------------------------- 线程间通讯数据包相关 ------------------------------- */