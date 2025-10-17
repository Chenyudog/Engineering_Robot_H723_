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
#define HEAD_BUF_LEN        4       // 帧头长度（0-3字节:帧头0xFF、地址、命名ID、数据长度）

#define USB_INS_DATA_LEN    36      // 数据部分长度（9个int32_t）
#define INS_BUF_LEN         42      // 总长度：4(帧头)+36(数据)+2(校验)

#define USB_CMD_CHASSIS_DATA_LEN    12      // 数据部分长度（3个int32_t）
#define CMD_CHASSIS_BUF_LEN         18      // 总长度：4(帧头)+12(数据)+2(校验)


/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
static struct ins_msg transmission_subscribe_ins_data;
static struct cmd_chassis_msg transmission_subscribe_cmd_chassis_data;
static struct cmd_chassis_msg receive_pc_cmd_chassis_data;

static subscriber_t *subscribe_ins_topic;
static subscriber_t *subscribe_cmd_chassis_topic;

static uint8_t usb_txbuffer[MAX_USB_BUF_LEN] = {0};

extern uint8_t USB_Received_Data[APP_RX_DATA_SIZE];//接收usb数据缓冲区
extern uint32_t USB_Received_Len;
extern volatile uint8_t USB_Data_Ready_Flag;


static uint8_t Rx_data[64];
static uint8_t Data_len = 0;
static uint32_t num = 0;
static uint32_t sum_check = 0;
static uint32_t addr_check = 0;


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
            USB_Received_Len = 0;
            USB_Data_Ready_Flag = 0;
        }
        //发送数据给上位机
        InsDataPack();
        vTaskDelay(1);
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
void uppack_pc_cmd_chassis_data(void)
{
    receive_pc_cmd_chassis_data.vx = (float)((Rx_data[4] << 0) |  // byte3占 31-24位
                                             (Rx_data[5] << 8) |  // byte2占 23-16位
                                             (Rx_data[6] << 16)  |  // byte1占 15-8位
                                             (Rx_data[7] << 24)) / 10000.0f;    // byte0占 7-0位
    receive_pc_cmd_chassis_data.vy = (float)((Rx_data[8] << 0) |  // byte3占 31-24位
                                             (Rx_data[9] << 8) |  // byte2占 23-16位
                                             (Rx_data[10] << 16)  |  // byte1占 15-8位
                                             (Rx_data[11] << 24)) / 10000.0f;    // byte0占 7-0位
    receive_pc_cmd_chassis_data.vw = (float)((Rx_data[12] << 0) |  // byte3占 31-24位
                                             (Rx_data[13] << 8) |  // byte2占 23-16位
                                             (Rx_data[14] << 16)  |  // byte1占 15-8位
                                             (Rx_data[15] << 24)) / 10000.0f;    // byte0占 7-0位
}
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
    sum_check = 0;
    addr_check = 0;
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
    int32_t cmd_chassis_vy = (int32_t)(transmission_subscribe_cmd_chassis_data.vy * scale);
    int32_t cmd_chassis_vw = (int32_t)(transmission_subscribe_cmd_chassis_data.vw * scale);

    uint32_t offset = HEAD_BUF_LEN;  // 从帧头后开始
    memcpy(&usb_txbuffer[offset], &cmd_chassis_vx, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &cmd_chassis_vy, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &cmd_chassis_vw, sizeof(int32_t)); offset += sizeof(int32_t);

    sum_check = 0;
    addr_check = 0;
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


void UnpackPCData(void)
{
    uint8_t rx_data_state = 0;
    Data_len = 0;
    sum_check = 0;
    addr_check = 0;
    memset(Rx_data, 0, sizeof(Rx_data));

    for (num = 0; num < USB_Received_Len; num++)
    {
        uint8_t curr_byte = USB_Received_Data[num];

        if (curr_byte == 0xFF && rx_data_state == 0) // 等待帧头
        {
            set_data(curr_byte);
            rx_data_state = 1;
        }
        else if (curr_byte == 0x05 && rx_data_state == 1) // 等待地址
        {
            set_data(curr_byte);
            rx_data_state = 2;
        }
        else if (rx_data_state == 2)
        {
            set_data(curr_byte);
            rx_data_state = 3;
        }
        else if (rx_data_state == 3)
        {
            if (Data_len == 3)
            {
                set_data(curr_byte);
            }
            else
            {
                // 已接收数据字节数 = 总接收字节数 - 帧头长度（4字节）
                if ((Data_len - HEAD_BUF_LEN) <= Rx_data[3])
                {
                    set_data(curr_byte);
                }
                // 数据段接收完成，进入校验阶段
                if ((Data_len - HEAD_BUF_LEN) == Rx_data[3])
                {
                    rx_data_state = 4;
                }
            }
            if (Data_len >= sizeof(Rx_data))
            {
                rx_data_state = 0;
                Data_len = 0;
                sum_check = 0;
                addr_check = 0;
            }
        }
        else if (rx_data_state == 4) // 校验sum_check
        {
            if ((sum_check & 0xFF) == curr_byte)
            {
                rx_data_state = 5;
            }
            else
            {
                rx_data_state = 0;
                Data_len = 0;
                sum_check = 0;
                addr_check = 0;
            }
        }
        else if (rx_data_state == 5) // 校验addr_check
        {
            if ((addr_check & 0xFF) == curr_byte)
            {
                rx_data_state = 0;
                Data_len = 0;
                sum_check = 0;
                addr_check = 0;
                if(Rx_data[2] == 0x12)
                {
                        uppack_pc_cmd_chassis_data();
                }


            }
            else
            {
                rx_data_state = 0;
                Data_len = 0;
                sum_check = 0;
                addr_check = 0;
            }
        }
        else
        {
            rx_data_state = 0;
            Data_len = 0;
            sum_check = 0;
            addr_check = 0;
        }
    }
}


void set_data(uint8_t rx_byte)
{
    if (Data_len < sizeof(Rx_data))
    {
        Rx_data[Data_len] = rx_byte;
        Data_len++;
        sum_check += rx_byte;
        addr_check += sum_check;
    }
    else
    {
        Data_len = 0;
        sum_check = 0;
        addr_check = 0;
    }
}
/* -------------------------------- 线程间通讯数据包相关 ------------------------------- */