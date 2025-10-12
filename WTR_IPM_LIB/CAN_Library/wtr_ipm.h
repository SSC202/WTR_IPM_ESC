/**
 * @file    wtr_ipm.h
 * @brief   IPM-ESC MCU 驱动库
 * @author  S.S.C
 * @version V1.0.0
 * @date    2025-10-11
 * @details 该驱动库仅适用于 WTR IPM-ESC 电子调速器
 *          1. IPM-ESC 推荐适用 CAN/FDCAN 接口进行通信，CAN通信波特率请配置为 500k.
 *          2. 使用 CAN/FDCAN 通信前，强烈建议使用串口调试接口进行校准操作.校准后请不要修改三相线.
 *          3. 建议以 5 Hz 频率以上发送 CAN 消息指令，如果超时将使得电调进入停止状态.
 */
#ifndef __WTR_IPM_H
#define __WTR_IPM_H

#include "can.h"
#include "main.h"

// 命令类型定义
typedef enum
{
    CAN_STOP = 1,         // 停止指令
    CAN_SET_POSITION = 2, // 设置位置指令
    CAN_SET_SPEED = 3,    // 设置速度指令
} CAN_COMMAND_TYPE;

// 接收消息类型定义
typedef struct
{
    float position; // 位置(rad)(绝对位置,自动包含圈数)
    float speed;    // 速度(rad/s)
    uint8_t id;     // 电调 ID
} IPM_ESC_RX_MSG;

/******************************************************************
 *                      用户自定义 CAN/FDCAN 接口
 ******************************************************************/

#define IPM_ESC_CAN_Handler hcan1

/******************************************************************
 *                           用户接口变量
 ******************************************************************/
extern IPM_ESC_RX_MSG ipm_esc_msg; // 接收的 CAN 消息
extern uint8_t ipm_esc_it_flag;    // CAN 消息中断标志

/******************************************************************
 *                           用户接口函数
 ******************************************************************/

void IPM_ESC_CAN_Config(void);                         // CAN 初始化
void IPM_ESC_Set_Speed(float speed, uint8_t id);       // 设置电机速度
void IPM_ESC_Set_Position(float position, uint8_t id); // 设置电机位置
void IPM_ESC_Stop(uint8_t id);                         // 停止电机
float IPM_ESC_Get_Speed(uint8_t id);                   // 获取电机速度
float IPM_ESC_Get_Position(uint8_t id);                // 获取电机位置

float array_2_float(const uint8_t *byte_array);
void float_2_array(float value, uint8_t *byte_array);

// __weak void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//     uint8_t RxData[8];
//     CAN_RxHeaderTypeDef RxHeader;
//     if (hcan->Instance == IPM_ESC_CAN_Handler.Instance) {
//         HAL_CAN_GetRxMessage(&IPM_ESC_CAN_Handler, CAN_RX_FIFO0, &RxHeader, RxData);
//         if (ipm_esc_it_flag == 0) {
//             ipm_esc_it_flag = 1;
//             IPM_ESC_Decode(&ipm_esc_msg, RxHeader.StdId, RxData);
//             ipm_esc_it_flag = 0;
//         }
//     }
// }

#endif
