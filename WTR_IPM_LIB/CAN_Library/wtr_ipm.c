#include "wtr_ipm.h"
#include <string.h>
#include <math.h>

IPM_ESC_RX_MSG ipm_esc_msg;  // 接收的 CAN 消息
uint8_t ipm_esc_it_flag = 0; // CAN 消息中断标志

/******************************************************************
 *                       float-array 转换
 ******************************************************************/

/**
 * @brief  将 float 类型数据转换为字节数组
 * @param  value        float 类型数据
 * @param  byte_array   字节数组
 */
void float_2_array(float value, uint8_t *byte_array)
{
    uint32_t int_value;
    memcpy(&int_value, &value, sizeof(float));

    byte_array[0] = (int_value >> 24) & 0xFF;
    byte_array[1] = (int_value >> 16) & 0xFF;
    byte_array[2] = (int_value >> 8) & 0xFF;
    byte_array[3] = int_value & 0xFF;
}

/**
 * @brief  将字节数组转换为 float 类型数据
 * @param  byte_array   字节数组
 * @return float 类型数据
 */
float array_2_float(const uint8_t *byte_array)
{
    uint32_t int_value = 0;
    float result;

    int_value |= ((uint32_t)byte_array[0] << 24);
    int_value |= ((uint32_t)byte_array[1] << 16);
    int_value |= ((uint32_t)byte_array[2] << 8);
    int_value |= (uint32_t)byte_array[3];

    memcpy(&result, &int_value, sizeof(float));
    return result;
}

/******************************************************************
 *                       CAN 底层接口
 ******************************************************************/

/**
 * @brief  CAN 初始化
 */
void IPM_ESC_CAN_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&IPM_ESC_CAN_Handler, &sFilterConfig) != HAL_OK) {
    }

    if (HAL_CAN_Start(&IPM_ESC_CAN_Handler) != HAL_OK) {
    }

    if (HAL_CAN_ActivateNotification(&IPM_ESC_CAN_Handler, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    }
}

/**
 * @brief   发送 CAN 消息
 * @param   msg     8 字节 CAN 消息
 * @param   id      CAN 消息 ID
 */
static uint8_t CAN_Send_Msg(uint8_t *msg, uint8_t id)
{
    uint8_t i = 0;
    uint8_t message[8];
    uint32_t TxMailbox;
    CAN_TxHeaderTypeDef CAN_TxHeader;

    // 设置发送参数
    CAN_TxHeader.StdId              = id;           // 标准ID
    CAN_TxHeader.ExtId              = 0x00;         // 扩展ID
    CAN_TxHeader.IDE                = CAN_ID_STD;   // 标准帧
    CAN_TxHeader.RTR                = CAN_RTR_DATA; // 数据帧
    CAN_TxHeader.DLC                = 8;
    CAN_TxHeader.TransmitGlobalTime = DISABLE;

    // 装载数据
    for (i = 0; i < 8; i++) {
        message[i] = msg[i];
    }

    // 发送CAN消息
    if (HAL_CAN_AddTxMessage(&IPM_ESC_CAN_Handler, &CAN_TxHeader, message, &TxMailbox) != HAL_OK) {
        return 1;
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(&IPM_ESC_CAN_Handler) != 3) {
    }
    return 0;
}

/******************************************************************
 *                         用户接口
 ******************************************************************/

/**
 * @brief   设置电机速度
 * @param   speed   目标速度(rad/s)
 * @param   id      电调 ID
 */
void IPM_ESC_Set_Speed(float speed, uint8_t id)
{
    uint8_t msg[8]         = {0};
    uint8_t float_array[4] = {0};

    msg[0] = CAN_SET_SPEED; // 设置速度指令

    float_2_array(speed, float_array);
    for (int i = 0; i < 4; i++) {
        msg[i + 1] = float_array[i];
    }

    CAN_Send_Msg(msg, id);
}

/**
 * @brief   设置电机位置
 * @param   position    目标位置(rad)
 * @param   id          电调 ID
 */
void IPM_ESC_Set_Position(float position, uint8_t id)
{
    uint8_t msg[8]         = {0};
    uint8_t float_array[4] = {0};

    msg[0] = CAN_SET_POSITION; // 设置速度指令

    float_2_array(position, float_array);
    for (int i = 0; i < 4; i++) {
        msg[i + 1] = float_array[i];
    }

    CAN_Send_Msg(msg, id);
}

/**
 * @brief   停止电机
 * @param   id          电调 ID
 */
void IPM_ESC_Stop(uint8_t id)
{
    uint8_t msg[8] = {0};

    msg[0] = CAN_STOP; // 停止指令

    CAN_Send_Msg(msg, id);
}

/**
 * @brief   电调信息解码
 */
float IPM_ESC_Decode(IPM_ESC_RX_MSG *rx_msg, uint8_t _id, uint8_t *data)
{
    rx_msg->id              = _id;
    uint8_t float_array1[4] = {0};
    uint8_t float_array2[4] = {0};

    for (int i = 0; i < 4; i++) {
        float_array1[i] = data[i];
        float_array2[i] = data[i + 4];
    }
    rx_msg->position = array_2_float(float_array1);
    rx_msg->speed    = array_2_float(float_array2);
}

/**
 * @brief   获取电机位置
 * @param   id          电调 ID
 * @return  电机位置(rad)(绝对位置,自动包含圈数)
 */
float IPM_ESC_Get_Position(uint8_t id)
{
    if (ipm_esc_msg.id != id) {
        return NAN;
    } // ID 不匹配直接返回无效

    return ipm_esc_msg.position;
}

/**
 * @brief   获取电机速度
 * @param   id          电调 ID
 * @return  电机速度(rad/s)
 */
float IPM_ESC_Get_Speed(uint8_t id)
{
    if (ipm_esc_msg.id != id) {
        return NAN;
    } // ID 不匹配直接返回无效

    return ipm_esc_msg.speed;
}

/**
 * @brief   CAN 接收中断回调函数
 */
__weak void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t RxData[8];
    CAN_RxHeaderTypeDef RxHeader;
    if (hcan->Instance == IPM_ESC_CAN_Handler.Instance) {
        HAL_CAN_GetRxMessage(&IPM_ESC_CAN_Handler, CAN_RX_FIFO0, &RxHeader, RxData);
        if (ipm_esc_it_flag == 0) {
            ipm_esc_it_flag = 1;
            IPM_ESC_Decode(&ipm_esc_msg, RxHeader.StdId, RxData);
            ipm_esc_it_flag = 0;
        }
    }
}
