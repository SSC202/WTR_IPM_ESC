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
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType       = FDCAN_STANDARD_ID;       // 只接收标准帧ID
    sFilterConfig.FilterIndex  = 0;                       // 滤波器索引0
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;       // 滤波器类型
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 滤波器关联到RXFIFO0
    sFilterConfig.FilterID1    = 0x00;                    // 滤波ID1 0x00
    sFilterConfig.FilterID2    = 0x00;                    // 滤波ID2 0x00

    if (HAL_FDCAN_ConfigFilter(&IPM_ESC_CAN_Handler, &sFilterConfig) != HAL_OK) {
        while (1) {
            ;
        }
    }

    /**
     * @note    设置滤波器全局配置
     *          设置标准帧ID，接收的报文ID没有匹配上滤波器时，选择拒绝接收(没有匹配上时,可以选择放入FIFO0或者FIFO1)。
     *          设置拓展帧ID，接收的报文ID没有匹配上滤波器时，选择拒绝接收。
     *          设置是否拒绝远程标准帧，ENABLE代表拒绝接收。
     *          设置是否拒绝远程拓展帧，ENABLE代表拒绝接收。
     */
    if (HAL_FDCAN_ConfigGlobalFilter(&IPM_ESC_CAN_Handler, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE) != HAL_OK) /* 设置FDCAN1滤波器0全局配置  */
    {
        while (1) {
            ;
        }
    }

    if (HAL_FDCAN_Start(&IPM_ESC_CAN_Handler) != HAL_OK) {
        while (1) {
            ;
        }
    }

    if (HAL_FDCAN_ActivateNotification(&IPM_ESC_CAN_Handler, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        while (1) {
            ;
        }
    }
    if (HAL_FDCAN_ActivateNotification(&IPM_ESC_CAN_Handler, FDCAN_IT_RX_FIFO0_FULL, 0) != HAL_OK) {
        while (1) {
            ;
        }
    }
}

/**
 * @brief   发送 CAN 消息
 * @param   msg     8 字节 CAN 消息
 * @param   id      CAN 消息 ID
 */
static uint8_t CAN_Send_Msg(uint8_t *msg, uint8_t id)
{
    uint8_t message[8];
    FDCAN_TxHeaderTypeDef sTxHeader;

    sTxHeader.Identifier          = id;                 // 32位ID
    sTxHeader.IdType              = FDCAN_STANDARD_ID;  // 标准ID
    sTxHeader.TxFrameType         = FDCAN_DATA_FRAME;   // 数据帧
    sTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // ESI位
    sTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;      // 关闭速率转换
    sTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;  // 标准CAN模式
    sTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 无发送事件
    sTxHeader.MessageMarker       = 0;
    sTxHeader.TxFrameType         = FDCAN_DATA_FRAME;

    // 装载数据
    for (uint8_t i = 0; i < 8; i++) {
        message[i] = msg[i];
    }

    // 将需要发送的数据压入到TX FIFO
    if (HAL_FDCAN_AddMessageToTxFifoQ(&IPM_ESC_CAN_Handler, &sTxHeader, message) == HAL_OK) {
        return 1;
    } else {
        return 0;
    }
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
__weak void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint8_t RxData[8];
    FDCAN_RxHeaderTypeDef sRxHeader;
    if (hfdcan->Instance == IPM_ESC_CAN_Handler.Instance && (RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) {
        HAL_FDCAN_GetRxMessage(&IPM_ESC_CAN_Handler, FDCAN_RX_FIFO0, &sRxHeader, RxData);
        if (ipm_esc_it_flag == 0) {
            ipm_esc_it_flag = 1;
            IPM_ESC_Decode(&ipm_esc_msg, sRxHeader.Identifier, RxData);
            ipm_esc_it_flag = 0;
        }
    }
}
