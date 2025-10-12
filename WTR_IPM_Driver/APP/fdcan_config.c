#include "fdcan_config.h"
#include "float2array.h"

/**
 * @brief   FDCAN 初始化函数
 */
void FDCAN_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        while (1) {
            ;
        }
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_FULL, 0) != HAL_OK) {
        while (1) {
            ;
        }
    }

    sFilterConfig.IdType       = FDCAN_STANDARD_ID;       // 只接收标准帧ID
    sFilterConfig.FilterIndex  = 0;                       // 滤波器索引0
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;       // 滤波器类型
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 滤波器关联到RXFIFO0
    sFilterConfig.FilterID1    = 0x00;                    // 滤波ID1 0x00
    sFilterConfig.FilterID2    = 0x00;                    // 滤波ID2 0x00

    // 设置失败进入死循环
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
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
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE) != HAL_OK) /* 设置FDCAN1滤波器0全局配置  */
    {
        while (1) {
            ;
        }
    }

    HAL_FDCAN_Start(&hfdcan1);
}

/**
 * @brief   FDCAN 发送函数
 */
uint8_t FDCAN_Send_Msg(CAN_MSG *msg)
{
    FDCAN_TxHeaderTypeDef sTxHeader; // FDCAN1 发送处理单元句柄
    // 选择数据的长度
    switch (msg->len) {
        case 0:
            sTxHeader.DataLength = FDCAN_DLC_BYTES_0; /* 数据长度:0 */
            break;
        case 1:
            sTxHeader.DataLength = FDCAN_DLC_BYTES_1; /* 数据长度:1 */
            break;
        case 2:
            sTxHeader.DataLength = FDCAN_DLC_BYTES_2; /* 数据长度:2 */
            break;
        case 3:
            sTxHeader.DataLength = FDCAN_DLC_BYTES_3; /* 数据长度:3 */
            break;
        case 4:
            sTxHeader.DataLength = FDCAN_DLC_BYTES_4; /* 数据长度:4 */
            break;
        case 5:
            sTxHeader.DataLength = FDCAN_DLC_BYTES_5; /* 数据长度:5 */
            break;
        case 6:
            sTxHeader.DataLength = FDCAN_DLC_BYTES_6; /* 数据长度:6 */
            break;
        case 7:
            sTxHeader.DataLength = FDCAN_DLC_BYTES_7; /* 数据长度:7 */
            break;
        case 8:
            sTxHeader.DataLength = FDCAN_DLC_BYTES_8; /* 数据长度:8 */
            break;
        default:
            return 0;
    }

    sTxHeader.Identifier          = msg->id;            // 32位ID
    sTxHeader.IdType              = FDCAN_STANDARD_ID;  // 标准ID
    sTxHeader.TxFrameType         = FDCAN_DATA_FRAME;   // 数据帧
    sTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // ESI位
    sTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;      // 关闭速率转换
    sTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;  // 标准CAN模式
    sTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 无发送事件
    sTxHeader.MessageMarker       = 0;

    if (REMOTE_FRAME == msg->rtr)
        sTxHeader.TxFrameType = FDCAN_REMOTE_FRAME; // 远程帧
    else
        sTxHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据帧

    // 将需要发送的数据压入到TX FIFO
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sTxHeader, (uint8_t *)msg->buffer) == HAL_OK) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief   FDCAN 消息解码函数
 */
void FDCAN_Msg_Decode(uint8_t *buf, CAN_RX_MSG *msg)
{
    static uint8_t float_array[4];
    for (int i = 0; i < 4; i++) {
        float_array[i] = buf[i + 1];
    }
    msg->value   = array_2_float(float_array);
    msg->command = (CAN_COMMAND_TYPE)buf[0];
}