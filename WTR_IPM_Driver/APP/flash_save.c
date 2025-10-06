#include "flash_save.h"

float position_pid_kp        = 20;  // 电机位置 PID kp 值
float position_pid_ki        = 20;  // 电机位置 PID ki 值
float position_pid_kd        = 0.1; // 电机位置 PID kd 值
float position_pid_maxoutput = 400; // 电机位置 PID 最大输出

float speed_pi_kp        = 0.1; // 电机速度 PI 参数 P
float speed_pi_ki        = 0.5; // 电机速度 PI 参数 I
float speed_pi_maxoutput = 1;   // 电机速度 PI 最大输出

float f_c      = 100; // 电流滤波器截止频率(Hz)
float id_pi_kp = 0.5; // d 轴电流 PI 参数 P
float id_pi_ki = 20;  // d 轴电流 PI 参数 I
float iq_pi_kp = 0.5; // q 轴电流 PI 参数 P
float iq_pi_ki = 20;  // q 轴电流 PI 参数 I

uint8_t pole_pairs        = 7;      // 电机极对数
int encoder_direct        = 1;      // 编码器方向
float encoder_offset      = 1.400f; // 编码器机械零位偏移
Encoder_Type encoder_type = MT6701; // 编码器类型

/**
 * @brief   计算校验和
 */
static uint32_t CalculateChecksum(FlashData_t *data)
{
    uint32_t sum     = 0;
    uint8_t *bytePtr = (uint8_t *)&data->position_kp;

    size_t dataSize = (uint8_t *)&data->encoder_type - (uint8_t *)&data->position_kp + sizeof(Encoder_Type);

    for (size_t i = 0; i < dataSize; i++) {
        sum += bytePtr[i];
    }
    return sum;
}
/**
 * @brief   初始化 FLASH
 */
void Flash_Init(void)
{
    Flash_Read(&position_pid_kp, &position_pid_ki, &position_pid_kd, &position_pid_maxoutput, &speed_pi_kp, &speed_pi_ki, &speed_pi_maxoutput, &f_c, &id_pi_kp, &id_pi_ki, &iq_pi_kp, &iq_pi_ki, &pole_pairs, &encoder_direct, &encoder_offset, &encoder_type);

    if (!Flash_IsDataValid()) {
        position_pid_kp        = 20;  // 电机位置 PID kp 值
        position_pid_ki        = 20;  // 电机位置 PID ki 值
        position_pid_kd        = 0.1; // 电机位置 PID kd 值
        position_pid_maxoutput = 400; // 电机位置 PID 最大输出

        speed_pi_kp        = 0.1; // 电机速度 PI 参数 P
        speed_pi_ki        = 0.5; // 电机速度 PI 参数 I
        speed_pi_maxoutput = 1;   // 电机速度 PI 最大输出

        f_c      = 100; // 电流滤波器截止频率(Hz)
        id_pi_kp = 0.5; // d 轴电流 PI 参数 P
        id_pi_ki = 20;  // d 轴电流 PI 参数 I
        iq_pi_kp = 0.5; // q 轴电流 PI 参数 P
        iq_pi_ki = 20;  // q 轴电流 PI 参数 I
        Flash_Save(position_pid_kp, position_pid_ki, position_pid_kd, position_pid_maxoutput, speed_pi_kp, speed_pi_ki, speed_pi_maxoutput, f_c, id_pi_kp, id_pi_ki, iq_pi_kp, iq_pi_ki, pole_pairs, encoder_direct, encoder_offset, encoder_type);
    }
}

/**
 * @brief   检查数据有效性
 */
uint8_t Flash_IsDataValid(void)
{
    FlashData_t *flashData = (FlashData_t *)FLASH_STORAGE_ADDR;

    if (flashData->key != FLASH_DATA_KEY) {
        return 0;
    }

    uint32_t calcChecksum = CalculateChecksum(flashData);
    if (flashData->checksum != calcChecksum) {
        return 0;
    }

    return 1;
}

/**
 * @brief   读取 FLASH 数据
 */
void Flash_Read(float *pos_kp, float *pos_ki, float *pos_kd, float *pos_max, float *spd_kp, float *spd_ki, float *spd_max, float *fc, float *id_kp, float *id_ki, float *iq_kp, float *iq_ki, uint8_t *pps, int *dir, float *offset, Encoder_Type *type)
{
    if (Flash_IsDataValid()) {
        FlashData_t *flashData = (FlashData_t *)FLASH_STORAGE_ADDR;
        *pos_kp                = flashData->position_kp;
        *pos_ki                = flashData->position_ki;
        *pos_kd                = flashData->position_kd;
        *pos_max               = flashData->position_pid_maxoutput;
        *spd_kp                = flashData->speed_pi_kp;
        *spd_ki                = flashData->speed_pi_ki;
        *spd_max               = flashData->speed_pi_maxoutput;
        *fc                    = flashData->f_c;
        *id_kp                 = flashData->id_pi_kp;
        *id_ki                 = flashData->id_pi_ki;
        *iq_kp                 = flashData->iq_pi_kp;
        *iq_ki                 = flashData->iq_pi_ki;
        *pps                   = flashData->pole_pairs;
        *dir                   = flashData->encoder_direct;
        *offset                = flashData->encoder_offset;
        *type                  = flashData->encoder_type;
    } else {
        *pos_kp  = 0.0f;
        *pos_ki  = 0.0f;
        *pos_kd  = 0.0f;
        *pos_max = 0.0f;
        *spd_kp  = 0.0f;
        *spd_ki  = 0.0f;
        *spd_max = 0.0f;
        *fc      = 0.0f;
        *id_kp   = 0.0f;
        *id_ki   = 0.0f;
        *iq_kp   = 0.0f;
        *iq_ki   = 0.0f;
        *pps     = 1;
        *dir     = 1;
        *offset  = 0.0f;
        *type    = MT6701;
    }
}

/**
 * @brief   保存 FLASH 数据
 */
HAL_StatusTypeDef Flash_Save(float pos_kp, float pos_ki, float pos_kd, float pos_max, float spd_kp, float spd_ki, float spd_max, float fc, float id_kp, float id_ki, float iq_kp, float iq_ki, uint8_t pps, int dir, float offset, Encoder_Type type)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;

    // 准备要写入的数据
    FlashData_t newData;
    newData.key                    = FLASH_DATA_KEY;
    newData.position_kp            = pos_kp;
    newData.position_ki            = pos_ki;
    newData.position_kd            = pos_kd;
    newData.position_pid_maxoutput = pos_max;
    newData.speed_pi_kp            = spd_kp;
    newData.speed_pi_ki            = spd_ki;
    newData.speed_pi_maxoutput     = spd_max;
    newData.f_c                    = fc;
    newData.id_pi_kp               = id_kp;
    newData.id_pi_ki               = id_ki;
    newData.iq_pi_kp               = iq_kp;
    newData.iq_pi_ki               = iq_ki;
    newData.pole_pairs             = pps;
    newData.encoder_direct         = dir;
    newData.encoder_offset         = offset;
    newData.encoder_type           = type;
    newData.checksum               = CalculateChecksum(&newData);

    // 解锁FLASH
    HAL_FLASH_Unlock();

    // 配置擦除参数
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.Banks     = FLASH_BANK_1;
    eraseInit.Page      = 127;
    eraseInit.NbPages   = 1;

    // 擦除FLASH页
    status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    // 使用双字编程写入数据 - STM32G4系列
    uint64_t *dataPtr       = (uint64_t *)&newData;
    uint32_t dataSizeDwords = (sizeof(FlashData_t) + 7) / 8; // 计算需要多少个双字

    for (uint32_t i = 0; i < dataSizeDwords; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   FLASH_STORAGE_ADDR + (i * 8),
                                   dataPtr[i]);
        if (status != HAL_OK) {
            break;
        }
    }

    // 锁定FLASH
    HAL_FLASH_Lock();

    // 更新全局变量
    if (status == HAL_OK) {
        position_pid_kp        = pos_kp;
        position_pid_ki        = pos_ki;
        position_pid_kd        = pos_kd;
        position_pid_maxoutput = pos_max;
        speed_pi_kp            = spd_kp;
        speed_pi_ki            = spd_ki;
        speed_pi_maxoutput     = spd_max;
        f_c                    = fc;
        id_pi_kp               = id_kp;
        id_pi_ki               = id_ki;
        iq_pi_kp               = iq_kp;
        iq_pi_ki               = iq_ki;
        pole_pairs             = pps;
        encoder_offset         = offset;
        encoder_direct         = dir;
        encoder_type           = type;
    }

    return status;
}