#ifndef __FLASH_SAVE_H
#define __FLASH_SAVE_H

#include "stm32g4xx.h"
#include "encoder.h"

#define FLASH_STORAGE_ADDR 0x0801F800 // Page 127起始地址
#define FLASH_DATA_KEY     0xABCD1234 // 数据有效性标识

#pragma pack(push, 1)
typedef struct {
    uint32_t key; // 数据有效性标识
    float position_kp;
    float position_ki;
    float position_kd;
    float position_pid_maxoutput;
    float speed_pi_kp;
    float speed_pi_ki;
    float speed_pi_maxoutput;
    float f_c;
    float id_pi_kp;
    float id_pi_ki;
    float iq_pi_kp;
    float iq_pi_ki;
    uint8_t pole_pairs;
    int encoder_direct;
    float encoder_offset;
    Encoder_Type encoder_type;
    uint8_t id;
    float u_dc;
    uint32_t checksum; // 校验和
} FlashData_t;
#pragma pack(pop)

extern float position_pid_kp;        // 电机位置 PID kp 值
extern float position_pid_ki;        // 电机位置 PID ki 值
extern float position_pid_kd;        // 电机位置 PID kd 值
extern float position_pid_maxoutput; // 电机位置 PID 最大输出

extern float speed_pi_kp;        // 电机速度 PI 参数 P
extern float speed_pi_ki;        // 电机速度 PI 参数 I
extern float speed_pi_maxoutput; // 电机速度 PI 最大输出

extern float f_c;      // 电流滤波器截止频率(Hz)
extern float id_pi_kp; // d 轴电流 PI 参数 P
extern float id_pi_ki; // d 轴电流 PI 参数 I
extern float iq_pi_kp; // q 轴电流 PI 参数 P
extern float iq_pi_ki; // q 轴电流 PI 参数 I

extern uint8_t pole_pairs;        // 电机极对数
extern int encoder_direct;        // 编码器方向
extern float encoder_offset;      // 编码器机械零位偏移
extern Encoder_Type encoder_type; // 编码器类型

extern float u_dc; // 直流母线电压

extern uint8_t id; // ESC-IPM ID

void Flash_Init(void);
HAL_StatusTypeDef Flash_Save(float pos_kp, float pos_ki, float pos_kd, float pos_max, float spd_kp, float spd_ki, float spd_max, float fc, float id_kp, float id_ki, float iq_kp, float iq_ki, uint8_t pps, int dir, float offset, Encoder_Type type, uint8_t _id, uint8_t udc);
void Flash_Read(float *pos_kp, float *pos_ki, float *pos_kd, float *pos_max, float *spd_kp, float *spd_ki, float *spd_max, float *fc, float *id_kp, float *id_ki, float *iq_kp, float *iq_ki, uint8_t *pps, int *dir, float *offset, Encoder_Type *type, uint8_t *_id, uint8_t *udc);
uint8_t Flash_IsDataValid(void);

#endif