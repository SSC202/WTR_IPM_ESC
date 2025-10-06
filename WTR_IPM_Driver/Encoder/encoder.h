#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32g4xx.h"

#include "mt6701.h"

/**
 * @brief   编码器数值编号定义
 */
typedef enum Encoder_Type {
    MT6701, // MT6701
    AS5600, // AS5600
}Encoder_Type;
typedef struct {
    enum Encoder_Type encoder_type; // 编码器类型(数值编号)

    float encoder_offset; // 编码器偏移量
    uint8_t pole_pairs;   // 电机极对数
    int encoder_direct;   // 编码器方向

    float curr_encoder_theta;  // 电机当前机械角度
    float last_encoder_theta;  // 电机前一机械角度
    float encoder_theta_diff;  // 电机角度差值
    float electric_theta;      // 电机电角度
    float encoder_speed;       // 电机机械速度
    float encoder_total_theta; // 电机累计机械角度(上电归零的相对位置)
} Encoder_t;

void Encoder_Init(Encoder_t *encoder, uint8_t pole_pairs, int encoder_direct, Encoder_Type encoder_type, float offset);
void Encoder_Get_Angle_Speed(Encoder_t *encoder);

#endif // __ENCODER_H