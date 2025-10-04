#include "usermain.h"

/**
 * @brief   系统全局变量
 */
const float system_sample_time = 0.0001f; // 系统采样频率 10kHz

enum SYSTEM_STATE {
    SYSTEM_INIT,  // 初始化
    SYSTEM_STOP,  // 停止
    SYSTEM_FAULT, // 故障
    SYSTEM_RUN    // 运行
};
enum SYSTEM_STATE system_state = SYSTEM_INIT; // 系统状态

enum SYSTEM_SAMPLE_STATE {
    SAMPLE_INIT, // 采样初始化
    SAMPLE_RUN   // 采样运行
};
enum SYSTEM_SAMPLE_STATE system_sample_state = SAMPLE_INIT; // 采样状态

// FOC 相关变量
float u_dc = 12.0f;  // 直流母线电压
Encoder_t encoder;   // 编码器
PID_t speed_pi;      // 电机速度 PI 控制器
abc_t i_abc;         // 电机三相电流
dq_t i_dq;           // 电机 dq 轴电流
LPF_t id_lpf;        // 电机 d 轴电流滤波器
LPF_t iq_lpf;        // 电机 q 轴电流滤波器
dq_t i_dql;          // 电机 dq 轴低频电流
PID_t id_pi;         // 电机 d 轴电流 PI 控制器
PID_t iq_pi;         // 电机 q 轴电流 PI 控制器
dq_t u_dq;           // dq 轴指令电压
abc_t u_abc;         // 三相指令电压
duty_abc_t duty_abc; // 三相占空比

/**
 * @brief   调试者临时变量
 */
float speed_ref = 0; // 电机速度目标值

/**
 * @brief   系统初始化函数
 */
static void init(void)
{
    // 控制器/滤波器初始化
    LPF_Init(&id_lpf, 100, system_sample_time);
    LPF_Init(&iq_lpf, 100, system_sample_time);
    PID_Init(&id_pi, 0.5, 20, 0, u_dc / M_SQRT3);
    PID_Init(&iq_pi, 0.5, 20, 0, u_dc / M_SQRT3);
    PID_Init(&speed_pi, 0.1, 0.5, 0, 1);
    // 编码器初始化
    Encoder_Init(&encoder, 7, 1, 1.400f);
    // 电流采样校准
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    HAL_TIM_Base_Start(&htim1);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    while (system_sample_state == SAMPLE_INIT) {
        ;
    }
    system_state = SYSTEM_STOP;
    // 使能 PWM 输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief   系统主函数
 */
void usermain(void)
{
    init();
    while (1) {
        printf("speed_pi: %.5f,%.5f\r\n", speed_pi.ref, speed_pi.fdb);
    }
}

/**
 * @brief   ADC 注入通道采样回调函数
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /**
     * @brief   三相电流采样
     */
    static uint32_t adc_cnt = 0;

    static uint32_t adc_ia_offset_sum = 0;
    static uint32_t adc_ia            = 0;
    static float adc_ia_offset        = 0;

    static uint32_t adc_ib_offset_sum = 0;
    static uint32_t adc_ib            = 0;
    static float adc_ib_offset        = 0;

    static uint32_t adc_ic_offset_sum = 0;
    static uint32_t adc_ic            = 0;
    static float adc_ic_offset        = 0;

    if (hadc->Instance == ADC1) {
        if (system_sample_state == SAMPLE_INIT) {
            adc_cnt++;
            if (adc_cnt >= 1) {
                adc_ia_offset_sum += hadc1.Instance->JDR1;
                adc_ib_offset_sum += hadc1.Instance->JDR2;
                adc_ic_offset_sum += hadc1.Instance->JDR3;
            }
            if (adc_cnt == 1000) {
                adc_ia_offset       = adc_ia_offset_sum / 1000.0f;
                adc_ib_offset       = adc_ib_offset_sum / 1000.0f;
                adc_ic_offset       = adc_ic_offset_sum / 1000.0f;
                system_sample_state = SAMPLE_RUN;
                adc_ia_offset_sum   = 0;
                adc_ib_offset_sum   = 0;
                adc_ic_offset_sum   = 0;
                adc_cnt             = 0;
            }
        } else {
            adc_ia  = hadc1.Instance->JDR1;
            i_abc.a = ((((float)adc_ia - adc_ia_offset) / 4096.f) * 3.3f) * 20.0f;

            adc_ib  = hadc1.Instance->JDR2;
            i_abc.b = ((((float)adc_ib - adc_ib_offset) / 4096.f) * 3.3f) * 20.0f;

            adc_ic  = hadc1.Instance->JDR3;
            i_abc.c = ((((float)adc_ic - adc_ic_offset) / 4096.f) * 3.3f) * 20.0f;

            // 过流保护
            if ((i_abc.a * i_abc.a > 64) || (i_abc.b * i_abc.b > 64) || (i_abc.c * i_abc.c > 64)) {
                system_state = SYSTEM_FAULT;
            }
        }
    }

    /**
     * @brief   编码器采样
     */
    Encoder_Get_Angle_Speed(&encoder);

    /**
     * @brief   FOC 闭环
     */

    // speed PI Controller
    speed_pi.ref = speed_ref;
    speed_pi.fdb = encoder.encoder_speed;
    if (system_state == SYSTEM_RUN) {
        PID_Calc(&speed_pi, 1, system_sample_time);
    } else {
        PID_Calc(&speed_pi, 0, system_sample_time);
    }
    // abc-to-dq
    abc_2_dq(&i_abc, &i_dq, encoder.electric_theta);

    // dq current LPF
    id_lpf.input = i_dq.d;
    LPF_Calc(&id_lpf, 1);
    i_dql.d = id_lpf.output;

    iq_lpf.input = i_dq.q;
    LPF_Calc(&iq_lpf, 1);
    i_dql.q = iq_lpf.output;

    // (id=0 control)Current PI Controller
    // d-axis
    id_pi.ref = 0;
    id_pi.fdb = i_dql.d;
    if (system_state == SYSTEM_RUN) {
        PID_Calc(&id_pi, 1, system_sample_time);
    } else {
        PID_Calc(&id_pi, 0, system_sample_time);
    }
    u_dq.d = id_pi.output;

    // q-axis
    iq_pi.ref = speed_pi.output;
    iq_pi.fdb = i_dql.q;
    if (system_state == SYSTEM_RUN) {
        PID_Calc(&iq_pi, 1, system_sample_time);
    } else {
        PID_Calc(&iq_pi, 0, system_sample_time);
    }
    u_dq.q = iq_pi.output;

    // dq-to-abc
    dq_2_abc(&u_dq, &u_abc, encoder.electric_theta);

    // SVPWM
    e_svpwm(&u_abc, u_dc, &duty_abc);

    /**
     * @brief   三相 PWM 输出
     */
    if (system_state == SYSTEM_RUN) {
        TIM1->CCR1 = duty_abc.dutya * TIM1->ARR;
        TIM1->CCR2 = duty_abc.dutyb * TIM1->ARR;
        TIM1->CCR3 = duty_abc.dutyc * TIM1->ARR;
    } else if (system_state == SYSTEM_STOP || system_state == SYSTEM_FAULT) {
        TIM1->CCR1 = 0 * TIM1->ARR;
        TIM1->CCR2 = 0 * TIM1->ARR;
        TIM1->CCR3 = 0 * TIM1->ARR;
    }
}