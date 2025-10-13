#include "usermain.h"

/**
 * @brief   系统全局变量
 */
const float system_sample_time = 0.0001f; // 系统采样频率 10kHz

enum SYSTEM_SAMPLE_STATE {
    SAMPLE_INIT, // 采样初始化
    SAMPLE_RUN   // 采样运行
};
enum SYSTEM_SAMPLE_STATE system_sample_state = SAMPLE_INIT; // 采样状态

enum SYSTEM_STATE {
    SYSTEM_INIT,      // 初始化
    SYSTEM_STOP,      // 停止
    SYSTEM_FAULT,     // 故障
    SYSTEM_RUN,       // 运行
    SYSTEM_DEBUG_RUN, // 调试运行
    SYSTEM_TEST       // 参数校准
};
enum SYSTEM_STATE system_state = SYSTEM_INIT; // 系统状态

enum SYSTEM_RUN_STATE {
    SYSTEM_VELOCITY_CTL, // 速度控制模式
    SYSTEM_POSITION_CTL  // 位置控制模式
};
enum SYSTEM_RUN_STATE system_run_state = SYSTEM_VELOCITY_CTL; // 系统运行状态

enum SYSTEM_TEST_STATE {
    SYSTEM_POLE_PAIRS_TEST, // 电机极对数检测
    SYSTEM_OFFSET_TEST,     // 编码器偏置/方向检测
    SYSTEM_R_TEST,          // 定子电感检测
    SYSTEM_LD_TEST,         // 定子电阻检测
    SYSTEM_LQ_TEST          // 定子q轴电感检测
};
enum SYSTEM_TEST_STATE system_test_state = SYSTEM_OFFSET_TEST; // 系统校准状态

enum SYSTEM_UART_STATE {
    SEND_POSITION, // 发送位置
    SEND_SPEED,    // 发送速度
    SEND_NONE,     // 不发送
    SEND_CONFIG    // 发送配置
};
enum SYSTEM_UART_STATE system_uart_state = SEND_NONE; // 系统串口发送状态

/**
 * @brief   系统接口
 */

float speed_ref    = 0; // 电机速度目标值
float position_ref = 0; // 电机位置目标值

/**
 * @brief   FOC 相关变量
 */

Encoder_t encoder;  // 编码器
PID_t position_pid; // 电机位置 PID 控制器
PID_t speed_pi;     // 电机速度 PI 控制器

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
 * @brief   串口相关变量
 */
uint8_t uart_rx_buf[100]; // 串口接收缓冲区
uint8_t uart_tx_buf[400]; // 串口发送缓冲区

uint8_t command[50];                                    // 指令存放区
const char *encoder_type_str[2] = {"MT6701", "AS5600"}; // 串口输出编码器型号
char encoder_type_rxstr[10];                            // 串口读取编码器型号

/**
 * @brief   CAN 相关变量
 */
uint8_t can_flag = 0;     // CAN 中断标志
CAN_RX_MSG can_rx_msg;    // CAN 接收消息
uint8_t can_watchdog = 0; // CAN 看门狗

/**
 * @brief   参数辨识相关变量
 */
float r_s; // 定子电阻
float L_d; // 定子d轴电感
float L_q; // 定子q轴电感

alpha_beta_t i_alphabeta; // 定子静止坐标系电流
float i_s;                // 定子电流幅值
LPF_t is_lpf;             // 定子电流幅值低通滤波器

LPF_t idm_lpf; // 定子d轴电流幅值低通滤波器
LPF_t iqm_lpf; // 定子q轴电流幅值低通滤波器

/**
 * @brief   调试者临时变量
 */

/**
 * @brief   系统初始化函数
 */
static void init(void)
{
    // 参数读取
    Flash_Init();
    // 参数辨识控制器初始化
    LPF_Init(&is_lpf, 10, system_sample_time);
    LPF_Init(&idm_lpf, 100, system_sample_time * 10);
    LPF_Init(&iqm_lpf, 100, system_sample_time * 10);
    // 控制器/滤波器初始化
    LPF_Init(&id_lpf, f_c, system_sample_time);
    LPF_Init(&iq_lpf, f_c, system_sample_time);
    PID_Init(&id_pi, id_pi_kp, id_pi_ki, 0, u_dc / M_SQRT3);
    PID_Init(&iq_pi, iq_pi_kp, iq_pi_ki, 0, u_dc / M_SQRT3);
    PID_Init(&speed_pi, speed_pi_kp, speed_pi_ki, 0, speed_pi_maxoutput);
    PID_Init(&position_pid, position_pid_kp, position_pid_ki, position_pid_kd, position_pid_maxoutput);
    // 编码器初始化
    Encoder_Init(&encoder, pole_pairs, encoder_direct, encoder_type, encoder_offset);
    // 串口初始化
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, sizeof(uart_rx_buf));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    // 电流采样校准
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    HAL_TIM_Base_Start(&htim1);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    while (system_sample_state == SAMPLE_INIT) {
        ;
    }
    system_state = SYSTEM_STOP;
    HAL_Delay(100);
    sprintf(uart_tx_buf, "all ready.\r\n");
    HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, strlen(uart_tx_buf));
    // 使能 FDCAN 传输
    FDCAN_Config();
    HAL_TIM_Base_Start_IT(&htim2);
    // 使能 PWM 输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief   系统状态机线程
 */
static void command_thread(uint8_t command_length)
{
    static uint8_t float_array[4];

    // 系统状态机
    switch (system_state) {
        case SYSTEM_STOP:
            /**
             * @brief 接收串口指令
             */
            if (command_length != 0) {
                // 1. 位置设置指令
                if (memcmp(command, "set_position", 12) == 0) {
                    system_state     = SYSTEM_DEBUG_RUN;
                    system_run_state = SYSTEM_POSITION_CTL;
                    sscanf(command, "set_position %f\r\n", &position_ref);
                }
                // 2. 速度设置指令
                else if (memcmp(command, "set_speed", 9) == 0) {
                    system_state     = SYSTEM_DEBUG_RUN;
                    system_run_state = SYSTEM_VELOCITY_CTL;
                    sscanf(command, "set_speed %f\r\n", &speed_ref);
                }
                // 3. 设置位置环配置指令
                else if (memcmp(command, "config_position_pid", 19) == 0) {
                    sscanf(command, "config_position_pid %f %f %f %f\r\n", &position_pid_kp, &position_pid_ki, &position_pid_kd, &position_pid_maxoutput);
                    position_pid.KP        = position_pid_kp;
                    position_pid.KI        = position_pid_ki;
                    position_pid.KD        = position_pid_kd;
                    position_pid.outputMax = position_pid_maxoutput;
                }
                // 4. 设置速度环配置指令
                else if (memcmp(command, "config_speed_pi", 15) == 0) {
                    sscanf(command, "config_speed_pi %f %f %f\r\n", &speed_pi_kp, &speed_pi_ki, &speed_pi_maxoutput);
                    speed_pi.KP        = speed_pi_kp;
                    speed_pi.KI        = speed_pi_ki;
                    speed_pi.outputMax = speed_pi_maxoutput;
                }
                // 5. 设置电流环配置指令
                else if (memcmp(command, "config_current_pi", 17) == 0) {
                    sscanf(command, "config_current_pi %f %f %f %f\r\n", &id_pi_kp, &id_pi_ki, &iq_pi_kp, &iq_pi_ki);
                    id_pi.KP = id_pi_kp;
                    id_pi.KI = id_pi_ki;
                    iq_pi.KP = iq_pi_kp;
                    iq_pi.KI = iq_pi_ki;
                }
                // 6. 设置电流滤波器截止频率指令
                else if (memcmp(command, "config_idq_filter", 17) == 0) {
                    sscanf(command, "config_idq_filter %f\r\n", &f_c);
                    LPF_Init(&id_lpf, f_c, system_sample_time);
                    LPF_Init(&iq_lpf, f_c, system_sample_time);
                }
                // 7. 设置编码器指令
                else if (memcmp(command, "config_encoder", 14) == 0) {
                    sscanf(command, "config_encoder %d %d %f %s\r\n", &pole_pairs, &encoder_direct, &encoder_offset, encoder_type_rxstr);
                    for (int i = 0; i < 2; i++) {
                        if (strcmp(encoder_type_rxstr, encoder_type_str[i]) == 0) {
                            encoder_type = i;
                            break;
                        }
                    }
                    encoder.pole_pairs     = pole_pairs;
                    encoder.encoder_direct = encoder_direct;
                    encoder.encoder_offset = encoder_offset;
                    encoder.encoder_type   = encoder_type;
                }
                // 8. 设置id指令
                else if (memcmp(command, "config_id", 9) == 0) {
                    sscanf(command, "config_id %d\r\n", &id);
                }
                // 9. 设置母线电压指令
                else if (memcmp(command, "config_udc", 10) == 0) {
                    sscanf(command, "config_udc %f\r\n", &u_dc);
                    PID_Init(&id_pi, id_pi_kp, id_pi_ki, 0, u_dc / M_SQRT3);
                    PID_Init(&iq_pi, iq_pi_kp, iq_pi_ki, 0, u_dc / M_SQRT3);
                }
                // 9. 保存配置指令
                else if (memcmp(command, "save", 4) == 0) {
                    Flash_Save(position_pid_kp, position_pid_ki, position_pid_kd, position_pid_maxoutput, speed_pi_kp, speed_pi_ki, speed_pi_maxoutput, f_c, id_pi_kp, id_pi_ki, iq_pi_kp, iq_pi_ki, pole_pairs, encoder_direct, encoder_offset, encoder_type, id, u_dc);
                }
                // 10. 校准指令
                else if (memcmp(command, "calibration", 11) == 0) {
                    system_state      = SYSTEM_TEST;
                    system_test_state = SYSTEM_POLE_PAIRS_TEST;
                }
            }
            /**
             * @brief 接收CAN线指令
             */
            switch (can_rx_msg.command) {
                case CAN_SET_POSITION:
                    if (can_watchdog > 0) {
                        position_ref     = can_rx_msg.value;
                        system_state     = SYSTEM_RUN;
                        system_run_state = SYSTEM_POSITION_CTL;
                    }
                    break;
                case CAN_SET_SPEED:
                    if (can_watchdog > 0) {
                        speed_ref        = can_rx_msg.value;
                        system_state     = SYSTEM_RUN;
                        system_run_state = SYSTEM_VELOCITY_CTL;
                    }
                    break;
                default:
                    break;
            }
            break;
        case SYSTEM_DEBUG_RUN:
            switch (system_run_state) {
                case SYSTEM_POSITION_CTL:
                    /**
                     * @brief 接收串口指令
                     */
                    if (command_length != 0) {
                        // 1. 位置设置指令
                        if (memcmp(command, "set_position", 12) == 0) {
                            sscanf(command, "set_position %f\r\n", &position_ref);
                        }
                        // 2. 速度设置指令
                        else if (memcmp(command, "set_speed", 9) == 0) {
                            system_run_state = SYSTEM_VELOCITY_CTL;
                            sscanf(command, "set_speed %f\r\n", &speed_ref);
                        }
                        // 3. 停止指令
                        else if (memcmp(command, "stop", 4) == 0) {
                            system_state = SYSTEM_STOP;
                            speed_ref    = 0;
                            position_ref = encoder.encoder_total_theta;
                        }
                    }
                    /**
                     * @brief 接收CAN线指令
                     */
                    switch (can_rx_msg.command) {
                        case CAN_SET_POSITION:
                            if (can_watchdog > 0) {
                                position_ref     = can_rx_msg.value;
                                system_state     = SYSTEM_RUN;
                                system_run_state = SYSTEM_POSITION_CTL;
                            }
                            break;
                        case CAN_SET_SPEED:
                            if (can_watchdog > 0) {
                                speed_ref        = can_rx_msg.value;
                                system_state     = SYSTEM_RUN;
                                system_run_state = SYSTEM_VELOCITY_CTL;
                            }
                            break;
                        case CAN_STOP:
                            if (can_watchdog > 0) {
                                system_state = SYSTEM_STOP;
                                speed_ref    = 0;
                                position_ref = encoder.encoder_total_theta;
                            }
                            break;
                        default:
                            break;
                    }
                    break;
                case SYSTEM_VELOCITY_CTL:
                    /**
                     * @brief 接收串口指令
                     */
                    if (command_length != 0) {
                        // 1. 位置设置指令
                        if (memcmp(command, "set_position", 12) == 0) {
                            system_run_state = SYSTEM_POSITION_CTL;
                            sscanf(command, "set_position %f\r\n", &position_ref);
                        }
                        // 2. 速度设置指令
                        else if (memcmp(command, "set_speed", 9) == 0) {
                            sscanf(command, "set_speed %f\r\n", &speed_ref);
                        }
                        // 3. 停止指令
                        else if (memcmp(command, "stop", 4) == 0) {
                            system_state = SYSTEM_STOP;
                            speed_ref    = 0;
                            position_ref = encoder.encoder_total_theta;
                        }
                    }
                    /**
                     * @brief 接收CAN线指令
                     */
                    if (can_watchdog > 0) {
                        switch (can_rx_msg.command) {
                            case CAN_SET_POSITION:
                                position_ref     = can_rx_msg.value;
                                system_state     = SYSTEM_RUN;
                                system_run_state = SYSTEM_POSITION_CTL;
                                break;
                            case CAN_SET_SPEED:
                                speed_ref        = can_rx_msg.value;
                                system_state     = SYSTEM_RUN;
                                system_run_state = SYSTEM_VELOCITY_CTL;
                                break;
                            case CAN_STOP:
                                system_state = SYSTEM_STOP;
                                speed_ref    = 0;
                                position_ref = encoder.encoder_total_theta;
                                break;
                            default:
                                break;
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        case SYSTEM_RUN:
            switch (system_run_state) {
                case SYSTEM_VELOCITY_CTL:
                    /**
                     * @brief   接收 CAN 指令
                     */
                    if (can_watchdog > 0) {
                        switch (can_rx_msg.command) {
                            case CAN_SET_POSITION:
                                position_ref     = can_rx_msg.value;
                                system_state     = SYSTEM_RUN;
                                system_run_state = SYSTEM_POSITION_CTL;
                                break;
                            case CAN_SET_SPEED:
                                speed_ref        = can_rx_msg.value;
                                system_state     = SYSTEM_RUN;
                                system_run_state = SYSTEM_VELOCITY_CTL;
                                break;
                            case CAN_STOP:
                                system_state = SYSTEM_STOP;
                                speed_ref    = 0;
                                position_ref = encoder.encoder_total_theta;
                                break;
                            default:
                                break;
                        }
                    } else {
                        system_state = SYSTEM_STOP;
                        speed_ref    = 0;
                        position_ref = encoder.encoder_total_theta;
                    }
                    break;
                case SYSTEM_POSITION_CTL:
                    /**
                     * @brief   接收 CAN 指令
                     */
                    if (can_watchdog > 0) {
                        switch (can_rx_msg.command) {
                            case CAN_SET_POSITION:
                                position_ref     = can_rx_msg.value;
                                system_state     = SYSTEM_RUN;
                                system_run_state = SYSTEM_POSITION_CTL;
                                break;
                            case CAN_SET_SPEED:
                                speed_ref        = can_rx_msg.value;
                                system_state     = SYSTEM_RUN;
                                system_run_state = SYSTEM_VELOCITY_CTL;
                                break;
                            case CAN_STOP:
                                system_state = SYSTEM_STOP;
                                speed_ref    = 0;
                                position_ref = encoder.encoder_total_theta;
                                break;
                            default:
                                break;
                        }
                    } else {
                        system_state = SYSTEM_STOP;
                        speed_ref    = 0;
                        position_ref = encoder.encoder_total_theta;
                    }
                    break;
                default:
                    break;
            }
            break;
        case SYSTEM_FAULT:
            /**
             * @brief 接收串口指令
             */
            if (command_length != 0) {
                // 1. 位置设置指令
                if (memcmp(command, "clear_fault", 11) == 0) {
                    system_run_state = SYSTEM_STOP;
                }
            }
            break;
        case SYSTEM_TEST:
            break;
        default:
            break;
    }
}

/**
 * @brief   串口发送线程
 */
void uart_send_thread(uint8_t command_length)
{
    switch (system_uart_state) {
        case SEND_POSITION:
            sprintf(uart_tx_buf, "position: %f\r\n", encoder.encoder_total_theta);
            HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, strlen(uart_tx_buf));
            if (command_length != 0) {
                if (memcmp(command, "get_speed", 9) == 0) {
                    system_uart_state = SEND_SPEED;
                } else if (memcmp(command, "get_none", 8) == 0) {
                    system_uart_state = SEND_NONE;
                }
            }
            break;
        case SEND_SPEED:
            sprintf(uart_tx_buf, "speed: %f\r\n", encoder.encoder_speed);
            HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, strlen(uart_tx_buf));
            if (command_length != 0) {
                if (memcmp(command, "get_position", 12) == 0) {
                    system_uart_state = SEND_POSITION;
                } else if (memcmp(command, "get_none", 8) == 0) {
                    system_uart_state = SEND_NONE;
                }
            }
            break;
        case SEND_NONE:
            if (command_length != 0) {
                if (memcmp(command, "get_position", 12) == 0) {
                    system_uart_state = SEND_POSITION;
                } else if (memcmp(command, "get_speed", 9) == 0) {
                    system_uart_state = SEND_SPEED;
                } else if (memcmp(command, "get_config", 10) == 0) {
                    system_uart_state = SEND_CONFIG;
                }
            }
            break;
        case SEND_CONFIG:
            sprintf(uart_tx_buf,
                    "id:%d\r\n\r\nudc:%f\r\n\r\nposition_pid:\r\nKp: %f\r\nKi: %f\r\nKd: %f\r\nmaxoutput: %f\r\n\r\nspeed_pi:\r\nKp: %f\r\nKi: %f\r\nmaxoutput: %f\r\n\r\nidq_filter_fc: %f\r\n\r\ni_pi:\r\nid_Kp: %f\r\nid_Ki: %f\r\niq_Kp: %f\r\niq_Ki: %f\r\n\r\nencoder:\r\npole_pairs:%d\r\nencoder_direct:%d\r\nencoder_offset:%f\r\nencoder_type:%s\r\n",
                    id,
                    u_dc,
                    position_pid.KP,
                    position_pid.KI,
                    position_pid.KD,
                    position_pid.outputMax,
                    speed_pi.KP,
                    speed_pi.KI,
                    speed_pi.outputMax,
                    f_c,
                    id_pi.KP,
                    id_pi.KI,
                    iq_pi.KP,
                    iq_pi.KI,
                    pole_pairs,
                    encoder_direct,
                    encoder_offset,
                    encoder_type_str[encoder_type]);
            HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, strlen(uart_tx_buf));
            system_uart_state = SEND_NONE;
            break;
        default:
            break;
    }
}

/**
 * @brief   FDCAN发送线程
 */
void can_send_thread(void)
{
    /**
     * @brief   心跳包发送
     */
    CAN_MSG msg;
    static uint8_t float_array1[4];
    static uint8_t float_array2[4];
    msg.id  = id;
    msg.len = 0x08;
    msg.rtr = DATA_FRAME;
    float_2_array(encoder.encoder_total_theta, float_array1);
    for (int i = 0; i < 4; i++) {
        msg.buffer[i] = float_array1[i];
    }
    float_2_array(encoder.encoder_speed, float_array2);
    for (int j = 0; j < 4; j++) {
        msg.buffer[j + 4] = float_array2[j];
    }
    FDCAN_Send_Msg(&msg);
}

/**
 * @brief   系统主函数
 */
void usermain(void)
{
    init();
    while (1) {
        static uint8_t command_length;
        command_length = command_get_command(command);
        command_thread(command_length);   // 系统状态机线程
        uart_send_thread(command_length); // 串口发送线程
        /**
         * @brief   DEBUG
         */
        // printf("i: %f,%f,%f\r\n", i_abc.a, i_abc.b, i_abc.c);
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

    // 校准专用变量
    // 1. 极对数检测变量
    static int pole_pairs_read_num = 0; // 计数
    static float pole_pair_theta   = 0; // 指令电角度
    static float pole_pair_theta_1 = 0; // 初始编码器角度
    static float pole_pair_float;       // 浮点极对数
    // 2. 编码器偏移和方向校准变量
    static int offset_read_num  = 0; // 计数
    static float offset_theta_1 = 0; // 电角度0对应的机械角度
    static float offset_theta_2 = 0; // 电角度pi/3对应的机械角度
    static float offset_theta_3 = 0; // 电角度-pi/3对应的机械角度
    // 3. 定子电阻校准变量
    static int r_read_num = 0; // 计数
    static float r_s1;         // 第一次计算电阻值
    static float r_s2;         // 第二次计算电阻值
    static float r_s3;         // 第三次计算电阻值
    static float r_s4;         // 第四次计算电阻值
    // 4. 定子电感校准变量
    static int ld_read_num     = 0; // 计数
    static int lq_read_num     = 0; // 计数
    static int ld_index        = 0; // 正弦表索引
    static int lq_index        = 0; // 正弦表索引
    static float re            = 0; // DFT 实部
    static float im            = 0; // DFT 虚部
    static float sin_table[10] = {0, 0.5878f, 0.9511f, 0.9511f, 0.5878f, 0, -0.5878f, -0.9511f, -0.9511f, -0.5878f};
    static float cos_table[10] = {1, 0.8090f, 0.3090f, -0.3090f, -0.8090f, -1, -0.8090f, -0.3090f, 0.3090f, 0.8090f};
    static float idm           = 0;
    static float iqm           = 0;

    // 1. RUN
    if (system_state == SYSTEM_RUN || system_state == SYSTEM_DEBUG_RUN) {
        // position PID controller
        position_pid.ref = position_ref;
        position_pid.fdb = encoder.encoder_total_theta;
        if (system_run_state == SYSTEM_POSITION_CTL) {
            PID_Calc(&position_pid, 1, system_sample_time);
        } else {
            PID_Calc(&position_pid, 0, system_sample_time);
        }

        // speed PI Controller
        if (system_run_state == SYSTEM_POSITION_CTL) {
            speed_pi.ref = position_pid.output;
        } else {
            speed_pi.ref = speed_ref;
        }
        speed_pi.fdb = encoder.encoder_speed;
        PID_Calc(&speed_pi, 1, system_sample_time);

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
        PID_Calc(&id_pi, 1, system_sample_time);
        u_dq.d = id_pi.output;

        // q-axis
        iq_pi.ref = speed_pi.output;
        iq_pi.fdb = i_dql.q;
        PID_Calc(&iq_pi, 1, system_sample_time);
        u_dq.q = iq_pi.output;

        // dq-to-abc
        dq_2_abc(&u_dq, &u_abc, encoder.electric_theta);
    }
    // 2. TEST
    else if (system_state == SYSTEM_TEST) {
        /**
         * @brief   极对数检测
         */
        if (system_test_state == SYSTEM_POLE_PAIRS_TEST) {
            u_dq.d = 1.0f;
            u_dq.q = 0.0f;
            if (pole_pairs_read_num < 10000) {
                dq_2_abc(&u_dq, &u_abc, 0);
            } else {
                pole_pair_theta = pole_pair_theta + 0.001f;
                dq_2_abc(&u_dq, &u_abc, pole_pair_theta);
            }
            pole_pairs_read_num++;
            if (pole_pairs_read_num == 8000) {
                pole_pair_theta_1 = encoder.curr_encoder_theta;
            }
            if ((pole_pairs_read_num > 12000) && ((encoder.curr_encoder_theta - pole_pair_theta_1) > -0.005f) && ((encoder.curr_encoder_theta - pole_pair_theta_1) < 0.005f)) {
                system_test_state   = SYSTEM_OFFSET_TEST;
                u_dq.d              = 0.0f;
                pole_pair_float     = pole_pair_theta / (2 * M_PI);
                pole_pairs          = (int)roundf(pole_pair_float);
                pole_pairs_read_num = 0;
                pole_pair_theta     = 0;
                encoder.pole_pairs  = pole_pairs;
                sprintf(uart_tx_buf, "pole_pairs read done:%d\r\n", pole_pairs);
                HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, strlen(uart_tx_buf));
            }
        }
        /**
         * @brief   编码器偏置和方向校准
         */
        else if (system_test_state == SYSTEM_OFFSET_TEST) {
            u_dq.d = 1.0f;
            u_dq.q = 0.0f;
            if (offset_read_num <= 10000) {
                dq_2_abc(&u_dq, &u_abc, 0);
            } else if (offset_read_num > 10000 && offset_read_num <= 20000) {
                dq_2_abc(&u_dq, &u_abc, M_PI / 3);
            } else if (offset_read_num > 20000 && offset_read_num <= 30000) {
                dq_2_abc(&u_dq, &u_abc, -M_PI / 3);
            }
            if (offset_read_num == 9000) {
                offset_theta_1 = encoder.encoder_total_theta;
            } else if (offset_read_num == 19000) {
                offset_theta_2 = encoder.encoder_total_theta;
            } else if (offset_read_num == 29000) {
                offset_theta_3 = encoder.encoder_total_theta;
            }
            offset_read_num++;
            if (offset_read_num == 30000) {
                // 计算偏置和方向
                if ((offset_theta_1 < offset_theta_2) && (offset_theta_1 > offset_theta_3)) {
                    encoder_direct         = 1;
                    encoder.encoder_direct = encoder_direct;
                    encoder_offset         = normalize(pole_pairs, encoder_direct, offset_theta_1);
                    encoder.encoder_offset = encoder_offset;
                    sprintf(uart_tx_buf, "offset read done.direct:1\r\n");
                    HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, strlen(uart_tx_buf));
                } else if ((offset_theta_1 > offset_theta_2) && (offset_theta_1 < offset_theta_3)) {
                    encoder_direct         = -1;
                    encoder.encoder_direct = encoder_direct;
                    encoder_offset         = normalize(pole_pairs, encoder_direct, offset_theta_1);
                    encoder.encoder_offset = encoder_offset;
                    sprintf(uart_tx_buf, "offset read done.direct:-1\r\n");
                    HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, strlen(uart_tx_buf));
                } else {
                    encoder_direct         = 1;
                    encoder.encoder_direct = encoder_direct;
                    encoder_offset         = normalize(pole_pairs, encoder_direct, offset_theta_1);
                    encoder.encoder_offset = encoder_offset;
                    sprintf(uart_tx_buf, "offset read failed.\r\n");
                    HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, strlen(uart_tx_buf));
                }
                offset_read_num             = 0;
                encoder.encoder_total_theta = 0.0f;
                system_test_state           = SYSTEM_R_TEST;
            }
        }
        /**
         * @brief   定子电阻检测
         * @note    采用阶梯电压法,向d轴注入直流电压检测定子电阻
         */
        else if (system_test_state == SYSTEM_R_TEST) {
            if (r_read_num < 10000) {
                u_dq.d = 0.5f;
                u_dq.d = 0.0f;
                dq_2_abc(&u_dq, &u_abc, 0);
            } else if (r_read_num > 9999 && r_read_num < 20000) {
                u_dq.d = 1.0f;
                u_dq.q = 0.0f;
                dq_2_abc(&u_dq, &u_abc, 0);
            } else if (r_read_num > 19999 && r_read_num < 30000) {
                u_dq.d = 1.5f;
                u_dq.q = 0.0f;
                dq_2_abc(&u_dq, &u_abc, 0);
            } else if (r_read_num > 29999 && r_read_num < 40000) {
                u_dq.d = 2.0f;
                u_dq.q = 0.0f;
                dq_2_abc(&u_dq, &u_abc, 0);
            }
            abc_2_alphabeta(&i_abc, &i_alphabeta);
            is_lpf.input = sqrtf(i_alphabeta.alpha * i_alphabeta.alpha + i_alphabeta.beta * i_alphabeta.beta);
            LPF_Calc(&is_lpf, 1);
            i_s = is_lpf.output;
            r_read_num++;
            if (r_read_num == 9500) {
                r_s1 = 0.5 / i_s;
            } else if (r_read_num == 19500) {
                r_s2 = 1.0 / i_s;
            } else if (r_read_num == 29500) {
                r_s3 = 1.5 / i_s;
            } else if (r_read_num == 39500) {
                r_s4 = 2.0 / i_s;
            } else if (r_read_num == 40000) {
                system_test_state = SYSTEM_LD_TEST;
                r_read_num        = 0;
                r_s               = (r_s1 + r_s2 + r_s3 + r_s4) / 4.f;
                sprintf(uart_tx_buf, "R read done:%f\r\n", r_s);
                HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, strlen(uart_tx_buf));
            }
        }
        /**
         * @brief   定子d轴电感检测
         */
        else if (system_test_state == SYSTEM_LD_TEST) {
            ld_index = ld_read_num % 10;
            u_dq.d   = 1.0f + 4.0f * sin_table[ld_index];
            u_dq.q   = 0.0f;
            dq_2_abc(&u_dq, &u_abc, 0);
            abc_2_dq(&i_abc, &i_dq, 0);
            if (ld_read_num > 999) {
                // DFT
                re = re + i_dq.d * cos_table[ld_index];
                im = im + i_dq.d * sin_table[ld_index];
                if (ld_index == 9) {
                    idm_lpf.input = sqrtf(re * re + im * im) / 5.f;
                    LPF_Calc(&idm_lpf, 1);
                    idm = idm_lpf.output;
                    re  = 0;
                    im  = 0;
                }
            }
            ld_read_num++;
            if (ld_read_num == 20000) {
                L_d = 4.0f / (2 * M_PI * 1000 * idm);
                sprintf(uart_tx_buf, "Ld read done:%f\r\n", L_d);
                HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, strlen(uart_tx_buf));
                system_test_state = SYSTEM_LQ_TEST;
                ld_read_num       = 0;
                re                = 0;
                im                = 0;
            }
        }
        /**
         * @brief   定子q轴电感检测
         */
        else if (system_test_state == SYSTEM_LQ_TEST) {
            lq_index = lq_read_num % 10;
            u_dq.d   = 1.0f;
            u_dq.q   = 4.0f * sin_table[lq_index];
            dq_2_abc(&u_dq, &u_abc, 0);
            abc_2_dq(&i_abc, &i_dq, 0);
            if (lq_read_num > 999) {
                // DFT
                re = re + i_dq.q * cos_table[lq_index];
                im = im + i_dq.q * sin_table[lq_index];
                if (lq_index == 9) {
                    iqm_lpf.input = sqrtf(re * re + im * im) / 5.f;
                    LPF_Calc(&iqm_lpf, 1);
                    iqm = iqm_lpf.output;
                    re  = 0;
                    im  = 0;
                }
            }
            lq_read_num++;
            if (lq_read_num == 20000) {
                L_q = 4.0f / (2 * M_PI * 1000 * iqm);
                sprintf(uart_tx_buf, "Lq read done:%f\r\n", L_q);
                HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, strlen(uart_tx_buf));
                // 电流环 PI 参数计算
                id_pi_kp = L_d * 5;
                id_pi_ki = r_s * 5;
                iq_pi_kp = L_q * 5;
                iq_pi_ki = r_s * 5;
                PID_Init(&id_pi, id_pi_kp, id_pi_ki, 0, u_dc / M_SQRT3);
                PID_Init(&iq_pi, iq_pi_kp, iq_pi_ki, 0, u_dc / M_SQRT3);
                system_state      = SYSTEM_STOP;
                system_test_state = SYSTEM_POLE_PAIRS_TEST;
                lq_read_num       = 0;
                re                = 0;
                im                = 0;
            }
        }
    } else {
        PID_Calc(&id_pi, 0, system_sample_time);
        PID_Calc(&iq_pi, 0, system_sample_time);
        PID_Calc(&speed_pi, 0, system_sample_time);
        PID_Calc(&position_pid, 0, system_sample_time);
        u_dq.d = 0;
        u_dq.q = 0;
        dq_2_abc(&u_dq, &u_abc, encoder.electric_theta);
    }
    /**
     * @brief   三相 PWM 输出
     */
    // SVPWM
    e_svpwm(&u_abc, u_dc, &duty_abc);

    if (system_state == SYSTEM_RUN || system_state == SYSTEM_TEST || system_state == SYSTEM_DEBUG_RUN) {
        TIM1->CCR1 = duty_abc.dutya * TIM1->ARR;
        TIM1->CCR2 = duty_abc.dutyb * TIM1->ARR;
        TIM1->CCR3 = duty_abc.dutyc * TIM1->ARR;
    } else if (system_state == SYSTEM_STOP || system_state == SYSTEM_FAULT) {
        TIM1->CCR1 = 0 * TIM1->ARR;
        TIM1->CCR2 = 0 * TIM1->ARR;
        TIM1->CCR3 = 0 * TIM1->ARR;
    }
}

/**
 * @brief  串口中断回调函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        command_write(uart_rx_buf, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, sizeof(uart_rx_buf));
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

/**
 * @brief   FDCAN中断回调函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef sRxHeader;
    uint8_t rx_buf[8];

    if (RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        if (can_flag == 0) {
            can_flag = 1;
            // 喂狗
            if (can_watchdog < 5) {
                can_watchdog++;
            }
            HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &sRxHeader, rx_buf);
            // 确认是本电机消息后解码
            if (sRxHeader.RxFrameType == FDCAN_DATA_FRAME && sRxHeader.Identifier == id) {
                FDCAN_Msg_Decode(rx_buf, &can_rx_msg);
            }
            can_flag = 0;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static int can_watchdog_cnt = 0;
    if (htim->Instance == TIM2) {
        /**
         * @brief   CAN 发送和接收管理
         */
        can_send_thread(); // CAN发送线程

        can_watchdog_cnt++;
        if (can_watchdog_cnt == 1) {
            can_watchdog_cnt = 0;
            if (can_watchdog > 0) {
                can_watchdog--;
            }
        }
    }
}