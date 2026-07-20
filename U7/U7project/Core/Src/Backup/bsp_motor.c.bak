#include "bsp_motor.h"
#include "bsp_gpio.h"
#include "tim.h"

extern TIM_HandleTypeDef htim3;

/* TIM3 时钟 = APB1×2 = 16MHz×2 = 32MHz */
#define TIM3_CLK_HZ  32000000UL

/* 电机状态 */
static volatile uint32_t motor_target;   /* 目标步数 */
static volatile uint32_t motor_count;    /* 已走步数 */
static volatile uint8_t  motor_busy;     /* 1=运动中 */

/**
  * @brief  电机驱动初始化
  * @note   在 MX_TIM3_Init() 和 BSP_GPIO_Init() 之后调用
  *         关闭 PWM，电机脱机，状态清零
  */
void BSP_Motor_Init(void)
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    BSP_EN2_Set(1);          /* 脱机 */
    BSP_DIR2_Set(0);         /* 方向 CW */

    motor_target = 0;
    motor_count  = 0;
    motor_busy   = 0;
}

/**
  * @brief  启动电机步进运动
  * @param  dir:     MOTOR_DIR_CW(0) / MOTOR_DIR_CCW(1)
  * @param  steps:   步数 (微步单位，当前默认 1/8 微步 = 1600 脉冲/圈)
  * @param  freq_hz: 脉冲频率 (Hz)，实际值 = 32MHz / (ARR+1)
  * @note   非阻塞，调用后立即返回，由 ISR 计步并在到达目标后自动停机
  */
void BSP_Motor_Move(uint8_t dir, uint32_t steps, uint32_t freq_hz)
{
    if (steps == 0 || freq_hz == 0) return;
    if (motor_busy) return;              /* 运动中，忽略新指令 */

    uint32_t arr = TIM3_CLK_HZ / freq_hz;
    if (arr < 1)   arr = 1;
    if (arr > 65535) arr = 65535;

    motor_target = steps;
    motor_count  = 0;
    motor_busy   = 1;

    BSP_DIR2_Set(dir ? MOTOR_DIR_CCW : MOTOR_DIR_CW);

    /* 更新 TIM3 周期 (速度) 和占空比 (50%) */
    __HAL_TIM_SET_AUTORELOAD(&htim3, (uint16_t)(arr - 1));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)(arr / 2));

    /* 强制更新寄存器并清除产生的 UIF 标志，避免 Start_IT 时虚计一步 */
    HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

    BSP_EN2_Set(0);                      /* 使能电机 */
    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
}

/**
  * @brief  急停电机
  */
void BSP_Motor_Stop(void)
{
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_2);
    BSP_EN2_Set(1);                      /* 脱机 */
    motor_busy   = 0;
    motor_target = 0;
    motor_count  = 0;
}

/**
  * @brief  查询电机是否在运动中
  * @retval 1=运动中, 0=空闲
  */
uint8_t BSP_Motor_IsBusy(void)
{
    return motor_busy;
}

/**
  * @brief  获取已完成步数
  */
uint32_t BSP_Motor_GetDone(void)
{
    return motor_count;
}

/**
  * @brief  TIM3 周期溢出回调 (每个 PWM 脉冲触发一次)
  * @note   HAL 库从 TIM3_IRQHandler → HAL_TIM_IRQHandler 自动调用
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    if (!motor_busy) return;

    motor_count++;

    if (motor_count >= motor_target) {
        /* 到达目标步数：停机脱机 */
        HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_2);
        BSP_EN2_Set(1);
        motor_busy = 0;
    }
}
