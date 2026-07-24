#include "bsp_motor.h"
#include "bsp_gpio.h"
#include "bsp_tmc2209.h"
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
    GPIOA->BRR = GPIO_PIN_5;             /* 直接寄存器确保拉低 PA5 */
    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
    TIM3->DIER |= TIM_DIER_UIE;          /* 补设更新中断 (HAL有时漏掉) */
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

/*====================================================================
 * 电机驱动测试用例
 *   - 依赖 USART1 (PA9, 115200 8N1) 输出进度, 不依赖 U1 协议
 *   - 直接写 USART1->DR, 不干扰 DMA RX 通道
 *====================================================================*/

/* 简单串口打印 */
static void motor_print(const char *s)
{
    while (*s) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = (uint8_t)(*s++);
    }
}

/* 打印无符号整数 */
static void motor_print_u32(uint32_t v)
{
    char buf[11];
    int i = 0;

    if (v == 0) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = '0';
        return;
    }
    while (v) {
        buf[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i--) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = (uint8_t)buf[i];
    }
}

/**
  * @brief  TIM3 独立验证 (纯串口, 无需示波器)
  *   - 读回所有关键寄存器, 打印实际值
  *   - 验证 CNT 在变化 (证明定时器时钟/计数正常)
  *   - 短暂启动 PWM, 检查 SR 无异常标志
  * @retval 'T'=全部正常 'E'=异常
  */
uint8_t BSP_TIM3_Test(void)
{
    motor_print("\r\n========== TIM3 Verify ==========\r\n");

    /*── 时钟 ──*/
    uint32_t apb1 = HAL_RCC_GetPCLK1Freq();
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t tclk = (apb1 == hclk) ? apb1 : apb1 * 2;
    motor_print("APB1:"); motor_print_u32(apb1);
    motor_print(" TIM3_CLK:"); motor_print_u32(tclk); motor_print("\r\n");

    /*── 寄存器快照 ──*/
    uint16_t cr1  = TIM3->CR1;
    uint16_t cr2  = TIM3->CR2;
    uint16_t smcr = TIM3->SMCR;
    uint16_t dier = TIM3->DIER;
    uint16_t sr   = TIM3->SR;
    uint16_t psc  = TIM3->PSC;
    uint16_t arr  = TIM3->ARR;
    uint16_t cc1  = TIM3->CCR1;
    uint16_t cc2  = TIM3->CCR2;
    uint16_t cnt1 = TIM3->CNT;

    motor_print("CR1=");  motor_print_u32(cr1);
    motor_print(" CR2="); motor_print_u32(cr2);
    motor_print(" SMCR="); motor_print_u32(smcr);
    motor_print(" DIER="); motor_print_u32(dier); motor_print("\r\n");

    motor_print("PSC=");  motor_print_u32(psc);
    motor_print(" ARR="); motor_print_u32(arr);
    motor_print(" CCR1="); motor_print_u32(cc1);
    motor_print(" CCR2="); motor_print_u32(cc2); motor_print("\r\n");

    motor_print("SR=0x"); motor_print_u32(sr);
    motor_print(" CNT="); motor_print_u32(cnt1); motor_print("\r\n");

    /*── 检查 CubeMX 配置是否写入 ──*/
    if (arr == 0 || arr == 0xFFFF) {
        motor_print("FAIL: ARR="); motor_print_u32(arr);
        motor_print(" (not configured)\r\n");
        return 'E';
    }
    if (cc2 == 0 || cc2 >= arr) {
        motor_print("WARN: CCR2="); motor_print_u32(cc2);
        motor_print(" (duty may be 0 or 100%)\r\n");
    }
    motor_print("Freq calc: "); motor_print_u32(tclk / ((psc+1)*(arr+1)));
    motor_print(" Hz\r\n");

    /*── 启动 PWM, 验证 CNT 在跑 ──*/
    motor_print("Start PWM CH2... ");
    TIM3->SR = 0;   /* 清全部标志 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    uint16_t cnt_a = TIM3->CNT;
    HAL_Delay(2);
    uint16_t cnt_b = TIM3->CNT;
    uint16_t sr2   = TIM3->SR;
    uint16_t cr1b  = TIM3->CR1;

    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

    motor_print("CEN="); motor_print_u32(cr1b & 1);
    motor_print(" CNT:"); motor_print_u32(cnt_a);
    motor_print(" -> ");  motor_print_u32(cnt_b);

    uint16_t delta = (cnt_b > cnt_a) ? (cnt_b - cnt_a) : (cnt_a - cnt_b);
    if (delta == 0) {
        motor_print(" STOPPED!\r\n");
        return 'E';
    }
    motor_print(" delta="); motor_print_u32(delta);
    motor_print(" (OK)\r\n");

    /*── SR 检查: 只关心 BIF(break), 其余标志来自未配置通道, 无害 ──*/
    motor_print("SR=0x"); motor_print_u32(sr2);
    if (sr2 & TIM_SR_BIF) {
        motor_print(" BREAK!\r\n");
        return 'E';
    }
    motor_print(" OK\r\n");

    motor_print("========== RESULT: T ==========\r\n");
    return 'T';
}

/**
  * @brief  电机驱动自测
  * @retval 'T'=全通过 '1'=TMC通信失败 '2'=CW超时 '3'=CCW超时
  */
/* 测试预设 */
#define REV_STEPS    3200U    /* 1圈 @ 16微步 (200×16) */
#define TIMEOUT_MS   10000U   /* 10s 超时 */

static uint8_t motor_run_test(const char *label, uint8_t dir,
                               uint32_t steps, uint32_t freq_hz)
{
    motor_print(label);
    BSP_Motor_Move(dir, steps, freq_hz);

    uint32_t to = TIMEOUT_MS;
    while (BSP_Motor_IsBusy() && to--) HAL_Delay(1);

    if (BSP_Motor_IsBusy()) {
        BSP_Motor_Stop();
        motor_print(" FAIL (timeout)\r\n");
        return 0;
    }
    motor_print(" PASS (done=");
    motor_print_u32(BSP_Motor_GetDone());
    motor_print(")\r\n");
    return 1;
}

/**
  * @brief  电机驱动全功能测试
  *   - TMC2209 通信 + 配置验证
  *   - CW/CCW 多速度: 500Hz(1圈) 2000Hz(5圈) 5000Hz(10圈)
  * @retval 'T'=全通过 '1'=通信失败 '2'=运动超时
  */
uint8_t BSP_Motor_Test(void)
{
    motor_print("\r\n===== Motor Test =====\r\n");

    /*── TMC2209 ──*/
    motor_print("[1] TMC2209... ");
    uint8_t tmc = BSP_TMC_Test();
    if (tmc == 'T') {
        motor_print("  PASS\r\n");
    } else if (tmc == 'W') {
        motor_print("  WARN (W=OK R=FAIL, continue)\r\n");
    } else {
        motor_print("  FAIL\r\n");
        return '1';
    }
    HAL_Delay(200);

    /*── 速度测试 ──*/
    motor_print("[2] Speed tests:\r\n");
    uint8_t ok = 1;

    ok &= motor_run_test("  500Hz  CW 1rev... ", MOTOR_DIR_CW,  REV_STEPS,      500);
    HAL_Delay(300);
    ok &= motor_run_test("  500Hz CCW 1rev... ", MOTOR_DIR_CCW, REV_STEPS,      500);
    HAL_Delay(300);

    ok &= motor_run_test(" 2000Hz  CW 5rev... ", MOTOR_DIR_CW,  REV_STEPS * 5, 2000);
    HAL_Delay(300);
    ok &= motor_run_test(" 2000Hz CCW 5rev... ", MOTOR_DIR_CCW, REV_STEPS * 5, 2000);
    HAL_Delay(300);

    ok &= motor_run_test(" 5000Hz  CW 10rev...", MOTOR_DIR_CW,  REV_STEPS * 10, 5000);
    HAL_Delay(300);
    ok &= motor_run_test(" 5000Hz CCW 10rev...", MOTOR_DIR_CCW, REV_STEPS * 10, 5000);

    if (ok) {
        motor_print("===== ALL PASS =====\r\n");
        return 'T';
    } else {
        motor_print("===== SOME FAILED =====\r\n");
        return '2';
    }
}
