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

/*===== 梯形加减速状态 [2026-08-19] =====
 * 丢步修复: 直接以目标频率硬启动/硬停 → 平滑起步/减速到位。
 * 全程在 ARR 域线性 (f = 32MHz/(ARR+1)), 加减速即 ARR 均匀增减, ISR 内无除法。
 * ARR 越小频率越高: 加速段 ARR 递减, 减速段 ARR 递增。 */
enum { RAMP_ACCEL = 0, RAMP_CRUISE, RAMP_DECEL };

static volatile uint8_t  ramp_state;        /* 0=ACCEL 1=CRUISE 2=DECEL */
static volatile uint16_t ramp_arr;          /* 当前 ARR (仅 ISR 维护) */
static uint16_t ramp_arr_start;             /* 起步低速 ARR */
static uint16_t ramp_arr_apex;              /* 目标/顶点 ARR */
static uint16_t ramp_step_a;                /* 加速每档 ARR 减量 */
static uint16_t ramp_step_d;                /* 减速每档 ARR 增量 */
static uint16_t ramp_decel_steps;           /* 剩余 ≤ 该值进入减速 */
static uint8_t  ramp_tier_cnt;              /* 分档计数器 */
static uint8_t  ramp_armed;                 /* 1=启用加减速 (小步数/无差量时禁用) */

#ifdef U7_DEBUG
static void motor_print(const char *s);   /* 前向声明: 调试打印 (定义见文件尾部) */
#define MOVE_DBG(s) motor_print(s)
#else
#define MOVE_DBG(s) do { } while (0)
#endif

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

    /* [2026-08-19] 加减速采用 ARPE=0/OC2PE=0 立即生效模式 (不改 tim.c):
     * ISR 在溢出后入口执行, 此时 CNT≈0 远小于新 ARR, 写 ARR 无周期截断;
     * 约束: ① 只在 ISR 内改 ARR/CCR2; ② 先写 CCR2 再写 ARR;
     *       ③ 频率 ≤10kHz (周期 100µs, ISR ~2µs)。
     * (曾尝试运行时开 ARPE+OC2PE 预装载, 引入装载时序不确定性且 TIM_CCMR2_OC2PE
     *  定义不存在 (CH2 的 OC2PE 在 CCMR1), 故弃用预装载方案) */

    motor_target = 0;
    motor_count  = 0;
    motor_busy   = 0;
    ramp_state   = RAMP_CRUISE;
    ramp_armed   = 0;
}

/**
  * @brief  启动电机步进运动 (带梯形加减速)
  * @param  dir:     MOTOR_DIR_CW(0) / MOTOR_DIR_CCW(1)
  * @param  steps:   步数 (微步单位，1/16 微步 = 3200 脉冲/圈,
  *                  由 BSP_TMC_SetMicrostep16() 在初始化时显式配置)
  * @param  freq_hz: 目标脉冲频率 (Hz)，自动从 MOTOR_F_START_HZ 加减速到达;
  *                  实际值 = 32MHz / (ARR+1)
  * @note   非阻塞，调用后立即返回，由 ISR 计步、加减速并在到达目标后停 PWM (保持力矩)
  * @retval 1=已启动, 0=被拒绝 (参数无效或电机运动中)
  *         [2026-08-17] M7: 增加返回值, 运动中拒绝必须让上层知晓并回错误
  *         [2026-08-19] 加减速: profile 预计算 + 安全启动序列 (防虚计/防周期截断)
  */
uint8_t BSP_Motor_Move(uint8_t dir, uint32_t steps, uint32_t freq_hz)
{
    uint32_t arr_start, arr_target, apex;
    uint32_t accel_steps, decel_steps, delta, a_t, d_t, step_a, step_d;
    uint8_t  armed;

    if (steps == 0 || freq_hz == 0) {
        MOVE_DBG("MOVE reject: param 0\r\n");
        return 0;
    }
    if (motor_busy) {
        MOVE_DBG("MOVE reject: BUSY\r\n");
        return 0;            /* 运动中，拒绝新指令 */
    }

    /* 频率截断到 [起步, 上限] (PSC=0 时 16 位 ARR 下限 ≈488Hz) */
    if (freq_hz < MOTOR_F_START_HZ) freq_hz = MOTOR_F_START_HZ;
    if (freq_hz > MOTOR_FREQ_MAX_HZ) freq_hz = MOTOR_FREQ_MAX_HZ;

    /*── Profile 预计算 (ARR 域线性) ──*/
    arr_start   = TIM3_CLK_HZ / MOTOR_F_START_HZ - 1U;   /* 起步低速 */
    arr_target  = TIM3_CLK_HZ / freq_hz - 1U;            /* 目标速度 */
    accel_steps = MOTOR_ACCEL_STEPS;
    decel_steps = MOTOR_DECEL_STEPS;
    apex        = arr_target;

    if (steps < accel_steps + decel_steps) {
        /* 三角退化: 目标太小, 加速到半程即转减速, 顶点未达目标速度 (防高速急停) */
        accel_steps = steps / 2U;
        decel_steps = steps - accel_steps;
        apex = arr_start - (arr_start - arr_target) * accel_steps / MOTOR_ACCEL_STEPS;
        if (apex < arr_target) apex = arr_target;
    }

    delta = (arr_start > apex) ? (arr_start - apex) : 0U;
    armed = (steps >= MOTOR_MIN_RAMP_STEPS) && (delta >= 2U);
    if (!armed) { apex = arr_start; delta = 0U; }        /* 微小步数 → 全程起步低速, 防除零 */

    a_t = accel_steps / MOTOR_RAMP_GRAN; if (a_t == 0U) a_t = 1U;
    d_t = decel_steps / MOTOR_RAMP_GRAN; if (d_t == 0U) d_t = 1U;
    step_a = delta / a_t; if (step_a == 0U) step_a = 1U;
    step_d = delta / d_t; if (step_d == 0U) step_d = 1U;

    /* 填充加减速全局 (ISR 只读) */
    ramp_arr_start   = (uint16_t)arr_start;
    ramp_arr_apex    = (uint16_t)apex;
    ramp_step_a      = (uint16_t)step_a;
    ramp_step_d      = (uint16_t)step_d;
    ramp_decel_steps = (uint16_t)decel_steps;
    ramp_armed       = armed;
    ramp_state       = (armed && accel_steps > 0U) ? RAMP_ACCEL : RAMP_CRUISE;
    ramp_tier_cnt    = 0;
    ramp_arr         = (uint16_t)arr_start;

    motor_target = steps;
    motor_count  = 0;

    BSP_DIR2_Set(dir ? MOTOR_DIR_CCW : MOTOR_DIR_CW);

    /*── 安全启动序列: 清 UIF/NVIC 防虚计一步; busy 最后置位 (ARPE=0, 写后立即生效) ──*/
    TIM3->CNT  = 0;                                    /* 复位计数, 首周期 = ARR+1 */
    TIM3->ARR  = ramp_arr_start;                       /* 起步低速, 立即生效 */
    TIM3->CCR2 = (uint16_t)(ramp_arr_start >> 1);      /* 50% 占空比 */
    TIM3->EGR  = TIM_EGR_UG;                           /* 更新事件: 同步计数 + 置 UIF */
    TIM3->SR   = (uint16_t)~TIM_SR_UIF;                /* 清 UIF, 防 Start 后虚计 */
    NVIC_ClearPendingIRQ(TIM3_IRQn);                   /* 清残留挂起中断 */

    BSP_EN2_Set(0);                                    /* 使能驱动 (电荷泵开始建立) */
    /* [2026-08-19 16:4x] Start_IT 前强制还原 HAL 状态 (双保险):
     * 新 HAL(v1.10+) 检查 ChannelState[CH2] (TIM_CHANNEL_STATE_GET) 且失败返回
     * HAL_ERROR; 旧 HAL 检查 htim->State。ISR 直接寄存器停后两者都可能残留,
     * 这里一并还原, 杜绝"第二段 START_IT FAIL" */
    TIM_CHANNEL_STATE_SET(&htim3, TIM_CHANNEL_2, HAL_TIM_CHANNEL_STATE_READY);
    htim3.State = HAL_TIM_STATE_READY;
    if (HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2) != HAL_OK) {
        /* [2026-08-19] 防御: HAL 状态机异常时拒绝假启动
         * (曾因 State/ChannelState 残留导致 Start_IT 静默失败, 上层误以为电机在动) */
        MOVE_DBG("MOVE reject: START_IT FAIL\r\n");
        BSP_EN2_Set(1);
        motor_busy = 0;
        return 0;
    }
    /* 计步只用溢出中断: 关掉 HAL 顺带开的 CC2 比较中断, 减少中断负载与干扰源 */
    TIM3->DIER &= ~TIM_DIER_CC2IE;
    TIM3->DIER |= TIM_DIER_UIE;                        /* 补设 UIE (HAL 有时漏掉) */

    motor_busy = 1;                                    /* busy 最后置位 */
    return 1;
}

/**
  * @brief  急停电机 (唯一脱机入口: EN 拉高失去保持力矩)
  * @note   [2026-08-19] 到位保持力矩后, 上层确认不再需要保持位置时调用本函数脱机
  */
void BSP_Motor_Stop(void)
{
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_2);   /* HAL 内部按 CC2E→UIE→CEN 顺序停 */
    BSP_EN2_Set(1);                      /* 脱机 */
    motor_busy   = 0;
    motor_target = 0;
    motor_count  = 0;
    ramp_armed   = 0;
    ramp_state   = RAMP_CRUISE;
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
  *         [2026-08-19] 梯形加减速状态机 + 到位只停 PWM 保持力矩 (不脱机)
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;
    if (!motor_busy) return;             /* 防停止瞬间残留 UIF 误入 */

    motor_count++;

    /*── 到位: 停 PWM, 保持 EN 低 (TMC 以 IHOLD 保持力矩) ──*/
    if (motor_count >= motor_target) {
        TIM3->DIER &= ~TIM_DIER_UIE;     /* ① 先断中断源 */
        TIM3->CCER &= ~TIM_CCER_CC2E;    /* ② 再关输出 (STEP 释放) */
        TIM3->CR1  &= ~TIM_CR1_CEN;      /* ③ 最后停计数 */
        /* [2026-08-19 16:4x] ★最终根因（U7_DEBUG 实证 "MOVE reject: START_IT FAIL"）:
         * 本工程 HAL 为 v1.10+，HAL_TIM_PWM_Start_IT 检查的是**通道级状态**
         * TIM_CHANNEL_STATE_GET(ChannelState[])（非旧版 htim->State），不匹配则
         * 返回 HAL_ERROR。第一段 Start_IT 置 ChannelState[CH2]=BUSY 后，ISR 直接
         * 寄存器停 PWM 未还原该状态 → 第二段 Start_IT 必然失败 → "START_IT FAIL"。
         * 必须同步还原 ChannelState（等价 HAL_TIM_PWM_Stop_IT 内部行为）。
         * 此前仅置 htim3.State=READY 不够（State 是新 HAL 的兼容遗留字段）。 */
        TIM_CHANNEL_STATE_SET(&htim3, TIM_CHANNEL_2, HAL_TIM_CHANNEL_STATE_READY);
        htim3.State = HAL_TIM_STATE_READY;   /* 顺带还原（兼容旧版 HAL 的 State 检查） */
        MOVE_DBG("MOVE done (ISR reached target)\r\n");   /* U7_DEBUG: 确认 ISR 到位 */
        motor_busy = 0;                  /* 不置 EN! 保持力矩 (脱机仅在 BSP_Motor_Stop) */
        return;
    }

    /*── 每步检查: 进入减速窗口 (加速中也可能触发 → 三角) ──*/
    if ((motor_target - motor_count) <= ramp_decel_steps)
        ramp_state = RAMP_DECEL;

    if (++ramp_tier_cnt >= MOTOR_RAMP_GRAN) {   /* 每 N 步调一档 ARR */
        ramp_tier_cnt = 0;
        switch (ramp_state) {
        case RAMP_ACCEL:
            ramp_arr -= ramp_step_a;
            if (ramp_arr <= ramp_arr_apex) { ramp_arr = ramp_arr_apex; ramp_state = RAMP_CRUISE; }
            break;
        case RAMP_DECEL:
            /* [2026-08-19] 修复: 先判 clamp 再相加, 防 uint16 溢出
             * (曾出现 64839+1920 溢出为 1223 → ARR 突变 26kHz, 减速段轨迹错乱) */
            if ((uint32_t)ramp_arr + ramp_step_d >= ramp_arr_start)
                ramp_arr = ramp_arr_start;
            else
                ramp_arr += ramp_step_d;
            break;
        default: /* CRUISE: 不动 */ break;
        }
        TIM3->CCR2 = (uint16_t)(ramp_arr >> 1);  /* 先 CCR2 后 ARR (50% 占空比保持) */
        TIM3->ARR  = ramp_arr;
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
