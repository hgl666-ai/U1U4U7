#include "adc_calib.h"
#include "app.h"
#include "app_internal.h"
#include "protocol_u7.h"
#include "protocol_u4.h"
#include "bsp_adc.h"
#include <string.h>

/*===== 状态机 =====
 * 流程: 回零 → U4开始校准(0x11) → CCW去3mm → 等到位 → CW回3mm → 等到位
 *       → U4完成校准(0x12) → 延迟 → 读U4报文 min/max → 阈值判定 → 回9B ACK
 * 极值来源: U4 报文流 0x30 的 min_vo1/max_vo1/min_vo2/max_vo2 (U4 校准窗口内累计)
 * 前提依赖: U4 固件 v1.3.1 须在 0x11→0x12 窗口内填充 min/max 字段
 */
enum { AC_IDLE=0, AC_HOME, AC_START_CALIB, AC_MOVE_FWD, AC_WAIT_FWD,
       AC_MOVE_BACK, AC_WAIT_BACK, AC_FINISH_CALIB, AC_DELAY, AC_READ, AC_DONE };

static uint8_t  ac_phase;
static uint16_t ac_max1, ac_min1;   /* CH1 极值 (来自 U4 min/max 字段) */
static uint16_t ac_max2, ac_min2;   /* CH2 极值 */
static uint8_t  ac_fail;            /* 1=整体失败 */
static uint32_t ac_deadline;        /* 电机/延迟等待截止时刻 */

/*===== 阈值判定: 单通道是否通过 =====
 * [2026-08-24] 修正: 失败条件为 MAX-MIN>2000 (摆幅过大异常) 或 MAX>4000 或 MIN<50 */
static uint8_t AC_ChPass(uint16_t maxv, uint16_t minv)
{
    if ((maxv - minv) > ADC_CALIB_MAX_DIFF) return 0;
    if (maxv > ADC_CALIB_MAX_VAL) return 0;
    if (minv < ADC_CALIB_MIN_VAL) return 0;
    return 1;
}

/*===== 公开 API =====*/

void ADC_Calib_Init(void)
{
    ac_phase = AC_IDLE;
}

uint8_t ADC_Calib_Start(void)
{
    if (ac_phase != AC_IDLE) return 1;  /* BUSY */

    ac_phase   = AC_HOME;
    ac_fail    = 0;
    ac_max1 = ac_max2 = 0;
    ac_min1 = ac_min2 = 0xFFFF;
    return 0;  /* 不立即 ACK, 由 ADC_Calib_Run 异步完成 */
}

uint8_t ADC_Calib_IsBusy(void)
{
    return ac_phase != AC_IDLE;
}

void ADC_Calib_Run(void)
{
    if (ac_phase == AC_IDLE) return;

    /*── 回零 ──*/
    if (ac_phase == AC_HOME) {
        int ret = U7_Home();
        if (ret == U7_PROTO_PENDING) return;
        if (ret != U7_PROTO_OK) { ac_fail = 1; ac_phase = AC_DONE; return; }
        ac_phase = AC_START_CALIB;
        return;
    }

    /*── U4 开始校准 (0x11) ──*/
    if (ac_phase == AC_START_CALIB) {
        int ret = U4_StartCalib();
        if (ret == U4_PROTO_PENDING) return;
        if (ret != U4_PROTO_OK) { ac_fail = 1; ac_phase = AC_DONE; return; }
        ac_phase = AC_MOVE_FWD;
        return;
    }

    /*── 去程: CCW 走 3mm 进行程区 (回零位于 CW 限位端点, 必须反向 CCW) ──*/
    if (ac_phase == AC_MOVE_FWD) {
        int ret = U7_MotorStep(ADC_CALIB_MOTOR_ID, U7_MOTOR_CCW, ADC_CALIB_SWEEP_STEPS);
        if (ret == U7_PROTO_PENDING) return;
        if (ret != U7_PROTO_OK) { ac_fail = 1; ac_phase = AC_DONE; return; }
        /* [2026-08-24] 等待须覆盖电机物理走完 (U7 非阻塞, 收到即回 OK):
         * 619步 @≈2ms/步 ≈1.24s + 500ms 余量, 否则提前发回程会被 U7 busy 拒
         * (现象: 只走一段就停, ACK 极值为初始值) */
        ac_deadline = HAL_GetTick() + ADC_CALIB_MOTOR_DELAY_MS
                    + (uint32_t)ADC_CALIB_SWEEP_STEPS * 2U;
        ac_phase = AC_WAIT_FWD;
        return;
    }

    /*── 等去程到位 (U4 校准窗口内累计极值) ──*/
    if (ac_phase == AC_WAIT_FWD) {
        if (HAL_GetTick() < ac_deadline) return;
        ac_phase = AC_MOVE_BACK;
        return;
    }

    /*── 回程: CW 走回 3mm 到零点 (严格往复) ──*/
    if (ac_phase == AC_MOVE_BACK) {
        int ret = U7_MotorStep(ADC_CALIB_MOTOR_ID, U7_MOTOR_CW, ADC_CALIB_SWEEP_STEPS);
        if (ret == U7_PROTO_PENDING) return;
        if (ret != U7_PROTO_OK) { ac_fail = 1; ac_phase = AC_DONE; return; }
        ac_deadline = HAL_GetTick() + ADC_CALIB_MOTOR_DELAY_MS
                    + (uint32_t)ADC_CALIB_SWEEP_STEPS * 2U;
        ac_phase = AC_WAIT_BACK;
        return;
    }

    /*── 等回程到位 ──*/
    if (ac_phase == AC_WAIT_BACK) {
        if (HAL_GetTick() < ac_deadline) return;
        ac_phase = AC_FINISH_CALIB;
        return;
    }

    /*── U4 完成校准 (0x12) ──*/
    if (ac_phase == AC_FINISH_CALIB) {
        int ret = U4_FinishCalib();
        if (ret == U4_PROTO_PENDING) return;
        if (ret != U4_PROTO_OK) { ac_fail = 1; ac_phase = AC_DONE; return; }
        ac_deadline = HAL_GetTick() + ADC_CALIB_READ_DELAY_MS;
        ac_phase = AC_DELAY;
        return;
    }

    /*── 延迟: 等 U4 报文更新到最终 min/max ──*/
    if (ac_phase == AC_DELAY) {
        if (HAL_GetTick() < ac_deadline) return;
        ac_phase = AC_READ;
        return;
    }

    /*── 读取 U4 报文 min/max + 阈值判定 ──*/
    if (ac_phase == AC_READ) {
        ac_max1 = u4_report.max_vo1;
        ac_min1 = u4_report.min_vo1;
        ac_max2 = u4_report.max_vo2;
        ac_min2 = u4_report.min_vo2;
        if (!AC_ChPass(ac_max1, ac_min1) || !AC_ChPass(ac_max2, ac_min2))
            ac_fail = 1;
        ac_phase = AC_DONE;
        return;
    }

    /*── 汇总: 9B [status][max1][min1][max2][min2] ──*/
    if (ac_phase == AC_DONE) {
        uint8_t result[ADC_CALIB_RESULT_LEN];
        result[0] = ac_fail ? APP_STATUS_ERR_TEST : APP_STATUS_OK;
        result[1] = (uint8_t)(ac_max1 >> 8);
        result[2] = (uint8_t)(ac_max1);
        result[3] = (uint8_t)(ac_min1 >> 8);
        result[4] = (uint8_t)(ac_min1);
        result[5] = (uint8_t)(ac_max2 >> 8);
        result[6] = (uint8_t)(ac_max2);
        result[7] = (uint8_t)(ac_min2 >> 8);
        result[8] = (uint8_t)(ac_min2);

        APP_SendAck(MGR_CMD_ADC_CALIB, result, ADC_CALIB_RESULT_LEN);
        ac_phase = AC_IDLE;
    }
}
