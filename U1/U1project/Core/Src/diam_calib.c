#include "diam_calib.h"
#include "app.h"
#include "app_internal.h"
#include "protocol_u7.h"
#include "protocol_u4.h"

/*===== 状态机 =====
 * Phase: 0=idle, 1=ZERO(U4归零), 2=MOVE(电机转动), 3=WAIT(电机到位),
 *        4=READ(读Dactual+判定+事件), 5=NEXT(切下一位置), 6=DONE(汇总)
 */
enum { DC_IDLE = 0, DC_ZERO, DC_MOVE, DC_WAIT, DC_READ, DC_NEXT, DC_DONE };

static const uint16_t dc_target_um[DIAM_CALIB_POS_COUNT] = {1000, 1500, 2000};

static uint8_t  dc_phase;
static uint8_t  dc_pos_idx;                       /* 当前位置 0..2 */
static uint16_t dc_result_n[DIAM_CALIB_POS_COUNT]; /* 目标 N */
static uint16_t dc_result_d[DIAM_CALIB_POS_COUNT]; /* 实测 Dactual */
static uint8_t  dc_result_pass[DIAM_CALIB_POS_COUNT];

static uint32_t mv_deadline;       /* 电机到位等待截止 */
static uint32_t dc_read_deadline;  /* 等 DUT 报文超时截止 */

/*===== 内部: 目标位置对应的电机步数 (开环) =====
 * steps = target_um × 3200 / 15500
 */
static uint16_t DC_StepsForTarget(uint16_t target_um)
{
    return (uint16_t)((uint32_t)target_um * DIAM_CALIB_STEPS_PER_REV
                      / DIAM_CALIB_OUTPUT_CIRC_UM);
}

/*===== 公开 API =====*/

void DiamCalib_Init(void)
{
    dc_phase = DC_IDLE;
}

uint8_t DiamCalib_Start(void)
{
    if (dc_phase != DC_IDLE) return 1;  /* BUSY */

    dc_phase   = DC_ZERO;
    dc_pos_idx = 0;
    /* 清空结果 (全部 FAIL 初值) */
    for (uint8_t i = 0; i < DIAM_CALIB_POS_COUNT; i++) {
        dc_result_n[i]    = dc_target_um[i];
        dc_result_d[i]    = 0;
        dc_result_pass[i] = 0;
    }
    return 0;
}

uint8_t DiamCalib_IsBusy(void)
{
    return dc_phase != DC_IDLE;
}

void DiamCalib_Run(void)
{
    if (dc_phase == DC_IDLE) return;

    /*── ZERO: DUT 测径点归零 (电机固定起点即零点, 无需动作) ──*/
    if (dc_phase == DC_ZERO) {
        int ret = U4_ZeroSensor();
        if (ret == U4_PROTO_PENDING) return;
        if (ret != U4_PROTO_OK) {
            dc_phase = DC_DONE;   /* 归零失败 → 汇总 FAIL */
            return;
        }
        dc_phase = DC_MOVE;   /* 开始位置1移动 */
        return;
    }

    /*── MOVE: 电机开环转动 (从上一位置到当前目标的增量步数) ──*/
    if (dc_phase == DC_MOVE) {
        uint16_t target = dc_target_um[dc_pos_idx];
        uint16_t steps;
        if (dc_pos_idx == 0) {
            steps = DC_StepsForTarget(target);           /* 0 → 1000um */
        } else {
            steps = DC_StepsForTarget(target)
                  - DC_StepsForTarget(dc_target_um[dc_pos_idx - 1]);  /* 增量 */
        }

        int ret = U7_MotorStep(DIAM_CALIB_MOTOR_ID, U7_MOTOR_CW, steps);
        if (ret == U7_PROTO_PENDING) return;  /* 电机会话进行中 */
        /* [2026-08-17] M2 修复: 电机指令失败 → 直接汇总 (结果已按全 FAIL 初始化),
         * 避免电机未动却按"已到位"继续读数 */
        if (ret != U7_PROTO_OK) {
            dc_phase = DC_DONE;
            return;
        }

        /* 指令已发出, 等物理到位 */
        mv_deadline = HAL_GetTick() + DIAM_CALIB_MOTOR_DELAY_MS;
        dc_phase = DC_WAIT;
        return;
    }

    /*── WAIT: 等电机到位 ──*/
    if (dc_phase == DC_WAIT) {
        if (HAL_GetTick() < mv_deadline) return;
        /* 到位 → 等 DUT 报文刷新, 读 Dactual */
        u4_report_fresh  = 0;
        dc_read_deadline = HAL_GetTick() + DIAM_CALIB_DUT_TIMEOUT_MS;
        dc_phase = DC_READ;
        return;
    }

    /*── READ: 等新报文 → 读 Dactual → 判定 → 发事件 ──*/
    if (dc_phase == DC_READ) {
        if (!u4_report_fresh) {
            if (HAL_GetTick() > dc_read_deadline) {
                dc_result_d[dc_pos_idx]    = 0;      /* DUT 无响应 */
                dc_result_pass[dc_pos_idx] = 0;
                dc_phase = DC_NEXT;
            }
            return;
        }
        /* 有新报文 */
        u4_report_fresh = 0;   /* 消费 */
        dc_result_d[dc_pos_idx] = (uint16_t)u4_report.length;
        {
            uint16_t n   = dc_result_n[dc_pos_idx];
            uint16_t d   = dc_result_d[dc_pos_idx];
            uint16_t err = (d > n) ? (d - n) : (n - d);
            dc_result_pass[dc_pos_idx] = (err <= DIAM_CALIB_TOLERANCE_UM) ? 1 : 0;
        }
        dc_phase = DC_NEXT;
        return;
    }

    /*── NEXT: 推进到下一位置或完成 ──*/
    if (dc_phase == DC_NEXT) {
        dc_pos_idx++;
        if (dc_pos_idx < DIAM_CALIB_POS_COUNT) {
            dc_phase = DC_MOVE;
        } else {
            dc_phase = DC_DONE;
        }
        return;
    }

    /*── DONE: 汇总 25B ACK ──*/
    if (dc_phase == DC_DONE) {
        uint8_t result[DIAM_CALIB_SUMMARY_LEN];
        uint8_t allpass = 1;
        uint8_t off     = 1;

        for (uint8_t i = 0; i < DIAM_CALIB_POS_COUNT; i++) {
            if (!dc_result_pass[i]) allpass = 0;
            uint16_t n   = dc_result_n[i];
            uint16_t d   = dc_result_d[i];
            uint16_t err = (d > n) ? (d - n) : (n - d);
            result[off]     = (uint8_t)(i + 1);
            result[off + 1] = (uint8_t)(n >> 8);   result[off + 2] = (uint8_t)n;
            result[off + 3] = (uint8_t)(d >> 8);   result[off + 4] = (uint8_t)d;
            result[off + 5] = (uint8_t)(err >> 8); result[off + 6] = (uint8_t)err;
            result[off + 7] = dc_result_pass[i];
            off += 8;
        }
        result[0] = allpass ? 1 : 0;

        APP_SendAck(MGR_CMD_DIAM_CALIB, result, DIAM_CALIB_SUMMARY_LEN);
        dc_phase = DC_IDLE;
    }
}
