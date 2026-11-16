#include "diam_calib.h"
#include "app.h"
#include "app_internal.h"
#include "protocol_u7.h"
#include "protocol_u4.h"

/*===== state machine =====
 * [2026-08-25] flow rewrite:
 *   motor home (U7 0x29) -> U4 zero (0x10) -> set offset 0 (0x17) ->
 *   CCW incremental move to 1mm/1.5mm/2mm -> dynamic wait done ->
 *   100ms settle delay -> wait new report -> read diameter ->
 *   check |Dactual-N|<=50um -> 25B summary ACK
 * direction: home stops at CW limit (IN1), move must use CCW into travel area
 *
 * ★★[2026-11-16] 判据改为"相对基准"★★
 *   原判据 |Dactual − N| ≤ 50 里, Dactual 是 U4 的**绝对读数**(实测起点约 1750),
 *   而 N 是"位移量"(200/400/600), 两者差一个固定偏置 → 永远 FAIL。
 *   实测三次数据(起点 1748~1750): 位移 200/400/600 对应的读数变化是
 *   179~182 / 411~415 / 582~585, 误差只有 11~21µm, **本来就在 50µm 容差内**。
 *   所以新增 DC_BASELINE 阶段: 归零/设偏移之后、移动之前先读一次作为基准 L0,
 *   每个位置判 | |L0 − Li| − N | ≤ 容差。ACK 里的 D 字段改为回"相对位移量",
 *   这样和 N 放在一起可以直接对照。
 */
enum { DC_IDLE = 0, DC_HOME, DC_ZERO, DC_SETOFFSET, DC_BASELINE, DC_MOVE,
       DC_WAIT_MOVE, DC_DELAY, DC_READ, DC_NEXT,
       DC_RETURN, DC_WAIT_RETURN, DC_DONE };

/* [2026-11-16] 测试点改为 0.2/0.4/0.6mm (原 1.0/1.5/2.0mm)
 * ★原因(实测扫位置数据): 传感器读数是以约 2047 计数(≈2mm)为周期循环的,
 *   并且从"磁铁对齐"位置往前约 1mm 处会读到 length=0 (超范围/盲区)。
 *   原来的 1.0/1.5/2.0mm 正好跨过盲区并绕了一整圈, 所以 D 永远对不上 N。
 *   现改为 0.2/0.4/0.6mm, 全部落在对齐位置往前的单调有效区间内。
 * 步数: 200/400/600 um × 1600/1000 = 320/640/960, 每段增量 320 步。 */
static const uint16_t dc_target_um[DIAM_CALIB_POS_COUNT] = {200, 400, 600};

static uint8_t  dc_phase;
static uint8_t  dc_pos_idx;                        /* current position 0..2 */
static uint16_t dc_result_n[DIAM_CALIB_POS_COUNT]; /* target N */
static uint16_t dc_result_d[DIAM_CALIB_POS_COUNT]; /* 相对基准的位移量 (不是绝对读数) */
static uint8_t  dc_result_pass[DIAM_CALIB_POS_COUNT];
static uint16_t dc_base_len;                       /* [2026-11-16] 基准读数 L0 */

static uint32_t mv_deadline;       /* motor settle deadline (500ms + steps*2ms) */
static uint32_t dc_delay_deadline; /* 100ms settle delay deadline */
static uint32_t dc_read_deadline;  /* wait DUT report timeout deadline */

/*===== steps for a target position (open loop) =====
 * steps = target_um * 3200 / 15500
 */
static uint16_t DC_StepsForTarget(uint16_t target_um)
{
    return (uint16_t)((uint32_t)target_um * DIAM_CALIB_STEPS_PER_REV
                      / DIAM_CALIB_OUTPUT_CIRC_UM);
}

/*===== public API =====*/

void DiamCalib_Init(void)
{
    dc_phase = DC_IDLE;
}

uint8_t DiamCalib_Start(void)
{
    if (dc_phase != DC_IDLE) return 1;  /* BUSY */

#if DIAM_CALIB_SKIP_HOME
    /* [2026-11-13] 跳过回零: 执行顺序 ③→④, ③ 结束已把物件送回后挡块(零点) */
    dc_phase   = DC_ZERO;
#else
    dc_phase   = DC_HOME;
#endif
    dc_pos_idx = 0;
    dc_base_len = 0;
    /* clear results (all FAIL default) */
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

#if !DIAM_CALIB_SKIP_HOME
    /*-- HOME: motor home (CW to IN1, motor_pos=0) --*/
    if (dc_phase == DC_HOME) {
        int ret = U7_Home();
        if (ret == U7_PROTO_PENDING) return;
        if (ret != U7_PROTO_OK) { dc_phase = DC_DONE; return; }  /* fail -> summary FAIL */
        dc_phase = DC_ZERO;
        return;
    }
#endif

    /*-- ZERO: U4 sensor zero --*/
    if (dc_phase == DC_ZERO) {
        int ret = U4_ZeroSensor();
        if (ret == U4_PROTO_PENDING) return;
        if (ret != U4_PROTO_OK) { dc_phase = DC_DONE; return; }
        dc_phase = DC_SETOFFSET;
        return;
    }

    /*-- SETOFFSET: set offset 0 so length = raw_length --*/
    if (dc_phase == DC_SETOFFSET) {
        int ret = U4_SetOffset(0);
        if (ret == U4_PROTO_PENDING) return;
        if (ret != U4_PROTO_OK) { dc_phase = DC_DONE; return; }
        /* [2026-11-16] 进入 BASELINE: 先读一次绝对读数作为基准 */
        u4_report_fresh  = 0;
        dc_read_deadline = HAL_GetTick() + DIAM_CALIB_DUT_TIMEOUT_MS;
        dc_phase = DC_BASELINE;
        return;
    }

    /*-- [2026-11-16] BASELINE: 移动前读一次绝对读数 L0 作为基准 --
     *   之后每个位置都判 | |L0 − Li| − N | ≤ 容差, 不再和绝对读数直接比。 */
    if (dc_phase == DC_BASELINE) {
        if (!u4_report_fresh) {
            if (HAL_GetTick() > dc_read_deadline) {
                dc_phase = DC_DONE;          /* 读不到基准 → 直接 FAIL 出结果 */
            }
            return;
        }
        u4_report_fresh = 0;                 /* consume */
        dc_base_len = (uint16_t)u4_report.length;
        dc_phase = DC_MOVE;
        return;
    }

    /*-- MOVE: CCW incremental move to current target --*/
    if (dc_phase == DC_MOVE) {
        uint16_t target = dc_target_um[dc_pos_idx];
        uint16_t steps;
        if (dc_pos_idx == 0) {
            steps = DC_StepsForTarget(target);           /* 0 -> 1000um */
        } else {
            steps = DC_StepsForTarget(target)
                  - DC_StepsForTarget(dc_target_um[dc_pos_idx - 1]);  /* delta */
        }

        /* [2026-08-25] direction fix: home is at CW limit, move CCW into area */
        int ret = U7_MotorStep(DIAM_CALIB_MOTOR_ID, U7_MOTOR_CCW, steps);
        if (ret == U7_PROTO_PENDING) return;  /* motor session in progress */
        if (ret != U7_PROTO_OK) { dc_phase = DC_DONE; return; }

        /* [2026-11-13] U7 移动按 MOTOR_SPEED_SLOW=2000Hz → 0.5ms/步, 这里取 1ms/步留 2x 余量。
         * 旧值 2ms/步是按 500Hz 低速估的; 换 1mm 导程后步数×15.5, 沿用 2ms 会白等 ~14s/段 */
        mv_deadline = HAL_GetTick() + DIAM_CALIB_MOTOR_DELAY_MS
                    + (uint32_t)steps * 1U;
        dc_phase = DC_WAIT_MOVE;
        return;
    }

    /*-- WAIT_MOVE: wait motor physically done --*/
    if (dc_phase == DC_WAIT_MOVE) {
        if (HAL_GetTick() < mv_deadline) return;
        dc_delay_deadline = HAL_GetTick() + DIAM_CALIB_READ_DELAY_MS;
        dc_phase = DC_DELAY;
        return;
    }

    /*-- DELAY: 100ms settle after arrival (user requirement) --*/
    if (dc_phase == DC_DELAY) {
        if (HAL_GetTick() < dc_delay_deadline) return;
        /* settle done -> wait new report, read Dactual */
        u4_report_fresh  = 0;
        dc_read_deadline = HAL_GetTick() + DIAM_CALIB_DUT_TIMEOUT_MS;
        dc_phase = DC_READ;
        return;
    }

    /*-- READ: wait new report -> read Dactual -> judge --*/
    if (dc_phase == DC_READ) {
        if (!u4_report_fresh) {
            if (HAL_GetTick() > dc_read_deadline) {
                dc_result_d[dc_pos_idx]    = 0;      /* DUT no response */
                dc_result_pass[dc_pos_idx] = 0;
                dc_phase = DC_NEXT;
            }
            return;
        }
        /* new report */
        u4_report_fresh = 0;   /* consume */
        {
            uint16_t n   = dc_result_n[dc_pos_idx];
            uint16_t d   = (uint16_t)u4_report.length;
            /* [2026-11-16] 相对基准: delta = |L0 − D|, 再和 N 比。
             * 原先是 |D − N|, 但 D 是绝对读数(起点约 1750), N 是位移量,
             * 差一个固定偏置 → 永远 FAIL。实测改成相对后误差只有 11~21µm。 */
            uint16_t delta = (dc_base_len > d) ? (uint16_t)(dc_base_len - d)
                                              : (uint16_t)(d - dc_base_len);
            uint16_t err = (delta > n) ? (uint16_t)(delta - n)
                                       : (uint16_t)(n - delta);
            dc_result_d[dc_pos_idx]    = delta;      /* ACK 里回"相对位移量" */
            dc_result_pass[dc_pos_idx] = (err <= DIAM_CALIB_TOLERANCE_UM) ? 1 : 0;
        }
        dc_phase = DC_NEXT;
        return;
    }

    /*-- NEXT: advance position or finish --*/
    if (dc_phase == DC_NEXT) {
        dc_pos_idx++;
        if (dc_pos_idx < DIAM_CALIB_POS_COUNT) {
            dc_phase = DC_MOVE;
        } else {
            /* [2026-11-13] 三个位置测完 → 先把物件送回零点, 再出结果 */
            dc_phase = DC_RETURN;
        }
        return;
    }

    /*-- [2026-11-13] RETURN: 退 2.5mm 顶住后挡块, 把物件送回零点 (供下一项测试直接启动) --*/
    if (dc_phase == DC_RETURN) {
        int ret = U7_MotorStep(DIAM_CALIB_MOTOR_ID, U7_MOTOR_CW, DIAM_CALIB_RETURN_STEPS);
        if (ret == U7_PROTO_PENDING) return;
        if (ret != U7_PROTO_OK) {
            /* 回程失败: 仍返回测量结果, 但物件不在零点 —— 下一项测试会因此失败,
             * 相当于"用下一项的失败来暴露本次回程失败"(ACK 无多余字段可标记) */
            dc_phase = DC_DONE;
            return;
        }
        mv_deadline = HAL_GetTick() + DIAM_CALIB_MOTOR_DELAY_MS
                    + (uint32_t)DIAM_CALIB_RETURN_STEPS * 1U;
        dc_phase = DC_WAIT_RETURN;
        return;
    }

    /*-- [2026-11-13] WAIT_RETURN: 等回程走完 --*/
    if (dc_phase == DC_WAIT_RETURN) {
        if (HAL_GetTick() < mv_deadline) return;
        dc_phase = DC_DONE;
        return;
    }

    /*-- DONE: summary 25B ACK --*/
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
