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
 */
enum { DC_IDLE = 0, DC_HOME, DC_ZERO, DC_SETOFFSET, DC_MOVE,
       DC_WAIT_MOVE, DC_DELAY, DC_READ, DC_NEXT, DC_DONE };

static const uint16_t dc_target_um[DIAM_CALIB_POS_COUNT] = {1000, 1500, 2000};

static uint8_t  dc_phase;
static uint8_t  dc_pos_idx;                        /* current position 0..2 */
static uint16_t dc_result_n[DIAM_CALIB_POS_COUNT]; /* target N */
static uint16_t dc_result_d[DIAM_CALIB_POS_COUNT]; /* measured Dactual */
static uint8_t  dc_result_pass[DIAM_CALIB_POS_COUNT];

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

    dc_phase   = DC_HOME;
    dc_pos_idx = 0;
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

    /*-- HOME: motor home (CW to IN1, motor_pos=0) --*/
    if (dc_phase == DC_HOME) {
        int ret = U7_Home();
        if (ret == U7_PROTO_PENDING) return;
        if (ret != U7_PROTO_OK) { dc_phase = DC_DONE; return; }  /* fail -> summary FAIL */
        dc_phase = DC_ZERO;
        return;
    }

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

        /* U7 returns OK immediately (non-blocking); wait enough for physical move:
         * 500ms base + steps*2ms (slow speed), same formula as MotorSeq_Run */
        mv_deadline = HAL_GetTick() + DIAM_CALIB_MOTOR_DELAY_MS
                    + (uint32_t)steps * 2U;
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

    /*-- NEXT: advance position or finish --*/
    if (dc_phase == DC_NEXT) {
        dc_pos_idx++;
        if (dc_pos_idx < DIAM_CALIB_POS_COUNT) {
            dc_phase = DC_MOVE;
        } else {
            dc_phase = DC_DONE;
        }
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
