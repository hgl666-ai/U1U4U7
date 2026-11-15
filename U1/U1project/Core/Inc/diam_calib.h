#ifndef __DIAM_CALIB_H
#define __DIAM_CALIB_H

#include "main.h"
#include <stdint.h>

/* 测径精度校准 (CMD 0x0011/0x0012)
 * [2026-08-25] 流程重写:
 *   电机回零(U7 0x29) → U4归零(0x10) → 设偏移0(0x17) →
 *   CCW 增量移动 1/1.5/2mm → 动态等到位 → 100ms 稳定延迟 →
 *   等新报文读直径 → 判定 |Dactual-N|≤50um → 25B 汇总 ACK
 * 方向: 回零在限位端点, 移动必须反向进入行程区 (2026-08-22 经验)
 * ★[2026-11-13] 下文"CW/CCW"是**逻辑方向**。换新丝杆电机后物理方向与旧电机相反,
 *   已在 U7 侧用 MOTOR_DIR_INVERT (bsp_motor.h) 统一取反 ——
 *   U1 发 CCW 仍表示"离开限位、进入行程区", 无需改动。 */

#define DIAM_CALIB_POS_COUNT        3U      /* 测试位置数 */
#define DIAM_CALIB_TOLERANCE_UM     50U     /* 容忍误差 0.05mm */

#define DIAM_CALIB_STEPS_PER_REV    1600U   /* [2026-11-13 ★重新标定] 3200→1600
                                             * 实测: 5次点动(8000步)走完 4.2mm 全程 → 1905 步/mm;
                                             * 稳定段 1600步走 1033um → 1549 步/mm。取 1600 步/mm
                                             * (= 200步/圈 × 8细分, 或 16细分 × 2mm导程; 需硬件确认)。
                                             * → 1mm = 1600 步 (原按 3200 算 → 实际走 2 倍) */
#define DIAM_CALIB_OUTPUT_CIRC_UM   1000U   /* 1 圈 = 1000um 直线 (标定基准, 与步数配合) */

#define DIAM_CALIB_MOTOR_DELAY_MS   500U    /* 电机到位基础等待 (实际 = 500 + 步数×1ms) */
#define DIAM_CALIB_READ_DELAY_MS    100U    /* 移动到位后、读直径前稳定延迟 */
#define DIAM_CALIB_DUT_TIMEOUT_MS   1000U   /* 等 DUT 报文超时 */
#define DIAM_CALIB_MOTOR_ID         0U      /* 电机编号 */
#define DIAM_CALIB_SUMMARY_LEN      25U     /* 汇总: [总判定1] + 3×[8] */

/*===== [2026-11-13] 去掉回零 + 结束送回零点 =====
 * 执行顺序是 ③ → ④: ③ 结束时已把物件送回后挡块(零点), 所以 ④ 可以直接启动。
 * ④ 本身是单向 0→1→1.5→2mm, 测完停在 +2mm, 必须再退 2.5mm 顶住后挡块,
 * 把物件送回零点, 否则下一轮(③或④)起点就错了。
 * ★剩余依赖: 每次运行前物件必须在后挡块上 (由上一项测试的结束动作保证)。 */
#define DIAM_CALIB_SKIP_HOME        1U      /* 1=跳过开头的回零 (物件已在零点) */
#define DIAM_CALIB_RETURN_UM     2200U      /* [2026-11-13] 结束回程 2mm + 超压 0.2mm
                                             * (原 2.5mm → 用户实测"回退的长了点") */
#define DIAM_CALIB_RETURN_STEPS  3200U      /* [2026-11-14 用户指定] 回程 2.0mm × 1600 步/mm = 3200
                                             * ★与最远前进量(2mm)等长 → 测完正好退回零点, 但不再"多压
                                             *   0.2mm", 因此**失去自校正**; 若观察到逐轮前移,
                                             *   把本值改回 3520 (2.2mm) 恢复顶压复位。 */

void     DiamCalib_Init(void);             /* 复位状态 */
uint8_t  DiamCalib_Start(void);            /* 0=已启动, 1=BUSY */
void     DiamCalib_Run(void);              /* APP_Run 周期推进 */
uint8_t  DiamCalib_IsBusy(void);           /* 1=进行中 */

#endif
