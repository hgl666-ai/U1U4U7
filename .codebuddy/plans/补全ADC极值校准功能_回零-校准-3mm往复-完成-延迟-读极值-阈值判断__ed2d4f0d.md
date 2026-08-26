---
name: 补全ADC极值校准功能（回零-校准-3mm往复-完成-延迟-读极值-阈值判断）
overview: 将现有 U1 的 ADC 极值校准（CMD 0x0010）改造成新流程：回零 → U4 开始校准(0x11) → 电机来回移动 3mm → U4 完成校准(0x12) → 延迟 → 从 U4 报文流读取 max/min → 阈值判断（MAX-MIN<2000 或 MAX>4000 或 MIN<50 即失败）→ 返回 9 字节 ACK [status][max1][min1][max2][min2]。
todos:
  - id: update-app-h-macros
    content: 修改 app.h：ADC_CALIB_RESULT_LEN 8→9，新增 3mm 步数/阈值/读前延迟宏
    status: completed
  - id: rewrite-adc-calib
    content: 重写 adc_calib.c 状态机：回零→U4开始校准→CCW移3mm→等到位→U4完成校准→延迟→读 min/max 字段→阈值判定→回 9B ACK
    status: completed
    dependencies:
      - update-app-h-macros
  - id: review-dispatch
    content: 核对 app.c 的 0x0010 分发与 busy 冲突检查，确认无需改动或补充
    status: completed
    dependencies:
      - rewrite-adc-calib
  - id: verify-impl
    content: 编译验证 U1 工程无警告错误，复核 Flash 占用与 9B 返回帧字节序正确
    status: completed
    dependencies:
      - review-dispatch
---

## 产品概述

补全 U1 治具的 ADC 极值校准功能（CMD 0x0010），将当前由 U1 移动电机并自行采样累计极值的实现，改为"回零 → U4 开始校准 → 电机往复 3mm → U4 完成校准 → 延迟 → 读取 U4 报文 min/max 字段 → 阈值判定"的完整校准流程，并支持阈值失败判定。

## 核心功能

- 新校准流程：回零（U7 0x29）→ 发送 U4 开始校准命令（0x11）→ 电机沿 CCW 移动 3mm（约 619 步）→ 等待电机到位 → 发送 U4 完成校准命令（0x12）→ 固定延迟 → 读取 U4 报文流 0x30 中的 min_vo1/max_vo1/min_vo2/max_vo2 字段。
- 阈值判定（逐通道，任一条件成立即该通道失败）：MAX-MIN < 2000 或 MAX > 4000 或 MIN < 50。CH1、CH2 任一通道失败则整体返回失败。
- 返回帧协议修改：ACK 由当前 8 字节 [max1][min1][max2][min2] 改为 9 字节 [status(1)] [max1(2)] [min1(2)] [max2(2)] [min2(2)]；status=0x00 表示 PASS，非 0 表示 FAIL（不细分失败通道/原因，极值仍返回供上位机查看）。
- 命令本身 0x0010（LEN=0）不需改动，仅 ACK 负载格式变化；阈值 2000/4000/50 为固定值硬编码，无需 DATA 参数。
- 前提依赖：U4 固件 v1.3.1 需在 0x11→0x12 校准窗口内累计并填充报文流 ccd_data_t 的 min/max 字段（U1 侧 U4_ParseReport 已完整解析这些字段）。

## 技术栈

- 平台：STM32F103C6 嵌入式固件（MDK-ARM，C90，Flash 32KB 极紧）
- 协议：U1↔PC 经理协议（大端 CRC）、U1↔U4（MODBUS 小端 CRC，U4_Proto_Run 非阻塞）、U1↔U7（U7_Proto_Run 非阻塞）
- 状态机模式：非阻塞 phase 状态机，由 APP_Run 主循环周期调用推进（与 diam_calib.c / 现有 adc_calib.c 一致）

## 实现方案

### 整体思路

复用现有 `diam_calib.c` 的非阻塞状态机模式，完全重写 `adc_calib.c` 的状态机，将原有"U1 移动电机逐段采样 vo1/vo2 自行累计极值"改为"U1 编排 回零→U4校准→电机往复3mm→完成校准→延迟→读 min/max"的完整闭环，并在汇总阶段增加阈值判定。

### 新状态机设计（adc_calib.c）

```
Phase 枚举: AC_IDLE=0, AC_HOME, AC_START_CALIB, AC_MOVE_FWD, AC_WAIT_FWD,
            AC_MOVE_BACK, AC_WAIT_BACK, AC_FINISH_CALIB, AC_DELAY, AC_READ, AC_DONE
```

- **AC_HOME**：调用 `U7_Home()`（非阻塞）；成功则 motor_pos=0，进入 AC_START_CALIB；失败则 AC_DONE(FAIL)。
- **AC_START_CALIB**：调用 `U4_StartCalib()`（0x11，非阻塞 U4_PROTO_PENDING）；成功进入 AC_MOVE_FWD。
- **AC_MOVE_FWD**：按 2026-08-22 方向修正，回零位于 CW 限位端点，3mm 行程必须用 `U7_MotorStep(MOTOR_ID, U7_MOTOR_CCW, ADC_CALIB_SWEEP_STEPS)` 进行程区；指令发出后进入 AC_WAIT_FWD。
- **AC_WAIT_FWD**：等 `ADC_CALIB_MOTOR_DELAY_MS`（500ms）让电机物理到位并让 U4 在校准窗口内充分累计 min/max。
- **AC_MOVE_BACK**：严格往复——反向 `U7_MotorStep(MOTOR_ID, U7_MOTOR_CW, ADC_CALIB_SWEEP_STEPS)` 走回零点；指令发出后进入 AC_WAIT_BACK。
- **AC_WAIT_BACK**：等 `ADC_CALIB_MOTOR_DELAY_MS`（500ms）让电机回零到位，U4 完成完整往复区间的极值累计。
- **AC_FINISH_CALIB**：调用 `U4_FinishCalib()`（0x12，非阻塞）；成功进入 AC_DELAY。
- **AC_DELAY**：固定延迟 `ADC_CALIB_READ_DELAY_MS`（新增，建议 200ms）等待 U4 完成报文更新后进入 AC_READ。
- **AC_READ**：读取 `u4_report.min_vo1/max_vo1/min_vo2/max_vo2` 到结果变量，对 CH1/CH2 分别做阈值判定，计算整体 PASS/FAIL，进入 AC_DONE。
- **AC_DONE**：拼 9 字节结果帧 [status][max1][min1][max2][min2]，`APP_SendAck(MGR_CMD_ADC_CALIB, result, ADC_CALIB_RESULT_LEN)`，返回 idle。

### 阈值判定

- CH1 失败：`max1-min1<2000 || max1>4000 || min1<50`
- CH2 失败：`max2-min2<2000 || max2>4000 || min2<50`
- status = (CH1 通过 && CH2 通过) ? 0x00 : 非0（统一用 APP_STATUS_ERR_TEST 或其他非0值表示 FAIL，不细分）。

### 3mm 步数换算

- 复用公式 `steps = um × 3200 / 15500`：3000um → 3000×3200/15500 = 619 步。
- 在 `app.h` 新增 `ADC_CALIB_SWEEP_UM 3000U` 与 `ADC_CALIB_3MM_STEPS`（可在 adc_calib.c 内用公式计算，避免宏重复）。

### 关键决策与权衡

- **读 U4 min/max 字段**：符合用户确认，减少 U1 侧采样逻辑与状态机复杂度；但**依赖 U4 固件在校准窗口内填充字段**。计划中明确标注该前提，若实测 U4 不填充，需回退到 U1 自行采样累计方案（U1 在校准窗口内累计 vo1/vo2 的 min/max）。
- **严格往复 3mm（用户明确要求，不简化）**：完整执行 CCW 去程 3mm → 到位停留 → CW 回程 3mm → 到位停留，两次移动均在 U4 校准窗口（0x11 开始 → 0x12 完成）内，确保 U4 累计到往返全程的极值。方向遵循 2026-08-22 修正（回零后首次移动用 CCW 进行程区，回程用 CW 回零点）。
- **统一 PASS/FAIL**：符合用户确认，最省 Flash（32KB 极紧）。

## 实现注意

- **Flash 空间极紧（32KB，-Os）**：尽量复用现有宏与函数，避免新增大块代码；阈值常量直接硬编码或新增宏，避免引入字符串/printf。
- **方向修正（关键）**：务必用 U7_MOTOR_CCW 作为回零后首次移动方向，否则会继续顶 CW 限位导致电机失步（2026-08-22 已踩坑）。
- **busy 检查**：`ADC_Calib_Start()` 沿用现有 `phase!=0` 判定；`MGR_CMD_ADC_CALIB` 分发处可参照 MGR_CMD_MOTOR_SEQ 增加 `deferred_cmd`/`app_state` 冲突检查，避免与回零/定点序列并发。
- **协议兼容**：命令号 0x0010 不变，仅 ACK 负载 8→9 字节；`ADC_CALIB_RESULT_LEN` 8→9 需同步改 `app.h`。PC 上位机需按新 9B 格式解析。
- **日志/诊断**：不新增调试打印（省 Flash）；失败时仅通过 status 非0 传递，不打印大块 payload。
- **重启状态**：`ADC_Calib_Init()` 在 APP_Init 调用，保持 phase=AC_IDLE。

## 架构设计

沿用现有非阻塞状态机架构，模块职责不变：

- `adc_calib.c`：校准编排 + 阈值判定 + 9B 汇总 ACK（重写内部状态机）
- `app.c`：仅命令分发调用 `ADC_Calib_Start()`，主循环调用 `ADC_Calib_Run()`（无需改动分发逻辑）
- `protocol_u4.c`：U4_StartCalib/FinishCalib 已存在，直接复用；u4_report.min/max 字段已解析
- `protocol_u7.c`：U7_Home/MotorStep 已存在，直接复用
- `app.h`：调整宏定义（RESULT_LEN 8→9、新增阈值/3mm/延迟宏）

依赖链清晰、组件间无新增耦合，完全符合现有架构。

## 目录结构

本次修改仅涉及 U1 工程的两个文件：

```
U1/U1project/Core/Inc/app.h                      # [MODIFY] ADC 校准宏定义
  - ADC_CALIB_RESULT_LEN: 8 → 9（新增 1B status 在帧首）
  - 新增 ADC_CALIB_SWEEP_UM=3000U（3mm 行程）
  - 新增 ADC_CALIB_SWEEP_STEPS=619U（3000×3200/15500 换算结果，直接常量避免运行时除法）
  - 新增 ADC_CALIB_MIN_DIFF=2000U / ADC_CALIB_MAX_VAL=4000U / ADC_CALIB_MIN_VAL=50U（阈值）
  - 新增 ADC_CALIB_READ_DELAY_MS=200U（完成校准后读极值前延迟）
  - 现有 MOVE_TIMEOUT_MS/ADC_TIMEOUT_MS 若不再使用可保留或清理

U1/U1project/Core/Src/adc_calib.c                # [MODIFY] 完全重写校准状态机
  - 新增 phase 枚举与状态变量（max1/min1/max2/min2、fail 标志、deadline）
  - 重写 ADC_Calib_Start：校验 idle、初始化状态与极值初值（max=0, min=0xFFFF）
  - 重写 ADC_Calib_Run：实现 回零→开始校准→CCW去3mm→等到位→CW回3mm→等到位→完成校准→延迟→读min/max→阈值判定→9B ACK 的非阻塞流程（严格往复）
  - 阈值判定函数（内联/静态）：ChPass(max,min) = !(max-min<2000 || max>4000 || min<50)
  - 汇总：result[0]=status, result[1..2]=max1, result[3..4]=min1, result[5..6]=max2, result[7..8]=min2（大端）
  - ADC_Calib_Init/IsBusy 逻辑保留
```

## 关键代码结构（接口/常量定义）

新增/修改的宏定义（app.h 中，供 adc_calib.c 使用）：

```c
/*===== ADC 极值自动校准 (CMD 0x0010) =====*/
#define ADC_CALIB_MOTOR_ID            0U  /* 电机编号, 与 U7 MOTOR_ID 一致 */
#define ADC_CALIB_SWEEP_UM         3000U  /* 3mm 往复行程 */
#define ADC_CALIB_SWEEP_STEPS       619U  /* 3000um×3200/15500 换算步数 */
#define ADC_CALIB_MOTOR_DELAY_MS     500U /* 电机到位等待 */
#define ADC_CALIB_READ_DELAY_MS      200U /* 完成校准后读极值前延迟 */
/* 阈值判定 */
#define ADC_CALIB_MIN_DIFF          2000U /* 同通道 MAX-MIN 下限 */
#define ADC_CALIB_MAX_VAL           4000U /* MAX 上限 */
#define ADC_CALIB_MIN_VAL             50U /* MIN 下限 */
#define ADC_CALIB_RESULT_LEN           9U /* [status] + 4极值×2B */
```

状态机 phase 枚举（adc_calib.c 内部）：

```c
enum { AC_IDLE=0, AC_HOME, AC_START_CALIB, AC_MOVE_FWD,
       AC_WAIT_FWD, AC_MOVE_BACK, AC_WAIT_BACK,
       AC_FINISH_CALIB, AC_DELAY, AC_READ, AC_DONE };
```