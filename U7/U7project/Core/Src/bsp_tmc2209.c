#include "bsp_tmc2209.h"
#include "bsp_gpio.h"
#include "tim.h"

extern TIM_HandleTypeDef htim3;

/*===== TIM3 UART 常量 =====
 * TIM3 时钟 = APB1×2 = 16MHz×2 = 32MHz
 * 9600 baud → 1 bit = 32,000,000/9600 = 3333.33 ticks
 * 使用 3333 ticks/bit, 误差 0.01% (标准 UART 容忍 ±2%)
 *
 * 对标 ESP m_gptimer_uart: 硬件定时器控 UART 时序, CPU无关
 * TIM3 保存/恢复保证电机 PWM (PB5) 不受影响
 *====================================================================*/
#define TMC_BIT_TICKS    3333
#define TMC_HALF_TICKS   1667

/*===== 直接寄存器操作, 消除函数调用开销 =====
 * BSP_GPIO_Write → HAL_GPIO_WritePin 有入栈/条件/出栈开销 (~20 cycles)
 * BSRR/BRR 单条指令完成, 时序最精确
 *================================================================*/
#define TX_HIGH()    (MOTO_TX_Port->BSRR = MOTO_TX_Pin)   /* PA7=HIGH */
#define TX_LOW()     (MOTO_TX_Port->BRR  = MOTO_TX_Pin)   /* PA7=LOW  */
#define RX_LEVEL()   ((MOTO_RX_Port->IDR & MOTO_RX_Pin) ? 1 : 0)  /* PA6 电平 */

/* TIM3 状态备份 */
static uint16_t tim3_saved_psc, tim3_saved_arr, tim3_saved_ccr2;
static uint16_t tim3_saved_cr1, tim3_saved_ccer;

static void tim3_save(void)
{
    TIM3->CR1  &= ~TIM_CR1_CEN;    /* 先停定时器 */
    tim3_saved_psc  = TIM3->PSC;
    tim3_saved_arr  = TIM3->ARR;
    tim3_saved_ccr2 = TIM3->CCR2;
    tim3_saved_ccer = TIM3->CCER;
    tim3_saved_cr1  = TIM3->CR1 & ~TIM_CR1_CEN;  /* 原配置(不含CEN) */
}

static void tim3_restore(void)
{
    TIM3->CR1  = 0;                 /* 停 */
    TIM3->PSC  = tim3_saved_psc;
    TIM3->ARR  = tim3_saved_arr;
    TIM3->CCR2 = tim3_saved_ccr2;
    TIM3->CNT  = 0;
    TIM3->CCER = tim3_saved_ccer;
    TIM3->CR1  = tim3_saved_cr1 | TIM_CR1_CEN;  /* 恢复原配置并启动 */
}

static void tim3_uart_init(void)
{
    tim3_save();

    /* 关 CH2 输出 (PB5 不再有 PWM) */
    TIM3->CCER &= ~TIM_CCER_CC2E;

    /* 自由计数: PSC=0 ARR=0xFFFF → 16-bit @ 32MHz, 约 2ms 周期 */
    TIM3->PSC = 0;
    TIM3->ARR = 0xFFFF;
    TIM3->CNT = 0;
    TIM3->CR1 = TIM_CR1_CEN;
}

/* 硬件定时器延迟 (精度 31.25ns, 不受 CPU 总线争抢影响) */
static void tim3_delay(uint16_t ticks)
{
    uint16_t start = TIM3->CNT;
    while ((uint16_t)(TIM3->CNT - start) < ticks);
}

/*===== CRC8 (Trinamic 官方 bit-by-bit LSB-first 算法) =====
 * 对齐 ESP m_tmc2209.c calc_crc 与 TMC-API 官方实现
 * 关键: CRC 的 MSB 与 data 的 LSB 异或决定是否 XOR 多项式,
 *       data 逐位右移(LSB first), 与标准 CRC-8(MSB first) 结果不同
 * 验证: CRC([0x05]) = 0x69, CRC([0x05,0x00,0x00]) = 0x48
 *====================================================================*/

static uint8_t tmc_crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t current_byte;
    for (uint8_t i = 0; i < len; i++) {
        current_byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (current_byte & 0x01)) {
                crc = (uint8_t)(crc << 1) ^ 0x07;
            } else {
                crc = (uint8_t)(crc << 1);
            }
            current_byte >>= 1;
        }
    }
    return crc;
}

/*===== 报文构建 =====*/

static void build_write_datagram(uint8_t addr, uint8_t reg,
                                  uint32_t data, uint8_t *dg)
{
    dg[0] = TMC2209_SYNC_BYTE;
    dg[1] = addr;
    dg[2] = reg | 0x80;
    dg[3] = (uint8_t)(data >> 24);
    dg[4] = (uint8_t)(data >> 16);
    dg[5] = (uint8_t)(data >> 8);
    dg[6] = (uint8_t)(data);
    /* CRC 覆盖 sync+addr+reg+data 共 7 字节 (Trinamic 官方规范, 与 ESP m_tmc2209 一致) */
    dg[7] = tmc_crc8(&dg[0], 7);
}

static void build_read_datagram(uint8_t addr, uint8_t reg, uint8_t *dg)
{
    dg[0] = TMC2209_SYNC_BYTE;
    dg[1] = addr;
    dg[2] = reg;
    /* CRC 覆盖 sync+addr+reg 共 3 字节 (Trinamic 官方规范, 与 ESP m_tmc2209 一致) */
    dg[3] = tmc_crc8(&dg[0], 3);
}

/*===== GPIO (推挽输出, 对齐 ESP32 m_gptimer 实现) =====
 * PA7 推挽输出经 R42(1kΩ) 接 PDN_UART:
 *   - TX 驱动 HIGH/LOW, 信号干净, 上升沿快 (开漏+弱上拉上升沿太慢)
 *   - TMC2209 响应时拉低总线, 电流 3.3V/1kΩ=3.3mA, 在 TMC2209 灌入能力内
 *   - 与 ESP32 m_gptimer (GPIO_MODE_OUTPUT 推挽) 完全一致
 *====================================================================*/

static void tx_mode(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* PA7=TX, 推挽输出 (ESP32 参考实现: GPIO_MODE_OUTPUT) */
    gpio.Pin   = MOTO_TX_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTO_TX_Port, &gpio);
    BSP_GPIO_Write(MOTO_TX_Port, MOTO_TX_Pin, 1);

    /* PA6=RX, 输入上拉 */
    gpio.Pin   = MOTO_RX_Pin;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(MOTO_RX_Port, &gpio);
}

static void rx_mode(void)
{
    /* PA7 切换为输入, 完全释放总线
     * 推挽输出若保持 HIGH, TMC2209 需通过 R42(1kΩ) 灌入 3.3mA 才能拉低
     * 切输入后 TMC2209 只需驱动 PA6 上拉(~40kΩ, 82µA), 轻松拉低 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = MOTO_TX_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MOTO_TX_Port, &gpio);
}

/*===== TIM3 硬件定时 UART 收发 (直接寄存器操作) =====*/

static void uart_tx(uint8_t byte)
{
    /* 起始位 */
    TX_LOW();
    tim3_delay(TMC_BIT_TICKS);

    /* D0..D7, LSB first */
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 1) TX_HIGH(); else TX_LOW();
        tim3_delay(TMC_BIT_TICKS);
        byte >>= 1;
    }

    /* 停止位 */
    TX_HIGH();
    tim3_delay(TMC_BIT_TICKS);
}

/* TX + 同时在 PA6 采样的回环版本 (使用与 uart_tx 相同的 TX_HIGH/TX_LOW 宏)
 * 用于验证: 实际发送期间 PA6 (直连 PDN_UART) 能否正确收到信号
 * 如果 PA6 收到的字节 ≠ 发送字节 → TX 宏/时序有问题
 * 如果 PA6 收到的字节 = 发送字节 → 信号确实到达 PDN_UART, TMC2209 应能收到 */
static uint8_t uart_tx_echo(uint8_t byte)
{
    uint8_t rx_val = 0;
    uint8_t t = byte;

    TX_LOW();                          /* 起始位 */
    tim3_delay(TMC_BIT_TICKS);

    for (uint8_t i = 0; i < 8; i++) {
        if (t & 1) TX_HIGH(); else TX_LOW();
        tim3_delay(TMC_HALF_TICKS);   /* 等 half bit 到位中心 */
        if (RX_LEVEL()) rx_val |= (1 << i);
        tim3_delay(TMC_BIT_TICKS - TMC_HALF_TICKS);
        t >>= 1;
    }

    TX_HIGH();                         /* 停止位 */
    tim3_delay(TMC_BIT_TICKS);
    return rx_val;
}

static uint8_t uart_rx(uint8_t *byte, uint32_t timeout_ms)
{
    uint32_t tout = timeout_ms * 32000UL;
    uint16_t last, now;
    uint32_t total;

    /* 阶段0: 等总线回到 HIGH (滤除 rx_mode 切换毛刺) */
    total = 0;
    last  = TIM3->CNT;
    while (!RX_LEVEL()) {
        now = TIM3->CNT;
        total += (uint16_t)(now - last);
        last = now;
        if (total > tout) return 0;
    }

    /* 阶段1: 等起始位 (HIGH → LOW) */
    total = 0;
    last  = TIM3->CNT;
    while (RX_LEVEL()) {
        now = TIM3->CNT;
        total += (uint16_t)(now - last);
        last = now;
        if (total > tout) return 0;
    }

    /* 阶段2: 绝对时间点采样 (避免 tim3_delay 累积误差)
     * 从起始位下降沿: +1.5bit=D0中心, +2.5bit=D1中心, ... */
    uint16_t t_sample = TIM3->CNT + TMC_BIT_TICKS + TMC_HALF_TICKS;
    uint8_t  val = 0;

    for (uint8_t i = 0; i < 8; i++) {
        while ((uint16_t)(TIM3->CNT - t_sample) >= 0x8000);  /* 等 CNT≥t_sample */
        if (RX_LEVEL()) val |= (1 << i);
        t_sample += TMC_BIT_TICKS;
    }

    /* 停止位 */
    while ((uint16_t)(TIM3->CNT - t_sample) >= 0x8000);
    if (!RX_LEVEL()) return 0;

    *byte = val;
    return 1;
}

/*===== 原始 RX 监听: 发送请求后连续采样 PA6, 记录电平变化 =====
 * 用于判断 TMC2209 是否有任何响应 (哪怕是不完整的)
 * changes=0 → TMC2209 完全无响应 (可能未供电/掉电/未收到请求)
 * changes>0 → TMC2209 有响应, 但可能时序/电平有问题
 *================================================================*/
static void tmc_print(const char *s);       /* 前向声明 */
static void tmc_print_hex(uint32_t val, uint8_t nibbles);

static void raw_rx_monitor(uint32_t duration_ms)
{
    uint32_t tout  = duration_ms * 32000UL;
    uint16_t last  = TIM3->CNT;
    uint32_t total = 0;
    uint8_t  prev  = RX_LEVEL();
    uint8_t  changes = 0;
    uint32_t first_us = 0;

    while (total < tout) {
        uint8_t cur = RX_LEVEL();
        if (cur != prev) {
            if (changes == 0) first_us = total / 32;  /* ticks → µs */
            changes++;
            prev = cur;
        }
        uint16_t now = TIM3->CNT;
        total += (uint16_t)(now - last);
        last = now;
    }

    tmc_print("[RAW] changes=0x");
    tmc_print_hex(changes, 2);
    tmc_print(" first@");
    tmc_print_hex(first_us, 4);
    tmc_print("us\r\n");
}

/*===== 调试串口输出 =====*/

static void tmc_print(const char *s)
{
    while (*s) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = (uint8_t)(*s++);
    }
}

static void tmc_print_hex(uint32_t val, uint8_t nibbles)
{
    for (uint8_t i = nibbles; i > 0; i--) {
        uint8_t n = (uint8_t)((val >> ((i - 1) * 4)) & 0x0F);
        uint8_t c = (n < 10) ? ('0' + n) : ('A' + n - 10);
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = c;
    }
}

static void tmc_print_reg(const char *name, uint8_t reg, uint32_t val)
{
    tmc_print("  ");
    tmc_print(name);
    tmc_print(" [0x");
    tmc_print_hex(reg, 2);
    tmc_print("] = 0x");
    tmc_print_hex(val, 8);
}

/*===== 响应解析 =====
 * TMC2209 读响应: 0x05 | 0xFF | addr | data[4] | CRC
 * CRC 覆盖: sync + 0xFF + addr + data[4] 共 7 字节 (Trinamic 官方规范)
 *====================================================================*/

static uint8_t tmc_parse_response(uint8_t *resp, uint8_t len, uint32_t *data)
{
    if (len < 8) return 0;

    for (uint8_t i = 1; i + 7 <= len; i++) {
        if (resp[i] == 0xFF && resp[i - 1] == TMC2209_SYNC_BYTE) {
            /* CRC 覆盖 sync+0xFF+addr+data[4] 共 7 字节 (Trinamic 官方规范, 与 ESP m_tmc2209 一致) */
            uint8_t crc = tmc_crc8(&resp[i - 1], 7);
            if (crc != resp[i + 6]) return 0;

            *data = ((uint32_t)resp[i + 2] << 24)
                  | ((uint32_t)resp[i + 3] << 16)
                  | ((uint32_t)resp[i + 4] << 8)
                  |  (uint32_t)resp[i + 5];
            return 1;
        }
    }
    return 0;
}

/*====================================================================
 * 公开 API
 *====================================================================*/

void BSP_TMC_Init(void)
{
    tim3_uart_init();   /* 接管 TIM3, 硬件定时器替代 DWT */
    tx_mode();
    /* 不做预同步: 首次 ReadReg 的首字节 0x05 自动锁定 autobaud (与 ESP32 一致) */
}

uint8_t BSP_TMC_WriteReg(uint8_t reg, uint32_t val)
{
    uint8_t dg[8];
    build_write_datagram(TMC2209_ADDR_MOTOR, reg, val, dg);

    tx_mode();
    __disable_irq();
    for (uint8_t i = 0; i < 8; i++)
        uart_tx(dg[i]);
    __enable_irq();

    HAL_Delay(1);
    return 1;
}

uint8_t BSP_TMC_ReadReg(uint8_t reg, uint32_t *val)
{
    uint8_t dg[4];
    build_read_datagram(TMC2209_ADDR_MOTOR, reg, dg);

    /* 打印发送的数据报 (确认 CRC 和字节序) */
    tmc_print("[DBG] tx:");
    for (uint8_t i = 0; i < 4; i++) {
        tmc_print_hex(dg[i], 2);
        tmc_print(" ");
    }
    tmc_print("\r\n");

    tx_mode();
    __disable_irq();
    for (uint8_t i = 0; i < 4; i++)
        uart_tx(dg[i]);

    rx_mode();
    uint8_t resp[8];
    uint8_t rx_ok = 1;
    uint8_t rx_fail = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (!uart_rx(&resp[i], 10)) {   /* 10ms 超时/字节 */
            rx_ok = 0;
            rx_fail = i;
            break;
        }
    }

    /* 超时后原始监控 PA6 20ms, 判断 TMC2209 是否有任何响应 */
    if (!rx_ok)
        raw_rx_monitor(20);

    __enable_irq();

    if (!rx_ok) {
        tmc_print("[DBG] TIMEOUT at byte ");
        tmc_print_hex(rx_fail, 1);
        tmc_print("\r\n");
        return 0;
    }
    tmc_print("[DBG] rx:");
    for (uint8_t i = 0; i < 8; i++) {
        tmc_print_hex(resp[i], 2);
        tmc_print(" ");
    }
    tmc_print("\r\n");

    return tmc_parse_response(resp, 8, val);
}

uint8_t BSP_TMC_IsAlive(void)
{
    uint32_t dummy;
    return BSP_TMC_ReadReg(TMC2209_REG_GCONF, &dummy);
}

/* 通信结束, 归还 TIM3 给电机 PWM */
void BSP_TMC_Deinit(void)
{
    tim3_restore();
}

/* 临时切到 UART 模式 (保存 PWM, 启动自由计数) */
void BSP_TMC_UARTMode(void)
{
    tim3_save();
    tim3_uart_init();
    tx_mode();
}

/* 恢复 PWM 模式 */
void BSP_TMC_PWMMode(void)
{
    tim3_restore();
}

/* 探测指定地址的 TMC2209: 发送 GCONF 读请求, 检查 10ms 内是否有任何响应
 * 返回: 1=有响应(检测到下降沿) 0=无响应
 * 用于地址扫描: MS1/MS2 焊接不良可能导致地址不是 0x00 */
static uint8_t tmc_probe_addr(uint8_t addr)
{
    uint8_t dg[4];
    dg[0] = TMC2209_SYNC_BYTE;
    dg[1] = addr;
    dg[2] = TMC2209_REG_GCONF;
    dg[3] = tmc_crc8(dg, 3);

    tx_mode();
    __disable_irq();
    for (uint8_t i = 0; i < 4; i++)
        uart_tx(dg[i]);
    rx_mode();

    /* 10ms 内检查 PA6 是否出现下降沿 (TMC2209 响应的起始位) */
    uint16_t last  = TIM3->CNT;
    uint32_t total = 0;
    uint32_t tout  = 10 * 32000UL;
    uint8_t  resp  = 0;
    while (total < tout) {
        if (!RX_LEVEL()) { resp = 1; break; }
        uint16_t now = TIM3->CNT;
        total += (uint16_t)(now - last);
        last = now;
    }
    __enable_irq();
    return resp;
}

/*====================================================================
 * TMC2209 独立测试
 *   流程: TIM3接管 → 自环回 → PDN电源循环 → TX回环验证 → 地址扫描 → 写配置 → 读回
 *   返回值: 'T'=全通 'W'=写OK读失败 '0'=不在线
 *====================================================================*/
uint8_t BSP_TMC_Test(void)
{
    tmc_print("\r\n========== TMC2209 Test ==========\r\n");

    /*── 步骤0: TIM3 接管 + 自环回验证 ──*/
    tmc_print("[0] TIM3 UART init... ");
    tim3_uart_init();
    tx_mode();
    tmc_print("OK\r\n");

    tmc_print("[0b] Loopback PA7->PA6... ");
    {
        __disable_irq();
        uint8_t echo = 0;

        BSP_GPIO_Write(MOTO_TX_Port, MOTO_TX_Pin, 0);  /* 起始位 */
        tim3_delay(TMC_BIT_TICKS);

        uint8_t tb = 0xA5;
        for (uint8_t i = 0; i < 8; i++) {
            BSP_GPIO_Write(MOTO_TX_Port, MOTO_TX_Pin, tb & 1);
            tim3_delay(TMC_HALF_TICKS);
            if (BSP_GPIO_Read(MOTO_RX_Port, MOTO_RX_Pin))
                echo |= (1 << i);
            tim3_delay(TMC_BIT_TICKS - TMC_HALF_TICKS);
            tb >>= 1;
        }

        BSP_GPIO_Write(MOTO_TX_Port, MOTO_TX_Pin, 1);
        tim3_delay(TMC_BIT_TICKS);
        __enable_irq();

        tmc_print("sent=0xA5 echo=0x"); tmc_print_hex(echo, 2);
        if (echo == 0xA5) tmc_print(" OK\r\n");
        else { tmc_print(" FAIL\r\n"); tim3_restore(); return '0'; }
    }

    /*── 步骤1: PDN 电源循环 (复位 autobaud!) ──
     * 关键: TMC2209 autobaud 一旦锁定就不再重新锁定 (除非掉电重启)!
     * 之前 5 轮测试 (DWT 等) 可能用错误波特率锁定了 autobaud,
     * 之后即使波特率正确(9600), TMC2209 也不接收 → changes=0x00
     *
     * 方案: 拉 PDN_UART LOW >2^18 时钟(~22ms) 触发掉电 → 释放 HIGH 唤醒
     * 掉电后 autobaud 复位, 首次收到的 0x05 SYNC 重新锁定正确波特率
     *
     * 之前的 "预充电" 只拉 HIGH 不触发掉电, 无法复位 autobaud! */
    tmc_print("[1] PDN power-cycle... ");
    BSP_GPIO_Write(MOTO_TX_Port, MOTO_TX_Pin, 0);  /* PA7=LOW → PDN_UART=LOW via R42 */
    HAL_Delay(50);                                  /* >21ms 触发掉电, 复位 autobaud */
    BSP_GPIO_Write(MOTO_TX_Port, MOTO_TX_Pin, 1);  /* PA7=HIGH → 唤醒 */
    HAL_Delay(100);                                 /* 等待振荡器稳定 (~1ms) + 余量 */
    tmc_print("OK\r\n");

    /*── 步骤2: PA6 空闲电平检测 ──
     * 不做预同步! ESP32 m_tmc2209.c 直接发读请求, 首字节 0x05 同时锁定
     * autobaud 并作为 datagram SYNC, 无需额外预同步字节 */
    tmc_print("[2] PA6 idle... ");
    rx_mode();
    HAL_Delay(1);
    tmc_print("PA6=");
    tmc_print_hex(RX_LEVEL(), 1);
    tmc_print("\r\n");
    tx_mode();

    /*── 步骤2b: TX 回环验证 (用实际读请求字节 + TX_HIGH/TX_LOW 宏) ──
     * 关键: 步骤0b 的环回用的是 BSP_GPIO_Write/Read (HAL 函数),
     * 而 uart_tx 用的是 TX_HIGH/TX_LOW (直接寄存器宏).
     * 本步骤用相同宏发送 0x05 00 00 48, 同时在 PA6 采样, 验证:
     *   1) TX_HIGH/TX_LOW 宏确实驱动了 PA7
     *   2) 信号经 R42 到达 PDN_UART (PA6 能读到)
     *   3) 4 字节连续发送的时序正确 */
    tmc_print("[2b] TX echo (read req bytes)...\r\n");
    {
        uint8_t dg[4] = {0x05, 0x00, 0x00, 0x48};
        uint8_t echo[4];

        __disable_irq();
        for (uint8_t i = 0; i < 4; i++)
            echo[i] = uart_tx_echo(dg[i]);
        __enable_irq();

        tmc_print("  tx: 05 00 00 48\r\n");
        tmc_print("  rx: ");
        for (uint8_t i = 0; i < 4; i++) {
            tmc_print_hex(echo[i], 2);
            tmc_print(" ");
        }
        tmc_print("\r\n");

        uint8_t tx_ok = 1;
        for (uint8_t i = 0; i < 4; i++) {
            if (echo[i] != dg[i]) { tx_ok = 0; break; }
        }
        if (tx_ok) {
            tmc_print("  TX echo OK (signal reaches PDN_UART)\r\n");
        } else {
            tmc_print("  TX echo FAIL (signal NOT reaching PDN_UART!)\r\n");
            tmc_print("========== RESULT: TX FAIL ==========\r\n");
            tim3_restore();
            return '0';
        }
    }

    /*── 步骤3: 地址扫描 (0x00-0x03) ──
     * MS1_AD0/MS2_AD1 下拉电阻焊接不良可能导致地址 ≠ 0x00
     * 逐个尝试所有 4 个地址, 报告哪个有响应 */
    tmc_print("[3] Address scan 0x00-0x03...\r\n");
    {
        uint8_t found_addr = 0xFF;
        for (uint8_t addr = 0; addr < 4; addr++) {
            tmc_print("  [0x");
            tmc_print_hex(addr, 2);
            tmc_print("]...");
            if (tmc_probe_addr(addr)) {
                tmc_print(" RESPONSE!\r\n");
                found_addr = addr;
                break;
            } else {
                tmc_print(" no response\r\n");
            }
            HAL_Delay(5);
        }

        if (found_addr == 0xFF) {
            tmc_print("  No TMC2209 at any address.\r\n");
            tmc_print("  TX verified OK but TMC2209 never drives bus.\r\n");
            tmc_print("  -> Check hardware: 24V supply, 5VOUT, VCC_IO, soldering\r\n");
            tmc_print("========== RESULT: no TMC2209 ==========\r\n");
            tim3_restore();
            return '0';
        }

        tmc_print("  Found at addr 0x");
        tmc_print_hex(found_addr, 2);
        tmc_print("\r\n");

        /* 用找到的地址读 GCONF */
        HAL_Delay(5);
        uint32_t raw = 0;
        if (BSP_TMC_ReadReg(TMC2209_REG_GCONF, &raw)) {
            tmc_print("  GCONF=0x"); tmc_print_hex(raw, 8); tmc_print("\r\n");
        } else {
            tmc_print("  GCONF read failed (probe said response but ReadReg timeout)\r\n");
        }
    }

    /*── 步骤4: 写配置 ──*/
    HAL_Delay(10);
    tmc_print("[4] Write config registers:\r\n");

    GCONF_reg_t gconf = {0};
    gconf.bits.I_scale_analog   = 0;   /* 内部5V基准, 忽略VREF */
    gconf.bits.internal_Rsense  = 0;
    gconf.bits.en_SpreadCycle   = 0;   /* StealthChop */
    gconf.bits.shaft            = 0;
    gconf.bits.index_otpw       = 1;
    gconf.bits.index_step       = 0;
    gconf.bits.pdn_disable      = 1;
    gconf.bits.mstep_reg_select = 1;
    gconf.bits.multistep_filt   = 1;
    gconf.bits.test_mode        = 0;
    BSP_TMC_WriteReg(TMC2209_REG_GCONF, gconf.raw);
    tmc_print_reg("GCONF     ", TMC2209_REG_GCONF, gconf.raw);
    tmc_print("\r\n");

    IHOLD_IRUN_reg_t ihold_irun = {0};
    ihold_irun.bits.irun       = 16;   /* 最大 ~1.5A */
    ihold_irun.bits.ihold      = 8;
    ihold_irun.bits.iholddelay = 7;
    BSP_TMC_WriteReg(TMC2209_REG_IHOLD_IRUN, ihold_irun.raw);
    tmc_print_reg("IHOLD_IRUN", TMC2209_REG_IHOLD_IRUN, ihold_irun.raw);
    tmc_print("\r\n");

    TPOWERDOWN_reg_t tpowerdown = {0};
    tpowerdown.bits.tpowerdown = 20;
    BSP_TMC_WriteReg(TMC2209_REG_TPOWERDOWN, tpowerdown.raw);
    tmc_print_reg("TPOWERDOWN", TMC2209_REG_TPOWERDOWN, tpowerdown.raw);
    tmc_print("\r\n");

    CHOPCONF_reg_t chopconf = {0};
    chopconf.bits.toff      = 3;
    chopconf.bits.hstrt     = 4;
    chopconf.bits.hend      = 1;
    chopconf.bits.tbl       = 2;
    chopconf.bits.vsense    = 1;
    chopconf.bits.mres      = 4; /* 16微步 */
    chopconf.bits.intpol    = 0; /* 关插值 */
    chopconf.bits.dedge     = 0;
    chopconf.bits.diss2g    = 0;
    chopconf.bits.diss2vs   = 0;
    BSP_TMC_WriteReg(TMC2209_REG_CHOPCONF, chopconf.raw);
    tmc_print_reg("CHOPCONF  ", TMC2209_REG_CHOPCONF, chopconf.raw);
    tmc_print("\r\n");

    PWMCONF_reg_t pwmconf = {0};
    pwmconf.bits.pwm_ofs       = 36;
    pwmconf.bits.pwm_grad      = 14;
    pwmconf.bits.pwm_freq      = 1;
    pwmconf.bits.pwm_autoscale = 1;
    pwmconf.bits.pwm_autograd  = 1;
    pwmconf.bits.freewheel     = 1;
    pwmconf.bits.pwm_reg       = 8;
    pwmconf.bits.pwm_lim       = 12;
    BSP_TMC_WriteReg(TMC2209_REG_PWMCONF, pwmconf.raw);
    tmc_print_reg("PWMCONF   ", TMC2209_REG_PWMCONF, pwmconf.raw);
    tmc_print("\r\n");

    tmc_print("  5 registers written.\r\n");

    /*── 步骤5: 读回验证 ──*/
    HAL_Delay(30);
    tmc_print("[5] Read-back GCONF... ");

    uint32_t gconf_read = 0;
    if (BSP_TMC_ReadReg(TMC2209_REG_GCONF, &gconf_read)) {
        tmc_print("0x"); tmc_print_hex(gconf_read, 8);
        if (gconf_read == gconf.raw) {
            tmc_print(" (match)\r\n");

            /* TIM3 仍为自由计数模式, 无 PWM 干扰, 验证全部配置 */
            tmc_print("[5b] Verify all registers:\r\n");
            {
                uint32_t v;
                if (BSP_TMC_ReadReg(0x06, &v))
                    { tmc_print("  IOIN       =0x"); tmc_print_hex(v,8);
                      tmc_print(" ENN="); tmc_print_hex(v&1,1);
                      tmc_print(" DIR="); tmc_print_hex((v>>9)&1,1);
                      tmc_print(" STEP="); tmc_print_hex((v>>7)&1,1);
                      tmc_print("\r\n"); }
                else
                    { tmc_print("  IOIN read FAIL\r\n"); }
                if (BSP_TMC_ReadReg(0x10, &v))
                    { tmc_print("  IHOLD_IRUN =0x"); tmc_print_hex(v,8); tmc_print("\r\n"); }
                else
                    { tmc_print("  IHOLD_IRUN read FAIL\r\n"); }
                if (BSP_TMC_ReadReg(0x6C, &v))
                    { tmc_print("  CHOPCONF   =0x"); tmc_print_hex(v,8); tmc_print("\r\n"); }
                else
                    { tmc_print("  CHOPCONF read FAIL\r\n"); }
                if (BSP_TMC_ReadReg(0x70, &v))
                    { tmc_print("  PWMCONF    =0x"); tmc_print_hex(v,8); tmc_print("\r\n"); }
                else
                    { tmc_print("  PWMCONF read FAIL\r\n"); }
                if (BSP_TMC_ReadReg(0x01, &v))
                    { tmc_print("  GSTAT      =0x"); tmc_print_hex(v,8);
                      if (v) tmc_print(" *** FAULT ***"); tmc_print("\r\n"); }
                else
                    { tmc_print("  GSTAT read FAIL\r\n"); }
                if (BSP_TMC_ReadReg(0x6F, &v))
                    { tmc_print("  DRV_STATUS =0x"); tmc_print_hex(v,8);
                      if (v & (1<<6))  tmc_print(" OLA");   /* Open Load A */
                      if (v & (1<<7))  tmc_print(" OLB");   /* Open Load B */
                      if (v & (1<<2))  tmc_print(" S2GA");  /* Short A */
                      if (v & (1<<3))  tmc_print(" S2GB");  /* Short B */
                      if (v & (1<<0))  tmc_print(" OT");    /* Overtemp */
                      tmc_print("\r\n"); }
                else
                    { tmc_print("  DRV_STATUS read FAIL\r\n"); }
            }

            tmc_print("========== RESULT: PASS (R/W) ==========\r\n");

            /* 清 GSTAT 故障标志 (写1清除 latched bits) */
            BSP_TMC_WriteReg(0x01, 0x00000007);

            tim3_restore();
            return 'T';
        } else {
            tmc_print(" (MISMATCH)\r\n");
            tmc_print("========== RESULT: W=OK R=MISMATCH ==========\r\n");
            tim3_restore();
            return 'W';
        }
    } else {
        tmc_print("no response\r\n");
        tmc_print("  (writes assumed OK, check motor movement)\r\n");
        tmc_print("========== RESULT: W=OK R=FAIL ==========\r\n");
        tim3_restore();
        return 'W';
    }
}

void BSP_TMC_SetDefaults(void)
{
    GCONF_reg_t gconf = {0};
    gconf.bits.I_scale_analog   = 1;   /* 内部5V基准, 忽略VREF */
    gconf.bits.internal_Rsense  = 0;
    gconf.bits.en_SpreadCycle   = 0;
    gconf.bits.shaft            = 0;
    gconf.bits.index_otpw       = 1;
    gconf.bits.index_step       = 0;
    gconf.bits.pdn_disable      = 1;
    gconf.bits.mstep_reg_select = 1;
    gconf.bits.multistep_filt   = 1;
    gconf.bits.test_mode        = 0;
    BSP_TMC_WriteReg(TMC2209_REG_GCONF, gconf.raw);

    IHOLD_IRUN_reg_t ihold_irun = {0};
    ihold_irun.bits.irun       = 24;
    ihold_irun.bits.ihold      = 12;
    ihold_irun.bits.iholddelay = 3;
    BSP_TMC_WriteReg(TMC2209_REG_IHOLD_IRUN, ihold_irun.raw);

    TPOWERDOWN_reg_t tpowerdown = {0};
    tpowerdown.bits.tpowerdown = 20;
    BSP_TMC_WriteReg(TMC2209_REG_TPOWERDOWN, tpowerdown.raw);

    CHOPCONF_reg_t chopconf = {0};
    chopconf.bits.toff      = 3;
    chopconf.bits.hstrt     = 4;
    chopconf.bits.hend      = 1;
    chopconf.bits.tbl       = 2;
    chopconf.bits.vsense    = 1;
    chopconf.bits.mres      = 4;
    chopconf.bits.intpol    = 1;
    chopconf.bits.dedge     = 0;
    chopconf.bits.diss2g    = 0;
    chopconf.bits.diss2vs   = 0;
    BSP_TMC_WriteReg(TMC2209_REG_CHOPCONF, chopconf.raw);

    PWMCONF_reg_t pwmconf = {0};
    pwmconf.bits.pwm_ofs       = 36;
    pwmconf.bits.pwm_grad      = 14;
    pwmconf.bits.pwm_freq      = 1;
    pwmconf.bits.pwm_autoscale = 1;
    pwmconf.bits.pwm_autograd  = 1;
    pwmconf.bits.freewheel     = 1;
    pwmconf.bits.pwm_reg       = 8;
    pwmconf.bits.pwm_lim       = 12;
    BSP_TMC_WriteReg(TMC2209_REG_PWMCONF, pwmconf.raw);
}
