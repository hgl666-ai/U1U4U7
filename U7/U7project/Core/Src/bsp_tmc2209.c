#include "bsp_tmc2209.h"
#include "bsp_gpio.h"

/*===== 软件 UART 常量 (64MHz / 9600 ≈ 6667 cycles/bit) =====*/
#define BIT_CYCLES    6667
#define HALF_CYCLES   3333


static uint8_t tmc_crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (uint8_t)(crc << 1) ^ 0x07;
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/*===== 报文构建 (ESP 样板) =====*/

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
    dg[7] = tmc_crc8(&dg[1], 6);   /* CRC 覆盖 addr→D0, 不含同步字节 */
}

static void build_read_datagram(uint8_t addr, uint8_t reg, uint8_t *dg)
{
    dg[0] = TMC2209_SYNC_BYTE;
    dg[1] = addr;
    dg[2] = reg;
    dg[3] = tmc_crc8(&dg[1], 2);   /* CRC 覆盖 addr+reg, 不含同步字节 */
}

/*===== DWT 周期计时 (与原有 bsp_tmc2209 一致) =====*/

static void delay_cycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/*===== GPIO 方向切换 =====*/

static void tx_mode(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = MOTO_TX_Pin;            /* PA7=TX */
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTO_TX_Port, &gpio);
    BSP_GPIO_Write(MOTO_TX_Port, MOTO_TX_Pin, 1);
}

static void rx_mode(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* PA6=RX, 切输入 */
    gpio.Pin   = MOTO_RX_Pin;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(MOTO_RX_Port, &gpio);

    /* PA7=TX, 切输入高阻, 避免推挽输出经 R42(1kΩ) 上拉总线
     * 与 TMC2209 PDN_UART 输出产生分压, 导致 PA6 读不到起始位
     */
    gpio.Pin   = MOTO_TX_Pin;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(MOTO_TX_Port, &gpio);
}

/*===== 软件 UART 收发 =====*/

static void uart_tx(uint8_t byte)
{
    /* 起始位 (PA7=TX) */
    BSP_GPIO_Write(MOTO_TX_Port, MOTO_TX_Pin, 0);
    delay_cycles(BIT_CYCLES);

    /* D0..D7, LSB first */
    for (uint8_t i = 0; i < 8; i++) {
        BSP_GPIO_Write(MOTO_TX_Port, MOTO_TX_Pin, byte & 1);
        delay_cycles(BIT_CYCLES);
        byte >>= 1;
    }

    /* 停止位 */
    BSP_GPIO_Write(MOTO_TX_Port, MOTO_TX_Pin, 1);
    delay_cycles(BIT_CYCLES);
}

static uint8_t uart_rx(uint8_t *byte, uint32_t timeout_us)
{
    uint32_t start   = DWT->CYCCNT;
    uint32_t tout_cyc = timeout_us * 64;   /* 64 周期/μs @ 64MHz */

    /* 等下降沿 (起始位) — 读 PA6=RX */
    while (BSP_GPIO_Read(MOTO_RX_Port, MOTO_RX_Pin)) {
        if ((DWT->CYCCNT - start) > tout_cyc) return 0;
    }

    /* 延时到起始位中心，再跳到 D0 中心 */
    delay_cycles(HALF_CYCLES + BIT_CYCLES);

    uint8_t val = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (BSP_GPIO_Read(MOTO_RX_Port, MOTO_RX_Pin))
            val |= (1 << i);
        delay_cycles(BIT_CYCLES);
    }

    delay_cycles(HALF_CYCLES);   /* 等停止位 */
    *byte = val;
    return 1;
}

/*====================================================================
 * 响应解析回调 (对应 ESP m_tmc2209_rx_callback)
 *
 * TMC2209 读响应帧 (8 字节, 见数据手册 UART Single Wire Interface):
 *   0x05 | 0xFF | addr | [data 31:24] [23:16] [15:8] [7:0] | CRC
 *
 * CRC 覆盖: 0xFF + addr + data[4] 共 6 字节
 *====================================================================*/
static uint8_t tmc_parse_response(uint8_t *resp, uint8_t len, uint32_t *data)
{
    if (len < 8) return 0;

    /* 跳过前导 0x05，搜索 0xFF (响应头) */
    for (uint8_t i = 1; i + 7 <= len; i++) {
        if (resp[i] == 0xFF && resp[i - 1] == TMC2209_SYNC_BYTE) {
            uint8_t crc = tmc_crc8(&resp[i], 6);   /* CRC 覆盖 0xFF+addr+data[4] */
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

/**
  * @brief  软件 UART + DWT 初始化
  */
void BSP_TMC_Init(void)
{
    dwt_init();
    tx_mode();

    /* 发同步字节让 TMC2209 锁定波特率 (关中断保证时序) */
    __disable_irq();
    for (uint8_t i = 0; i < 10; i++)
        uart_tx(0x00);
    __enable_irq();
    HAL_Delay(5);
}

/**
  * @brief  写 TMC2209 寄存器
  */
uint8_t BSP_TMC_WriteReg(uint8_t reg, uint32_t val)
{
    uint8_t dg[8];
    build_write_datagram(TMC2209_ADDR_MOTOR, reg, val, dg);

    tx_mode();
    __disable_irq();                        /* 关中断保证时序 */
    for (uint8_t i = 0; i < 8; i++)
        uart_tx(dg[i]);
    __enable_irq();

    HAL_Delay(1);   /* TMC2209 帧间最小间隔 */
    return 1;
}

/**
  * @brief  读 TMC2209 寄存器
  */
uint8_t BSP_TMC_ReadReg(uint8_t reg, uint32_t *val)
{
    uint8_t dg[4];
    build_read_datagram(TMC2209_ADDR_MOTOR, reg, dg);

    /* 发送读请求 (关中断保证软件UART时序) */
    tx_mode();
    __disable_irq();
    for (uint8_t i = 0; i < 4; i++)
        uart_tx(dg[i]);

    /* 切 RX，收 8 字节响应 (TMC2209 读响应帧固定 8 字节) */
    rx_mode();
    uint8_t resp[8];
    for (uint8_t i = 0; i < 8; i++) {
        if (!uart_rx(&resp[i], 10000)) {
            __enable_irq();
            return 0;
        }
    }
    __enable_irq();

    return tmc_parse_response(resp, 8, val);
}

/**
  * @brief  检测 TMC2209 是否在线
  */
uint8_t BSP_TMC_IsAlive(void)
{
    uint32_t dummy;
    return BSP_TMC_ReadReg(TMC2209_REG_GCONF, &dummy);
}

/**
  * @brief  自测: 初始化+写默认配置+读回GCONF验证
  * @retval 1=通过, 0=读回失败
  */
/**
  * @brief  自测: 初始化+写默认配置 (只写不读, 防中断干扰软件UART)
  * @retval 'T'=配置完成 (信任写入)
  * @note   软件UART受SysTick/DMA中断干扰, 运行时禁止读写TMC2209
  */
/**
  * @brief  自测: 初始化+写默认配置 (只写不读, 写时关中断防时序错乱)
  * @retval 'T'=配置完成
  * @note   软件UART受中断干扰, 运行时禁止读写TMC2209
  */
/**
  * @brief  自测: 关中断全流程 初始化→写配置→读回GCONF
  * @retval 'T'=读写成功 '1'=无响应 '2'=CRC错
  * @note   全程禁中断 ~150ms, 验证TMC2209双向通信
  */
/**
  * @brief  自测: 初始化+写配置+读回GCONF
  * @retval 'T'=读写成功 '1'=无响应 '2'=CRC错
  */
uint8_t BSP_TMC_Test(void)
{
    BSP_TMC_Init();
    BSP_TMC_SetDefaults();

    /* 读回 GCONF (BSP_TMC_ReadReg内部关中断防时序乱) */
    uint32_t gconf = 0;
    if (!BSP_TMC_ReadReg(TMC2209_REG_GCONF, &gconf))
        return '1';

    return (gconf != 0) ? 'T' : '2';
}

/**
  * @brief  写 TMC2209 默认配置 (ESP 样板风格, 用 union 位域)
  */
void BSP_TMC_SetDefaults(void)
{
    GCONF_reg_t gconf = {0};
    gconf.bits.I_scale_analog   = 0;   /* VREF 模拟调流 */
    gconf.bits.internal_Rsense  = 0;   /* 外部采样电阻 */
    gconf.bits.en_SpreadCycle   = 0;   /* StealthChop */
    gconf.bits.shaft            = 0;   /* 正转 */
    gconf.bits.index_otpw       = 1;   /* INDEX=过温预警 */
    gconf.bits.index_step       = 0;
    gconf.bits.pdn_disable      = 1;   /* UART 控制 PDN */
    gconf.bits.mstep_reg_select = 1;   /* 微步=MSTEP 寄存器 */
    gconf.bits.multistep_filt   = 1;
    gconf.bits.test_mode        = 0;
    BSP_TMC_WriteReg(TMC2209_REG_GCONF, gconf.raw);

    IHOLD_IRUN_reg_t ihold_irun = {0};
    ihold_irun.bits.irun       = 24;   /* ~1.2A RMS */
    ihold_irun.bits.ihold      = 12;   /* ~0.6A */
    ihold_irun.bits.iholddelay = 3;
    BSP_TMC_WriteReg(TMC2209_REG_IHOLD_IRUN, ihold_irun.raw);

    TPOWERDOWN_reg_t tpowerdown = {0};
    tpowerdown.bits.tpowerdown = 20;    /* ~0.5s @ 12MHz */
    BSP_TMC_WriteReg(TMC2209_REG_TPOWERDOWN, tpowerdown.raw);

    CHOPCONF_reg_t chopconf = {0};
    chopconf.bits.toff      = 3;        /* 必须 > 0 */
    chopconf.bits.hstrt     = 4;
    chopconf.bits.hend      = 1;
    chopconf.bits.tbl       = 2;        /* 36 clk blank */
    chopconf.bits.vsense    = 1;        /* 0.180V */
    chopconf.bits.mres      = 4;        /* 16 微步 */
    chopconf.bits.intpol    = 1;        /* 插值到 256 */
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
