#ifndef __BSP_TMC2209_H
#define __BSP_TMC2209_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*===== 从机地址 =====*/
#define TMC2209_SLAVE_ADDR0     ((uint8_t)0x00)
#define TMC2209_SLAVE_ADDR1     ((uint8_t)0x01)
#define TMC2209_SLAVE_ADDR2     ((uint8_t)0x02)
#define TMC2209_SLAVE_ADDR3     ((uint8_t)0x03)

#define TMC2209_ADDR_MOTOR      TMC2209_SLAVE_ADDR0  /* U7 仅 1 路电机 */
#define TMC2209_SYNC_BYTE       ((uint8_t)0x05)

/*===== 寄存器地址 =====*/
#define TMC2209_REG_GCONF         ((uint8_t)0x00)
#define TMC2209_REG_GSTAT         ((uint8_t)0x01)
#define TMC2209_REG_IFCNT         ((uint8_t)0x02)
#define TMC2209_REG_SLAVECONF     ((uint8_t)0x03)
#define TMC2209_REG_OTP_PROG      ((uint8_t)0x04)
#define TMC2209_REG_OTP_READ      ((uint8_t)0x05)
#define TMC2209_REG_IOIN          ((uint8_t)0x06)
#define TMC2209_REG_FACTORY_CONF  ((uint8_t)0x07)
#define TMC2209_REG_IHOLD_IRUN    ((uint8_t)0x10)
#define TMC2209_REG_TPOWERDOWN    ((uint8_t)0x11)
#define TMC2209_REG_TSTEP         ((uint8_t)0x12)
#define TMC2209_REG_TPWMTHRS      ((uint8_t)0x13)
#define TMC2209_REG_TCOOLTHRS     ((uint8_t)0x14)
#define TMC2209_REG_VACTUAL       ((uint8_t)0x22)
#define TMC2209_REG_SGTHRS        ((uint8_t)0x40)
#define TMC2209_REG_SG_RESULT     ((uint8_t)0x41)
#define TMC2209_REG_COOLCONF      ((uint8_t)0x42)
#define TMC2209_REG_MSCNT         ((uint8_t)0x6A)
#define TMC2209_REG_MSCURACT      ((uint8_t)0x6B)
#define TMC2209_REG_CHOPCONF      ((uint8_t)0x6C)
#define TMC2209_REG_DRV_STATUS    ((uint8_t)0x6F)
#define TMC2209_REG_PWMCONF       ((uint8_t)0x70)
#define TMC2209_REG_PWM_SCALE     ((uint8_t)0x71)
#define TMC2209_REG_PWM_AUTO      ((uint8_t)0x72)

/*===== 寄存器位域定义 (union, 与 ESP 样板一致) =====*/

typedef union {
    uint32_t raw;
    struct {
        uint32_t I_scale_analog   : 1;
        uint32_t internal_Rsense  : 1;
        uint32_t en_SpreadCycle   : 1;
        uint32_t shaft            : 1;
        uint32_t index_otpw       : 1;
        uint32_t index_step       : 1;
        uint32_t pdn_disable      : 1;
        uint32_t mstep_reg_select : 1;
        uint32_t multistep_filt   : 1;
        uint32_t test_mode        : 1;
        uint32_t reserved         : 22;
    } bits;
} GCONF_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t reset      : 1;
        uint32_t drv_err    : 1;
        uint32_t uv_cp      : 1;
        uint32_t reserved   : 29;
    } bits;
} GSTAT_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t ifcnt     : 8;
        uint32_t reserved  : 24;
    } bits;
} IFCNT_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t senddelay : 4;
        uint32_t reserved  : 28;
    } bits;
} SLAVECONF_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t otpbit   : 3;
        uint32_t reserved1: 1;
        uint32_t otpbyte  : 2;
        uint32_t reserved2: 2;
        uint32_t otpmagic : 8;
        uint32_t reserved3: 16;
    } bits;
} OTP_PROG_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t otp0     : 8;
        uint32_t otp1     : 8;
        uint32_t otp2     : 8;
        uint32_t reserved : 8;
    } bits;
} OTP_READ_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t enn       : 1;
        uint32_t reserved1 : 1;
        uint32_t ms1       : 1;
        uint32_t ms2       : 1;
        uint32_t diag      : 1;
        uint32_t reserved2 : 1;
        uint32_t pdn_uart  : 1;
        uint32_t step      : 1;
        uint32_t spread_en : 1;
        uint32_t dir       : 1;
        uint32_t reserved3 : 22;
    } bits;
} IOIN_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t fclktrim : 5;
        uint32_t reserved1: 3;
        uint32_t ottrim   : 2;
        uint32_t reserved2: 22;
    } bits;
} FACTORY_CONF_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t ihold      : 5;
        uint32_t reserved1  : 3;
        uint32_t irun       : 5;
        uint32_t reserved2  : 3;
        uint32_t iholddelay : 4;
        uint32_t reserved3  : 12;
    } bits;
} IHOLD_IRUN_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t tpowerdown : 8;
        uint32_t reserved   : 24;
    } bits;
} TPOWERDOWN_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t tstep    : 20;
        uint32_t reserved : 12;
    } bits;
} TSTEP_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t tpwmthrs : 20;
        uint32_t reserved : 12;
    } bits;
} TPWMTHRS_reg_t;

typedef union {
    uint32_t raw;
    struct {
        int32_t vactual   : 24;
        int32_t reserved  : 8;
    } bits;
} VACTUAL_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t tcoolthrs : 20;
        uint32_t reserved  : 12;
    } bits;
} TCOOLTHRS_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t sgthrs   : 8;
        uint32_t reserved : 24;
    } bits;
} SGTHRS_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t sg_result : 10;
        uint32_t reserved  : 22;
    } bits;
} SG_RESULT_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t semin    : 4;
        uint32_t reserved1: 1;
        uint32_t seup     : 2;
        uint32_t reserved2: 1;
        uint32_t semax    : 4;
        uint32_t reserved3: 1;
        uint32_t sedn     : 2;
        uint32_t seimin   : 1;
        uint32_t reserved4: 16;
    } bits;
} COOLCONF_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t mscnt    : 10;
        uint32_t reserved : 22;
    } bits;
} MSCNT_reg_t;

typedef union {
    uint32_t raw;
    struct {
        int32_t cur_a    : 9;
        int32_t reserved1: 7;
        int32_t cur_b    : 9;
        int32_t reserved2: 7;
    } bits;
} MSCURACT_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t toff      : 4;
        uint32_t hstrt     : 3;
        uint32_t hend      : 4;
        uint32_t reserved1 : 4;
        uint32_t tbl       : 2;
        uint32_t vsense    : 1;
        uint32_t reserved2 : 6;
        uint32_t mres      : 4;
        uint32_t intpol    : 1;
        uint32_t dedge     : 1;
        uint32_t diss2g    : 1;
        uint32_t diss2vs   : 1;
    } bits;
} CHOPCONF_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t ot       : 1;
        uint32_t otpw     : 1;
        uint32_t s2ga     : 1;
        uint32_t s2gb     : 1;
        uint32_t s2vsa    : 1;
        uint32_t s2vsb    : 1;
        uint32_t ola      : 1;
        uint32_t olb      : 1;
        uint32_t t120     : 1;
        uint32_t t143     : 1;
        uint32_t t150     : 1;
        uint32_t t157     : 1;
        uint32_t reserved1: 4;
        uint32_t cs_actual: 5;
        uint32_t reserved2: 3;
        uint32_t stealth  : 1;
        uint32_t stst     : 1;
        uint32_t reserved3: 6;
    } bits;
} DRV_STATUS_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t pwm_ofs      : 8;
        uint32_t pwm_grad     : 8;
        uint32_t pwm_freq     : 2;
        uint32_t pwm_autoscale: 1;
        uint32_t pwm_autograd : 1;
        uint32_t freewheel    : 2;
        uint32_t reserved1    : 2;
        uint32_t pwm_reg      : 4;
        uint32_t pwm_lim      : 4;
    } bits;
} PWMCONF_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t pwm_scale_sum : 8;
        uint32_t reserved1     : 8;
        int32_t  pwm_scale_auto: 9;
        uint32_t reserved2     : 7;
    } bits;
} PWM_SCALE_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t pwm_ofs_auto : 8;
        uint32_t reserved1    : 8;
        uint32_t pwm_grad_auto: 8;
        uint32_t reserved2    : 8;
    } bits;
} PWM_AUTO_reg_t;

/*===== API =====*/
uint8_t  BSP_TMC_Test(void);          /* 自测: T=通 1=无响应 2=CRC错 */
void     BSP_TMC_Init(void);
void     BSP_TMC_SetDefaults(void);
uint8_t  BSP_TMC_WriteReg(uint8_t reg, uint32_t val);
uint8_t  BSP_TMC_ReadReg(uint8_t reg, uint32_t *val);
uint8_t  BSP_TMC_IsAlive(void);

#endif
