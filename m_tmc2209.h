#ifndef __M_TMC2209_H__
#define __M_TMC2209_H__

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define TMC2209_SLAVE_ADDR0     ((uint8_t)0x00)
#define TMC2209_SLAVE_ADDR1     ((uint8_t)0x01)
#define TMC2209_SLAVE_ADDR2     ((uint8_t)0x02)
#define TMC2209_SLAVE_ADDR3     ((uint8_t)0x03)

#define TMC2209_ADDR_L    TMC2209_SLAVE_ADDR0
#define TMC2209_ADDR_S    TMC2209_SLAVE_ADDR2
#define TMC2209_ADDR_B    TMC2209_SLAVE_ADDR1

#define TMC2209_SYNC_BYTE       ((uint8_t)0x05)  /* 同步字节，读写固定填开头 */

/* GENERAL CONFIGURATION REGISTERS (0x00…0x0F) */
#define TMC2209_REG_GCONF         ((uint8_t)0x00)  // RW
#define TMC2209_REG_GSTAT         ((uint8_t)0x01)  // R+WC
#define TMC2209_REG_IFCNT         ((uint8_t)0x02)  // R
#define TMC2209_REG_SLAVECONF     ((uint8_t)0x03)  // W
#define TMC2209_REG_OTP_PROG      ((uint8_t)0x04)  // W
#define TMC2209_REG_OTP_READ      ((uint8_t)0x05)  // R
#define TMC2209_REG_IOIN          ((uint8_t)0x06)  // R
#define TMC2209_REG_FACTORY_CONF  ((uint8_t)0x07)  // RW

/* VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0x10…0x1F) */
#define TMC2209_REG_IHOLD_IRUN    ((uint8_t)0x10)  // W
#define TMC2209_REG_TPOWERDOWN    ((uint8_t)0x11)  // W
#define TMC2209_REG_TSTEP         ((uint8_t)0x12)  // R
#define TMC2209_REG_TPWMTHRS      ((uint8_t)0x13)  // W
#define TMC2209_REG_VACTUAL       ((uint8_t)0x22)  // W

/* COOLSTEP AND STALLGUARD CONTROL REGISTER SET (0x14, 0x40…0x42) */
#define TMC2209_REG_TCOOLTHRS     ((uint8_t)0x14)  // W
#define TMC2209_REG_SGTHRS        ((uint8_t)0x40)  // W
#define TMC2209_REG_SG_RESULT     ((uint8_t)0x41)  // R
#define TMC2209_REG_COOLCONF      ((uint8_t)0x42)  // W

/* MICROSTEPPING CONTROL REGISTER SET (0x60…0x6B) */
#define TMC2209_REG_MSCNT         ((uint8_t)0x6A)  // R
#define TMC2209_REG_MSCURACT      ((uint8_t)0x6B)  // R

/* DRIVER REGISTER SET (0x6C…0x7F) */
#define TMC2209_REG_CHOPCONF      ((uint8_t)0x6C)  // RW
#define TMC2209_REG_DRV_STATUS    ((uint8_t)0x6F)  // R
#define TMC2209_REG_PWMCONF       ((uint8_t)0x70)  // RW
#define TMC2209_REG_PWM_SCALE     ((uint8_t)0x71)  // R
#define TMC2209_REG_PWM_AUTO      ((uint8_t)0x72)  // R

/* GENERAL CONFIGURATION REGISTERS (0x00…0x0F) */

typedef union {
    uint32_t raw;
    struct {
        uint32_t I_scale_analog   : 1;  // Bit 0
        uint32_t internal_Rsense  : 1;  // Bit 1
        uint32_t en_SpreadCycle   : 1;  // Bit 2
        uint32_t shaft            : 1;  // Bit 3
        uint32_t index_otpw       : 1;  // Bit 4
        uint32_t index_step       : 1;  // Bit 5
        uint32_t pdn_disable      : 1;  // Bit 6
        uint32_t mstep_reg_select : 1;  // Bit 7
        uint32_t multistep_filt   : 1;  // Bit 8
        uint32_t test_mode        : 1;  // Bit 9
        uint32_t reserved         : 22; // Bit 10-31
    } bits;
} GCONF_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t reset      : 1;  // Bit 0: Reset occurred (write 1 to clear)
        uint32_t drv_err    : 1;  // Bit 1: Driver error (write 1 to clear)
        uint32_t uv_cp      : 1;  // Bit 2: Undervoltage on charge pump (not latched)
        uint32_t reserved   : 29; // Bit 3-31
    } bits;
} GSTAT_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t ifcnt     : 8;   // Bit 0-7: Interface transmission counter
        uint32_t reserved  : 24;  // Bit 8-31
    } bits;
} IFCNT_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t senddelay : 4;   // Bit 8-11: UART send delay (0=8, 1=3 * 8, 2=5 * 8... bits times)
        uint32_t reserved  : 28;  // Bit 0-7, 12-31
    } bits;
} SLAVECONF_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t otpbit   : 3;   // Bit 0-2: OTP bit selection
        uint32_t reserved1: 1;   // Bit 3
        uint32_t otpbyte  : 2;   // Bit 4-5: OTP byte selection
        uint32_t reserved2: 2;   // Bit 6-7
        uint32_t otpmagic : 8;   // Bit 8-15: Magic value 0xBD for programming
        uint32_t reserved3: 16;  // Bit 16-31
    } bits;
} OTP_PROG_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t otp0     : 8;   // Bit 0-7: OTP byte 0
        uint32_t otp1     : 8;   // Bit 8-15: OTP byte 1
        uint32_t otp2     : 8;   // Bit 16-23: OTP byte 2
        uint32_t reserved : 8;   // Bit 24-31
    } bits;
} OTP_READ_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t enn       : 1;  // Bit 0
        uint32_t reserved1 : 1;  // Bit 1
        uint32_t ms1       : 1;  // Bit 2
        uint32_t ms2       : 1;  // Bit 3
        uint32_t diag      : 1;  // Bit 4
        uint32_t reserved2 : 1;  // Bit 5
        uint32_t pdn_uart  : 1;  // Bit 6
        uint32_t step      : 1;  // Bit 7
        uint32_t spread_en : 1;  // Bit 8
        uint32_t dir       : 1;  // Bit 9
        uint32_t reserved3 : 22; // Bit 10-31 (includes VERSION in upper bits)
    } bits;
} IOIN_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t fclktrim : 5;   // Bit 0-4: Clock trim
        uint32_t reserved1: 3;   // Bit 5-7
        uint32_t ottrim   : 2;   // Bit 8-9: Overtemperature trim
        uint32_t reserved2: 22;  // Bit 10-31
    } bits;
} FACTORY_CONF_reg_t;

/* VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0x10…0x1F) */

typedef union {
    uint32_t raw;
    struct {
        uint32_t ihold      : 5;  // Bit 0-4: Hold current (0-31)
        uint32_t reserved1  : 3;  // Bit 5-7
        uint32_t irun       : 5;  // Bit 8-12: Run current (0-31)
        uint32_t reserved2  : 3;  // Bit 13-15
        uint32_t iholddelay : 4;  // Bit 16-19: Delay before power down
        uint32_t reserved3  : 12; // Bit 20-31
    } bits;
} IHOLD_IRUN_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t tpowerdown : 8;  // Bit 0-7: Delay after standstill for current reduction
        uint32_t reserved   : 24; // Bit 8-31
    } bits;
} TPOWERDOWN_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t tstep    : 20; // Bit 0-19: Actual measured time between microsteps
        uint32_t reserved : 12; // Bit 20-31
    } bits;
} TSTEP_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t tpwmthrs : 20; // Bit 0-19: Upper velocity for StealthChop
        uint32_t reserved : 12; // Bit 20-31
    } bits;
} TPWMTHRS_reg_t;

typedef union {
    uint32_t raw;
    struct {
        int32_t vactual   : 24; // Bit 0-23: Signed motor velocity
        int32_t reserved  : 8;  // Bit 24-31
    } bits;
} VACTUAL_reg_t;

/* COOLSTEP AND STALLGUARD CONTROL REGISTER SET (0x14, 0x40…0x42) */

typedef union {
    uint32_t raw;
    struct {
        uint32_t tcoolthrs : 20; // Bit 0-19: Lower threshold for CoolStep/StallGuard
        uint32_t reserved  : 12; // Bit 20-31
    } bits;
} TCOOLTHRS_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t sgthrs   : 8;  // Bit 0-7: StallGuard threshold
        uint32_t reserved : 24; // Bit 8-31
    } bits;
} SGTHRS_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t sg_result : 10; // Bit 0-9: StallGuard result
        uint32_t reserved  : 22; // Bit 10-31
    } bits;
} SG_RESULT_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t semin    : 4;  // Bit 0-3: Minimum StallGuard value for current scale
        uint32_t reserved1: 1;  // Bit 4
        uint32_t seup     : 2;  // Bit 5-6: Current increment step width
        uint32_t reserved2: 1;  // Bit 7
        uint32_t semax    : 4;  // Bit 8-11: StallGuard hysteresis for current down
        uint32_t reserved3: 1;  // Bit 12
        uint32_t sedn     : 2;  // Bit 13-14: Current decrement step speed
        uint32_t seimin   : 1;  // Bit 15: Minimum current setting
        uint32_t reserved4: 16; // Bit 16-31
    } bits;
} COOLCONF_reg_t;

/* MICROSTEPPING CONTROL REGISTER SET (0x60…0x6B) */

typedef union {
    uint32_t raw;
    struct {
        uint32_t mscnt    : 10; // Bit 0-9: Microstep counter position
        uint32_t reserved : 22; // Bit 10-31
    } bits;
} MSCNT_reg_t;

typedef union {
    uint32_t raw;
    struct {
        int32_t cur_a    : 9;  // Bit 0-8: Phase A current (signed)
        int32_t reserved1: 7;  // Bit 9-15
        int32_t cur_b    : 9;  // Bit 16-24: Phase B current (signed)
        int32_t reserved2: 7;  // Bit 25-31
    } bits;
} MSCURACT_reg_t;

/* DRIVER REGISTER SET (0x6C…0x7F) */

typedef union {
    uint32_t raw;
    struct {
        uint32_t toff      : 4;  // Bit 0-3: Off time (0=disable driver)
        uint32_t hstrt     : 3;  // Bit 4-6: Hysteresis start
        uint32_t hend      : 4;  // Bit 7-10: Hysteresis end
        uint32_t reserved1 : 4;  // Bit 11-14
        uint32_t tbl       : 2;  // Bit 15-16: Blank time
        uint32_t vsense    : 1;  // Bit 17: Sense resistor voltage based current scaling
        uint32_t reserved2 : 6;  // Bit 18-23
        uint32_t mres      : 4;  // Bit 24-27: Microstep resolution
        uint32_t intpol    : 1;  // Bit 28: Interpolation to 256 microsteps
        uint32_t dedge     : 1;  // Bit 29: Double edge step pulses
        uint32_t diss2g    : 1;  // Bit 30: Short to GND protection disable
        uint32_t diss2vs   : 1;  // Bit 31: Low side short protection disable
    } bits;
} CHOPCONF_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t ot       : 1;  // Bit 0: Overtemperature flag
        uint32_t otpw     : 1;  // Bit 1: Overtemperature pre-warning
        uint32_t s2ga     : 1;  // Bit 2: Short to GND A
        uint32_t s2gb     : 1;  // Bit 3: Short to GND B
        uint32_t s2vsa    : 1;  // Bit 4: Low side short A
        uint32_t s2vsb    : 1;  // Bit 5: Low side short B
        uint32_t ola      : 1;  // Bit 6: Open load A
        uint32_t olb      : 1;  // Bit 7: Open load B
        uint32_t t120     : 1;  // Bit 8: 120°C flag
        uint32_t t143     : 1;  // Bit 9: 143°C flag
        uint32_t t150     : 1;  // Bit 10: 150°C flag
        uint32_t t157     : 1;  // Bit 11: 157°C flag
        uint32_t reserved1: 4;  // Bit 12-15
        uint32_t cs_actual: 5;  // Bit 16-20: Actual current scale
        uint32_t reserved2: 3;  // Bit 21-23
        uint32_t stealth  : 1;  // Bit 24: StealthChop mode active
        uint32_t stst     : 1;  // Bit 25: Standstill detected
        uint32_t reserved3: 6;  // Bit 26-31
    } bits;
} DRV_STATUS_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t pwm_ofs   : 8;  // Bit 0-7: PWM amplitude offset
        uint32_t pwm_grad  : 8;  // Bit 8-15: PWM gradient
        uint32_t pwm_freq  : 2;  // Bit 16-17: PWM frequency
        uint32_t pwm_autoscale: 1; // Bit 18: PWM automatic scaling
        uint32_t pwm_autograd  : 1; // Bit 19: PWM automatic gradient
        uint32_t freewheel  : 2; // Bit 20-21: Freewheel mode
        uint32_t reserved1  : 2; // Bit 22-23
        uint32_t pwm_reg    : 4; // Bit 24-27: Regulation loop gradient
        uint32_t pwm_lim    : 4; // Bit 28-31: PWM scale limit
    } bits;
} PWMCONF_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t pwm_scale_sum : 8;  // Bit 0-7: Actual PWM duty cycle
        uint32_t reserved1    : 8;  // Bit 8-15
        int32_t pwm_scale_auto: 9;  // Bit 16-24: Auto-scaled PWM amplitude (signed)
        uint32_t reserved2    : 7;  // Bit 25-31
    } bits;
} PWM_SCALE_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t pwm_ofs_auto : 8;  // Bit 0-7: Auto-tuned PWM offset
        uint32_t reserved1    : 8;  // Bit 8-15
        uint32_t pwm_grad_auto: 8;  // Bit 16-23: Auto-tuned PWM gradient
        uint32_t reserved2    : 8;  // Bit 24-31
    } bits;
} PWM_AUTO_reg_t;

typedef struct {
    bool motor_l_connect;
    bool motor_s_connect;
    bool motor_b_connect;
} m_tmc2209_conf_t;

/* 初始化、配置 TMC2209 */
void m_tmc2209_init(void);

#endif
