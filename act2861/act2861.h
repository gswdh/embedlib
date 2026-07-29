#ifndef ACT2861_H
#define ACT2861_H

#include <stdint.h>

/* -------------------------------------------------------------------------
 * ACT2861 — 30 V buck-boost battery charger with integrated MOSFETs and OTG
 * (Qorvo, data sheet Rev. F).
 *
 * Portable driver: all register logic lives here.  The application supplies
 * the platform layer by implementing the three weakly defined hooks declared
 * below (two I2C transfers plus a millisecond delay used for the documented
 * ADC and reset timings).  The driver performs no printing, logging or
 * allocation; every failure is reported through a distinct act2861_error_t.
 *
 * No GPIO is required.  Charging, OTG and ship mode are all reachable over
 * I2C through the OVERRIDE_EN_CHG, OTG_EN_OVERRIDE and SHIPM_ENTER bits, so
 * a board that leaves EN_CHG, nOTG and SHIPM strapped can be driven entirely
 * from this API.
 *
 * Current readings depend on four external resistors supplied in
 * act2861_config_t: the input and output current-sense shunts and the ILIM
 * and OLIM programming resistors.
 * ------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
 * I2C addressing (Table 1).  The part is factory configured to one of two
 * 7-bit addresses; 0x24 is the standard option.
 * ------------------------------------------------------------------------- */
#define ACT2861_I2C_ADDR_DEFAULT (0x24U)

/* -------------------------------------------------------------------------
 * Register addresses
 * ------------------------------------------------------------------------- */
#define ACT2861_REG_MAIN_CONTROL_1   (0x00U)
#define ACT2861_REG_MAIN_CONTROL_2   (0x01U)
#define ACT2861_REG_GENERAL_STATUS   (0x02U)
#define ACT2861_REG_CHARGER_STATUS   (0x03U)
#define ACT2861_REG_TEMP_STATUS      (0x04U)
#define ACT2861_REG_FAULTS_1         (0x05U)
#define ACT2861_REG_FAULTS_2         (0x06U)
#define ACT2861_REG_ADC_OUT_1        (0x07U)
#define ACT2861_REG_ADC_OUT_2        (0x08U)
#define ACT2861_REG_ADC_CONFIG_1     (0x09U)
#define ACT2861_REG_ADC_CONFIG_2     (0x0AU)
#define ACT2861_REG_CHARGE_CONTROL_1 (0x0BU)
#define ACT2861_REG_CHARGE_CONTROL_2 (0x0CU)
#define ACT2861_REG_CHARGE_CONTROL_3 (0x0DU)
#define ACT2861_REG_OTG_CONTROL_1    (0x0EU)
#define ACT2861_REG_OTG_CONTROL_2    (0x0FU)
#define ACT2861_REG_OTG_CONTROL_3    (0x10U)
#define ACT2861_REG_VBAT_REG_1       (0x11U)
#define ACT2861_REG_VBAT_REG_2       (0x12U)
#define ACT2861_REG_OTG_VOLTAGE_1    (0x13U)
#define ACT2861_REG_OTG_VOLTAGE_2    (0x14U)
#define ACT2861_REG_INPUT_CURR_LIMIT (0x15U)
#define ACT2861_REG_INPUT_VOLT_LIMIT (0x16U)
#define ACT2861_REG_OTG_CURR_LIMIT   (0x17U)
#define ACT2861_REG_FAST_CHG_CURRENT (0x18U)
#define ACT2861_REG_PRE_TERM_CURRENT (0x19U)
#define ACT2861_REG_VBAT_LOW         (0x1AU)
#define ACT2861_REG_SAFETY_TIMER     (0x1BU)
#define ACT2861_REG_JEITA            (0x1CU)
#define ACT2861_REG_TEMP_SETTING     (0x1DU)
#define ACT2861_REG_IRQ_CONTROL_1    (0x1EU)
#define ACT2861_REG_IRQ_CONTROL_2    (0x1FU)
#define ACT2861_REG_OTG_STATUS       (0x20U)

#define ACT2861_REG_MAX              (0x20U)

/* -------------------------------------------------------------------------
 * Main Control 1 (0x00)
 * ------------------------------------------------------------------------- */
#define ACT2861_MC1_HIZ              (0x80U)
#define ACT2861_MC1_OVERRIDE_EN_CHG  (0x40U)
#define ACT2861_MC1_SHIPM_ENTER      (0x20U)
#define ACT2861_MC1_GPIO_OUT         (0x10U)
#define ACT2861_MC1_DIS_NCHG_CHG     (0x08U)
#define ACT2861_MC1_WATCHDOG_RESET   (0x04U)
#define ACT2861_MC1_AUDIO_FREQ_LIMIT (0x02U)
#define ACT2861_MC1_REGISTER_RESET   (0x01U)

/* -------------------------------------------------------------------------
 * Main Control 2 (0x01)
 * ------------------------------------------------------------------------- */
#define ACT2861_MC2_DIS_TH             (0x80U)
#define ACT2861_MC2_DIS_OCP_SHUTDOWN   (0x40U)
#define ACT2861_MC2_DIS_VBAT_OVP       (0x20U)
#define ACT2861_MC2_FET_ILIMIT         (0x10U)
#define ACT2861_MC2_VIN_OV_RESTART_DLY (0x08U)
#define ACT2861_MC2_VREG_EN            (0x04U)
#define ACT2861_MC2_WATCHDOG_MASK      (0x03U)

/* -------------------------------------------------------------------------
 * General Status (0x02, read only)
 * ------------------------------------------------------------------------- */
#define ACT2861_GS_NVBAT_GOOD      (0x80U)
#define ACT2861_GS_NIRQ_PIN_STATUS (0x40U)
#define ACT2861_GS_NOTG_PIN_STATUS (0x20U)
#define ACT2861_GS_INPUT_UVLO_CHG  (0x10U)
#define ACT2861_GS_INPUT_OV_CHG    (0x08U)
#define ACT2861_GS_GPIO_IN         (0x04U)
#define ACT2861_GS_OPERATION_MASK  (0x03U)

/* -------------------------------------------------------------------------
 * Charger Status (0x03, read only)
 * ------------------------------------------------------------------------- */
#define ACT2861_CS_EN_CHG_PIN_STATUS (0x80U)
#define ACT2861_CS_THERMAL_ACTIVE    (0x40U)
#define ACT2861_CS_INPUT_IINLIM      (0x20U)
#define ACT2861_CS_INPUT_VINLIM      (0x10U)
#define ACT2861_CS_CHG_STATUS_MASK   (0x0FU)

/* -------------------------------------------------------------------------
 * Temperature Status (0x04, read only)
 * ------------------------------------------------------------------------- */
#define ACT2861_TS_POK_VOUT      (0x80U)
#define ACT2861_TS_TH_BAT_DETECT (0x40U)
#define ACT2861_TS_OTG_COLD_DIS  (0x20U)
#define ACT2861_TS_OTG_HOT_DIS   (0x10U)
#define ACT2861_TS_CHRG_COLD     (0x08U)
#define ACT2861_TS_CHRG_COOL     (0x04U)
#define ACT2861_TS_CHRG_WARM     (0x02U)
#define ACT2861_TS_CHRG_HOT      (0x01U)

/* -------------------------------------------------------------------------
 * Faults 1 (0x05).  Bits 6-0 are latching and clear when read.
 * ------------------------------------------------------------------------- */
#define ACT2861_F1_NIRQ_CLEAR        (0x80U) /* write 1 to release nIRQ */
#define ACT2861_F1_CHG_TIMER_EXPIRED (0x40U)
#define ACT2861_F1_CHG_VBAT_OV       (0x20U)
#define ACT2861_F1_VREG_OC_UVLO      (0x10U)
#define ACT2861_F1_TSD               (0x08U)
#define ACT2861_F1_FET_OC            (0x04U)
#define ACT2861_F1_CHG_INPUT_OV      (0x02U)
#define ACT2861_F1_CHG_INPUT_UV      (0x01U)

/* -------------------------------------------------------------------------
 * Faults 2 (0x06).  Latching except DEADBATTERY, which is real time.
 * ------------------------------------------------------------------------- */
#define ACT2861_F2_WATCHDOG_FAULT  (0x80U)
#define ACT2861_F2_OTG_VOUT_FAULT  (0x40U)
#define ACT2861_F2_OTG_VBAT_CUTOFF (0x20U)
#define ACT2861_F2_OTG_VOUT_OV     (0x10U)
#define ACT2861_F2_OTG_LIGHT_LOAD  (0x08U)
#define ACT2861_F2_OTG_VBAT_OV     (0x04U)
#define ACT2861_F2_I2C_FAULT       (0x02U)
#define ACT2861_F2_DEADBATTERY     (0x01U)

/* -------------------------------------------------------------------------
 * ADC Configuration 1 (0x09) and 2 (0x0A)
 * ------------------------------------------------------------------------- */
#define ACT2861_ADC1_EN_ADC          (0x80U)
#define ACT2861_ADC1_ONE_SHOT        (0x40U)
#define ACT2861_ADC1_CH_SCAN         (0x20U)
#define ACT2861_ADC1_DIS_ADC_BUFFER  (0x10U)
#define ACT2861_ADC1_ADC_SWAP        (0x08U)
#define ACT2861_ADC1_HW_DIE_REV_MASK (0x07U)

#define ACT2861_ADC2_DATA_READY    (0x80U)
#define ACT2861_ADC2_CH_READ_MASK  (0x38U)
#define ACT2861_ADC2_CH_READ_SHIFT (3U)
#define ACT2861_ADC2_CH_CONV_MASK  (0x07U)

/* -------------------------------------------------------------------------
 * Charger Control 1 (0x0B)
 * ------------------------------------------------------------------------- */
#define ACT2861_CC1_VBAT_OV_DEGLITCH    (0x80U)
#define ACT2861_CC1_VBAT_SHORT_MASK     (0x70U)
#define ACT2861_CC1_VBAT_SHORT_SHIFT    (4U)
#define ACT2861_CC1_SHORT_CURRENT_MASK  (0x0CU)
#define ACT2861_CC1_SHORT_CURRENT_SHIFT (2U)
#define ACT2861_CC1_VREG_OVERRIDE       (0x02U)
#define ACT2861_CC1_VREG_SELECT         (0x01U)

/* -------------------------------------------------------------------------
 * Charger Control 2 (0x0C)
 * ------------------------------------------------------------------------- */
#define ACT2861_CC2_ILIM_LOW       (0x80U)
#define ACT2861_CC2_EN_TERM        (0x40U)
#define ACT2861_CC2_VCLAMP_MASK    (0x38U)
#define ACT2861_CC2_VCLAMP_SHIFT   (3U)
#define ACT2861_CC2_PATH_COMP_MASK (0x07U)

/* -------------------------------------------------------------------------
 * Charger Control 3 (0x0D)
 * ------------------------------------------------------------------------- */
#define ACT2861_CC3_VRECHARGE_MASK     (0xE0U)
#define ACT2861_CC3_VRECHARGE_SHIFT    (5U)
#define ACT2861_CC3_VIN_STRT_DLY_MASK  (0x18U)
#define ACT2861_CC3_VIN_STRT_DLY_SHIFT (3U)
#define ACT2861_CC3_DIS_CHG_VREG_FLT   (0x04U)
#define ACT2861_CC3_VBATGOOD_MASK      (0x03U)

/* -------------------------------------------------------------------------
 * OTG Control 1 (0x0E), 2 (0x0F) and 3 (0x10)
 * ------------------------------------------------------------------------- */
#define ACT2861_OTG1_OTG_EN        (0x80U)
#define ACT2861_OTG1_EN_OVERRIDE   (0x40U)
#define ACT2861_OTG1_SOFT_START    (0x20U)
#define ACT2861_OTG1_OFF_DLY_MASK  (0x0CU)
#define ACT2861_OTG1_OFF_DLY_SHIFT (2U)
#define ACT2861_OTG1_PIN_POLARITY  (0x02U)
#define ACT2861_OTG1_OFF_LOAD_EN   (0x01U)

#define ACT2861_OTG2_VBAT_CUTOFF_MASK  (0xE0U)
#define ACT2861_OTG2_VBAT_CUTOFF_SHIFT (5U)
#define ACT2861_OTG2_EN_OTG_NCHG       (0x10U)
#define ACT2861_OTG2_CORD_COMP_MASK    (0x0CU)
#define ACT2861_OTG2_CORD_COMP_SHIFT   (2U)
#define ACT2861_OTG2_EN_DLY_MASK       (0x03U)

#define ACT2861_OTG3_SLEW_MASK      (0xC0U)
#define ACT2861_OTG3_SLEW_SHIFT     (6U)
#define ACT2861_OTG3_PULLDOWN_RAMP  (0x20U)
#define ACT2861_OTG3_PULLDOWN_OV    (0x10U)
#define ACT2861_OTG3_BAT_ILIM_MASK  (0x0CU)
#define ACT2861_OTG3_BAT_ILIM_SHIFT (2U)
#define ACT2861_OTG3_DIS_VREG_FLT   (0x02U)
#define ACT2861_OTG3_DIS_PFM        (0x01U)

/* -------------------------------------------------------------------------
 * Battery regulation (0x11, 0x12) and OTG output voltage (0x13, 0x14)
 * ------------------------------------------------------------------------- */
#define ACT2861_VBAT1_VREG_MASK     (0xF8U)
#define ACT2861_VBAT1_VREG_SHIFT    (3U)
#define ACT2861_VBAT1_VTERM_HI_MASK (0x07U)

#define ACT2861_OTGV1_RFU_MASK      (0xF0U)
#define ACT2861_OTGV1_VOUT_I2C      (0x08U)
#define ACT2861_OTGV1_VOUT_HI_MASK  (0x07U)
#define ACT2861_OTGV2_VOUT_LO_MASK  (0xFEU)
#define ACT2861_OTGV2_VOUT_LO_SHIFT (1U)

/* -------------------------------------------------------------------------
 * Limit registers (0x15 - 0x1A)
 * ------------------------------------------------------------------------- */
#define ACT2861_IIN_DIS_LIMIT  (0x80U)
#define ACT2861_IIN_LIMIT_MASK (0x7FU)
#define ACT2861_VIN_DIS_LIMIT  (0x80U)
#define ACT2861_VIN_LIMIT_MASK (0x7FU)
#define ACT2861_OTG_DIS_CC     (0x80U)
#define ACT2861_OTG_CC_MASK    (0x7FU)
#define ACT2861_IFCHG_MASK     (0x7FU)
#define ACT2861_IPRECHG_MASK   (0xF0U)
#define ACT2861_IPRECHG_SHIFT  (4U)
#define ACT2861_ITERM_MASK     (0x0FU)
#define ACT2861_VBAT_LOW_MASK  (0x7FU)

/* -------------------------------------------------------------------------
 * Safety timer (0x1B), JEITA (0x1C), temperature setting (0x1D)
 * ------------------------------------------------------------------------- */
#define ACT2861_ST_DIS_SAFETY_TIMER     (0x40U)
#define ACT2861_ST_SUSPEND_SAFETY_TIMER (0x20U)
#define ACT2861_ST_FC_TIMER_MASK        (0x1FU)

#define ACT2861_JEITA_DIS_LDO_TSHUT (0x80U)
#define ACT2861_JEITA_DIS_JEITA     (0x40U)
#define ACT2861_JEITA_VSETH_MASK    (0x38U)
#define ACT2861_JEITA_VSETH_SHIFT   (3U)
#define ACT2861_JEITA_ISETH         (0x04U)
#define ACT2861_JEITA_ISETC_MASK    (0x03U)

#define ACT2861_TEMP_FREQ_MASK       (0xC0U)
#define ACT2861_TEMP_FREQ_SHIFT      (6U)
#define ACT2861_TEMP_DIS_SHIP_RENTER (0x20U)
#define ACT2861_TEMP_OTG_HOT_MASK    (0x18U)
#define ACT2861_TEMP_OTG_HOT_SHIFT   (3U)
#define ACT2861_TEMP_OTG_COLD        (0x04U)
#define ACT2861_TEMP_TREG_MASK       (0x03U)

/* -------------------------------------------------------------------------
 * IRQ masks.  In every case a 1 masks the source off the nIRQ pin.
 * ------------------------------------------------------------------------- */
#define ACT2861_IRQ1_CHGDONE      (0x80U)
#define ACT2861_IRQ1_VBAT_GOOD    (0x40U)
#define ACT2861_IRQ1_CHG_OVUV     (0x20U)
#define ACT2861_IRQ1_OTG_VBAT     (0x10U)
#define ACT2861_IRQ1_SAFETY_TIMER (0x08U)
#define ACT2861_IRQ1_CHG_VBAT_OV  (0x04U)
#define ACT2861_IRQ1_VREG_FLT     (0x02U)
#define ACT2861_IRQ1_TSD          (0x01U)

#define ACT2861_IRQ2_FET_OC      (0x80U)
#define ACT2861_IRQ2_WATCHDOG    (0x40U)
#define ACT2861_IRQ2_OTG_HICCUP  (0x20U)
#define ACT2861_IRQ2_OTG_LL      (0x10U)
#define ACT2861_IRQ2_A2D_DATA    (0x08U)
#define ACT2861_IRQ2_HIZ         (0x04U)
#define ACT2861_IRQ2_CHG_SUSPEND (0x02U)
#define ACT2861_IRQ2_OTG_BATTEMP (0x01U)

/* -------------------------------------------------------------------------
 * OTG / IRQ status (0x20)
 * ------------------------------------------------------------------------- */
#define ACT2861_OTGS_NIRQ_I2C_ERROR (0x80U)
#define ACT2861_OTGS_BATTERY_CC     (0x40U)
#define ACT2861_OTGS_OUTPUT_CC      (0x20U)
#define ACT2861_OTGS_VBAT_CUTOFF    (0x10U)
#define ACT2861_OTGS_VBAT_OV        (0x08U)
#define ACT2861_OTGS_STATE_MASK     (0x07U)

/* -------------------------------------------------------------------------
 * Register encodings (offsets, LSB weights and limits)
 * ------------------------------------------------------------------------- */
#define ACT2861_VTERM_OFFSET_V (5.0F)
#define ACT2861_VTERM_LSB_V    (0.01F)
#define ACT2861_VTERM_MAX_V    (22.5F)
#define ACT2861_VTERM_MAX_CODE (1750U) /* 22.5 V; higher codes clamp here */

#define ACT2861_VREG_OFFSET_V (2.0F)
#define ACT2861_VREG_LSB_V    (0.1F)
#define ACT2861_VREG_MAX_V    (5.1F)

#define ACT2861_OTG_VOUT_OFFSET_V (2.96F)
#define ACT2861_OTG_VOUT_LSB_V    (0.02F)
#define ACT2861_OTG_VOUT_MAX_V    (23.42F)

#define ACT2861_VINLIM_OFFSET_V (4.0F)
#define ACT2861_VINLIM_LSB_V    (0.1F)
#define ACT2861_VINLIM_MAX_V    (16.7F)

#define ACT2861_VBAT_LOW_OFFSET_V (2.5F)
#define ACT2861_VBAT_LOW_LSB_V    (0.1F)
#define ACT2861_VBAT_LOW_MAX_V    (15.2F)

#define ACT2861_PERCENT_MIN      (1U) /* IINLIM, OTG_CC, IFCHG */
#define ACT2861_PERCENT_MAX      (100U)
#define ACT2861_PRE_TERM_MIN_PCT (5U) /* IPRECHG, ITERM        */
#define ACT2861_PRE_TERM_MAX_PCT (20U)

#define ACT2861_SAFETY_TIMER_MIN_H (0.5F)
#define ACT2861_SAFETY_TIMER_MAX_H (16.0F)
#define ACT2861_SAFETY_TIMER_LSB_H (0.5F)

/* -------------------------------------------------------------------------
 * ADC conversion constants (Table 14).  Terms marked /R are additionally
 * divided by the relevant external resistors.
 * ------------------------------------------------------------------------- */
#define ACT2861_ADC_MIDSCALE    (2048)
#define ACT2861_ADC_K_CURRENT   (0.7633F)   /* /RCS/RLIM -> amps */
#define ACT2861_ADC_K_VIN       (0.02035F)  /* volts             */
#define ACT2861_ADC_K_VBAT      (0.01527F)  /* volts             */
#define ACT2861_ADC_K_VTH       (0.003053F) /* volts             */
#define ACT2861_ADC_K_VADC      (0.001527F) /* volts             */
#define ACT2861_ADC_K_TJ_GAIN   (0.2707F)   /* degrees Celsius   */
#define ACT2861_ADC_K_TJ_OFFSET (809.49F)

/* -------------------------------------------------------------------------
 * Enumerations
 * ------------------------------------------------------------------------- */

/** @brief Overall state machine (General Status OPERATION_MODE). */
typedef enum
{
    ACT2861_MODE_HIZ     = 0,
    ACT2861_MODE_CHARGER = 1,
    ACT2861_MODE_OTG     = 2,
    ACT2861_MODE_INVALID = 3
} act2861_mode_t;

/** @brief Charger state machine (Charger Status CHG_STATUS). */
typedef enum
{
    ACT2861_CHG_RESET     = 0x0, /* not in charge mode         */
    ACT2861_CHG_SCOND     = 0x1, /* short-circuit conditioning */
    ACT2861_CHG_SCSUSPEND = 0x2,
    ACT2861_CHG_PCOND     = 0x3, /* pre-conditioning           */
    ACT2861_CHG_PCSUSPEND = 0x4,
    ACT2861_CHG_FASTCHG   = 0x5,
    ACT2861_CHG_FCSUSPEND = 0x6,
    ACT2861_CHG_FULL      = 0x7,
    ACT2861_CHG_CFSUSPEND = 0x8,
    ACT2861_CHG_TERM      = 0x9,
    ACT2861_CHG_CTSUSPEND = 0xA,
    ACT2861_CHG_FAULT     = 0xB
} act2861_charge_state_t;

/** @brief OTG state machine (OTG Status OTG_STATUS). */
typedef enum
{
    ACT2861_OTG_RESET  = 0,
    ACT2861_OTG_SS     = 1, /* soft start */
    ACT2861_OTG_REG    = 2, /* regulating */
    ACT2861_OTG_HICCUP = 3,
    ACT2861_OTG_LL_DIS = 4 /* light-load disable */
} act2861_otg_state_t;

/** @brief ADC input channels (Table 14). */
typedef enum
{
    ACT2861_ADC_CH_INPUT_CURRENT   = 0, /* IIN  */
    ACT2861_ADC_CH_INPUT_VOLTAGE   = 1, /* VIN  */
    ACT2861_ADC_CH_BATTERY_VOLTAGE = 2, /* VBAT */
    ACT2861_ADC_CH_BATTERY_CURRENT = 3, /* IBAT */
    ACT2861_ADC_CH_THERMISTOR      = 4, /* TH   */
    ACT2861_ADC_CH_DIE_TEMPERATURE = 5, /* TJ   */
    ACT2861_ADC_CH_EXTERNAL_INPUT  = 6, /* ADC input pin */
    ACT2861_ADC_CH_AGND            = 7
} act2861_adc_channel_t;

/** @brief I2C watchdog timeout (Main Control 2 WATCHDOG). */
typedef enum
{
    ACT2861_WATCHDOG_DISABLED = 0,
    ACT2861_WATCHDOG_80S      = 1,
    ACT2861_WATCHDOG_160S     = 2,
    ACT2861_WATCHDOG_320S     = 3
} act2861_watchdog_t;

/** @brief Cycle-by-cycle FET current limit (Main Control 2 FET_ILIMIT). */
typedef enum
{
    ACT2861_FET_LIMIT_8A5 = 0,
    ACT2861_FET_LIMIT_10A = 1
} act2861_fet_limit_t;

/** @brief Battery short threshold (Charger Control 1 VBAT_SHORT). */
typedef enum
{
    ACT2861_VBAT_SHORT_4V0   = 0,
    ACT2861_VBAT_SHORT_5V0   = 1,
    ACT2861_VBAT_SHORT_6V0   = 2,
    ACT2861_VBAT_SHORT_7V5   = 3,
    ACT2861_VBAT_SHORT_8V0   = 4,
    ACT2861_VBAT_SHORT_10V0  = 5,
    ACT2861_VBAT_SHORT_10V0B = 6,
    ACT2861_VBAT_SHORT_12V5  = 7
} act2861_vbat_short_t;

/** @brief Charge current below the short threshold (VBAT_SHORT_CURRENT). */
typedef enum
{
    ACT2861_SHORT_CURRENT_1PCT = 0,
    ACT2861_SHORT_CURRENT_2PCT = 1,
    ACT2861_SHORT_CURRENT_4PCT = 2,
    ACT2861_SHORT_CURRENT_8PCT = 3
} act2861_short_current_t;

/** @brief Recharge threshold below VTERM (Charger Control 3 VRECHARGE). */
typedef enum
{
    ACT2861_VRECHARGE_200MV = 0,
    ACT2861_VRECHARGE_300MV = 1,
    ACT2861_VRECHARGE_400MV = 2,
    ACT2861_VRECHARGE_450MV = 3,
    ACT2861_VRECHARGE_500MV = 4,
    ACT2861_VRECHARGE_600MV = 5,
    ACT2861_VRECHARGE_750MV = 6,
    ACT2861_VRECHARGE_800MV = 7
} act2861_vrecharge_t;

/** @brief Charger start delay (Charger Control 3 VIN_STRT_DLY). */
typedef enum
{
    ACT2861_START_DELAY_NONE  = 0,
    ACT2861_START_DELAY_220MS = 1,
    ACT2861_START_DELAY_500MS = 2,
    ACT2861_START_DELAY_1S3   = 3
} act2861_start_delay_t;

/** @brief Battery-good threshold above VBAT_LOW (VBATGOOD). */
typedef enum
{
    ACT2861_VBATGOOD_PLUS_0V4 = 0,
    ACT2861_VBATGOOD_PLUS_0V6 = 1,
    ACT2861_VBATGOOD_PLUS_0V8 = 2,
    ACT2861_VBATGOOD_PLUS_1V0 = 3
} act2861_vbatgood_t;

/** @brief Battery path impedance compensation (BAT_PATH_COMP). */
typedef enum
{
    ACT2861_PATH_COMP_DISABLED = 0,
    ACT2861_PATH_COMP_20MOHM   = 1,
    ACT2861_PATH_COMP_40MOHM   = 2,
    ACT2861_PATH_COMP_60MOHM   = 3,
    ACT2861_PATH_COMP_80MOHM   = 4,
    ACT2861_PATH_COMP_100MOHM  = 5,
    ACT2861_PATH_COMP_120MOHM  = 6,
    ACT2861_PATH_COMP_140MOHM  = 7
} act2861_path_comp_t;

/** @brief Path compensation voltage clamp (BAT_PATH_COMP_VCLAMP). */
typedef enum
{
    ACT2861_VCLAMP_DISABLED = 0,
    ACT2861_VCLAMP_60MV     = 1,
    ACT2861_VCLAMP_120MV    = 2,
    ACT2861_VCLAMP_180MV    = 3,
    ACT2861_VCLAMP_240MV    = 4,
    ACT2861_VCLAMP_300MV    = 5,
    ACT2861_VCLAMP_360MV    = 6,
    ACT2861_VCLAMP_420MV    = 7
} act2861_vclamp_t;

/** @brief OTG battery cutoff, referenced to VBAT_LOW (OTG_VBAT_CUTOFF). */
typedef enum
{
    ACT2861_OTG_CUTOFF_VBAT_LOW  = 0,
    ACT2861_OTG_CUTOFF_MINUS_0V2 = 1,
    ACT2861_OTG_CUTOFF_MINUS_0V4 = 2,
    ACT2861_OTG_CUTOFF_MINUS_0V6 = 3,
    ACT2861_OTG_CUTOFF_MINUS_0V8 = 4,
    ACT2861_OTG_CUTOFF_MINUS_1V0 = 5,
    ACT2861_OTG_CUTOFF_MINUS_1V2 = 6,
    ACT2861_OTG_CUTOFF_MINUS_1V4 = 7
} act2861_otg_cutoff_t;

/** @brief OTG cord compensation at 2.4 A (OTG_CORD_COMP). */
typedef enum
{
    ACT2861_CORD_COMP_DISABLED = 0,
    ACT2861_CORD_COMP_100MV    = 1,
    ACT2861_CORD_COMP_200MV    = 2,
    ACT2861_CORD_COMP_300MV    = 3
} act2861_cord_comp_t;

/** @brief OTG enable delay (OTG_EN_DLY). */
typedef enum
{
    ACT2861_OTG_EN_DELAY_NONE  = 0,
    ACT2861_OTG_EN_DELAY_200MS = 1,
    ACT2861_OTG_EN_DELAY_500MS = 2,
    ACT2861_OTG_EN_DELAY_1S    = 3
} act2861_otg_en_delay_t;

/** @brief OTG light-load off delay (OTG_OFF_DLY). */
typedef enum
{
    ACT2861_OTG_OFF_DELAY_DISABLED = 0,
    ACT2861_OTG_OFF_DELAY_10S      = 1,
    ACT2861_OTG_OFF_DELAY_20S      = 2,
    ACT2861_OTG_OFF_DELAY_30S      = 3
} act2861_otg_off_delay_t;

/** @brief OTG output slew rate (OTG_OUTPUT_SLEW). */
typedef enum
{
    ACT2861_OTG_SLEW_1V_MS   = 0,
    ACT2861_OTG_SLEW_0V5_MS  = 1,
    ACT2861_OTG_SLEW_0V33_MS = 2,
    ACT2861_OTG_SLEW_0V1_MS  = 3
} act2861_otg_slew_t;

/**
 * @brief OTG battery-side current limit scaling (OTG_BAT_ILIM).
 *
 * Also selects the klim term used to convert the battery-current ADC
 * reading while in OTG mode (Table 15).
 */
typedef enum
{
    ACT2861_OTG_BAT_ILIM_DISABLED = 0,
    ACT2861_OTG_BAT_ILIM_150PCT   = 1,
    ACT2861_OTG_BAT_ILIM_200PCT   = 2,
    ACT2861_OTG_BAT_ILIM_150PCT_B = 3
} act2861_otg_bat_ilim_t;

/** @brief Converter switching frequency (FREQ_SEL). */
typedef enum
{
    ACT2861_FREQ_125KHZ = 0,
    ACT2861_FREQ_250KHZ = 1,
    ACT2861_FREQ_500KHZ = 2,
    ACT2861_FREQ_1MHZ   = 3
} act2861_frequency_t;

/** @brief OTG hot-temperature threshold (OTG_HOT). */
typedef enum
{
    ACT2861_OTG_HOT_55C      = 0,
    ACT2861_OTG_HOT_60C      = 1,
    ACT2861_OTG_HOT_65C      = 2,
    ACT2861_OTG_HOT_DISABLED = 3
} act2861_otg_hot_t;

/** @brief Die temperature regulation threshold (TREG). */
typedef enum
{
    ACT2861_TREG_DISABLED = 0,
    ACT2861_TREG_80C      = 1,
    ACT2861_TREG_100C     = 2,
    ACT2861_TREG_120C     = 3
} act2861_treg_t;

/** @brief JEITA warm-zone charge voltage reduction (JEITA_VSETH). */
typedef enum
{
    ACT2861_JEITA_VSETH_VTERM       = 0,
    ACT2861_JEITA_VSETH_MINUS_200MV = 1,
    ACT2861_JEITA_VSETH_MINUS_300MV = 2,
    ACT2861_JEITA_VSETH_MINUS_400MV = 3,
    ACT2861_JEITA_VSETH_MINUS_450MV = 4,
    ACT2861_JEITA_VSETH_MINUS_500MV = 5,
    ACT2861_JEITA_VSETH_MINUS_600MV = 6,
    ACT2861_JEITA_VSETH_MINUS_750MV = 7
} act2861_jeita_vseth_t;

/** @brief JEITA cool-zone charge current (JEITA_ISETC). */
typedef enum
{
    ACT2861_JEITA_ISETC_SUSPEND = 0,
    ACT2861_JEITA_ISETC_25PCT   = 1,
    ACT2861_JEITA_ISETC_50PCT   = 2,
    ACT2861_JEITA_ISETC_100PCT  = 3
} act2861_jeita_isetc_t;

/* -------------------------------------------------------------------------
 * Error codes.  Every distinct failure condition has its own value so the
 * caller can act on it without the driver emitting any diagnostic output.
 * ------------------------------------------------------------------------- */
typedef enum
{
    ACT2861_OK = 0,

    /* Platform / transport */
    ACT2861_ERR_I2C_READ,
    ACT2861_ERR_I2C_WRITE,
    ACT2861_ERR_NO_PLATFORM_READ,
    ACT2861_ERR_NO_PLATFORM_WRITE,

    /* Argument validation */
    ACT2861_ERR_NULL_PARAM,
    ACT2861_ERR_BAD_REGISTER,       /* address above 0x20                   */
    ACT2861_ERR_BAD_LENGTH,         /* zero-length or wrapping burst        */
    ACT2861_ERR_NOT_INITIALISED,    /* act2861_init has not succeeded       */
    ACT2861_ERR_BAD_SENSE_RESISTOR, /* a configured resistance was not > 0  */
    ACT2861_ERR_BAD_CHANNEL,        /* not an act2861_adc_channel_t         */
    ACT2861_ERR_BAD_PERCENT,        /* outside the register's percent range */
    ACT2861_ERR_BAD_VOLTAGE,        /* outside the register's voltage range */
    ACT2861_ERR_BAD_HOURS,          /* safety timer outside 0.5 - 16 h      */

    /* Enumerated settings, one code per field so a bad cast is traceable */
    ACT2861_ERR_BAD_WATCHDOG,
    ACT2861_ERR_BAD_FET_LIMIT,
    ACT2861_ERR_BAD_VBAT_SHORT,
    ACT2861_ERR_BAD_SHORT_CURRENT,
    ACT2861_ERR_BAD_VRECHARGE,
    ACT2861_ERR_BAD_START_DELAY,
    ACT2861_ERR_BAD_VBATGOOD,
    ACT2861_ERR_BAD_PATH_COMP,
    ACT2861_ERR_BAD_VCLAMP,
    ACT2861_ERR_BAD_OTG_CUTOFF,
    ACT2861_ERR_BAD_CORD_COMP,
    ACT2861_ERR_BAD_OTG_EN_DELAY,
    ACT2861_ERR_BAD_OTG_OFF_DELAY,
    ACT2861_ERR_BAD_OTG_SLEW,
    ACT2861_ERR_BAD_OTG_BAT_ILIM,
    ACT2861_ERR_BAD_FREQUENCY,
    ACT2861_ERR_BAD_OTG_HOT,
    ACT2861_ERR_BAD_TREG,
    ACT2861_ERR_BAD_JEITA_VSETH,
    ACT2861_ERR_BAD_JEITA_ISETC,

    /* Device state */
    ACT2861_ERR_WRITE_VERIFY,  /* read-back never matched the write    */
    ACT2861_ERR_RESET_TIMEOUT, /* REGISTER_RESET never self-cleared    */
    ACT2861_ERR_ADC_TIMEOUT,   /* ADC_DATA_READY never asserted        */
    ACT2861_ERR_KLIM_DISABLED, /* OTG_BAT_ILIM = 00, IBAT undefined    */
    ACT2861_ERR_HIZ_ACTIVE     /* HIZ blocks charging and OTG          */
} act2861_error_t;

/* -------------------------------------------------------------------------
 * Decoded register views
 * ------------------------------------------------------------------------- */

/** @brief General Status register (0x02). */
typedef struct
{
    uint8_t        raw;
    uint8_t        battery_low;  /* nVBAT_GOOD: 1 = below VBAT_GOOD */
    uint8_t        irq_asserted; /* our own nIRQ drive state        */
    uint8_t        notg_pin_high;
    uint8_t        input_below_uvlo;
    uint8_t        input_above_ov;
    uint8_t        gpio_in;
    act2861_mode_t mode;
} act2861_general_status_t;

/** @brief Charger Status register (0x03). */
typedef struct
{
    uint8_t                raw;
    uint8_t                en_chg_pin_high;
    uint8_t                thermal_regulation;
    uint8_t                in_input_current_limit;
    uint8_t                in_input_voltage_limit;
    act2861_charge_state_t state;
} act2861_charger_status_t;

/** @brief Temperature Status register (0x04). */
typedef struct
{
    uint8_t raw;
    uint8_t power_ok;         /* POK_VOUT      */
    uint8_t battery_detected; /* TH_BAT_DETECT */
    uint8_t otg_cold_disabled;
    uint8_t otg_hot_disabled;
    uint8_t charge_cold;
    uint8_t charge_cool;
    uint8_t charge_warm;
    uint8_t charge_hot;
} act2861_temp_status_t;

/**
 * @brief Latched faults from registers 0x05 and 0x06.
 *
 * Reading these registers clears the latched bits, so a fault is reported
 * exactly once unless the condition is still present.
 */
typedef struct
{
    uint8_t raw1;
    uint8_t raw2;

    /* Faults 1 */
    uint8_t charge_timer_expired;
    uint8_t charge_vbat_ov;
    uint8_t vreg_oc_uvlo;
    uint8_t thermal_shutdown;
    uint8_t fet_overcurrent;
    uint8_t input_overvoltage;
    uint8_t input_undervoltage;

    /* Faults 2 */
    uint8_t watchdog_fault;
    uint8_t otg_vout_hiccup;
    uint8_t otg_vbat_cutoff;
    uint8_t otg_vout_ov;
    uint8_t otg_light_load;
    uint8_t otg_vbat_ov;
    uint8_t i2c_fault;
    uint8_t dead_battery; /* real time, not latched */
} act2861_faults_t;

/** @brief OTG / IRQ Status register (0x20). */
typedef struct
{
    uint8_t             raw;
    uint8_t             battery_constant_current;
    uint8_t             output_constant_current;
    uint8_t             vbat_below_cutoff;
    uint8_t             vbat_above_ov;
    act2861_otg_state_t state;
} act2861_otg_status_t;

/** @brief Full ADC snapshot in engineering units. */
typedef struct
{
    float input_current_a;      /* CH0, IIN  */
    float input_voltage_v;      /* CH1, VIN  */
    float battery_voltage_v;    /* CH2, VBAT */
    float battery_current_a;    /* CH3, IBAT */
    float thermistor_voltage_v; /* CH4, TH   */
    float die_temperature_c;    /* CH5, TJ   */
    float external_voltage_v;   /* CH6       */
} act2861_measurements_t;

/**
 * @brief Board constants required to scale the current measurements.
 *
 * All four resistances are in ohms.  RCS_IN and RCS_OUT are the input and
 * output current-sense shunts; RILIM and ROLIM are the resistors that
 * program the hardware input and output current limits.
 */
typedef struct
{
    float   rcs_in_ohms;  /* input sense shunt, e.g. 0.01F  */
    float   rilim_ohms;   /* ILIM programming resistor      */
    float   rcs_out_ohms; /* output sense shunt, e.g. 0.01F */
    float   rolim_ohms;   /* OLIM programming resistor      */
    uint8_t i2c_address;  /* 7-bit; 0 selects the default   */
} act2861_config_t;

/* -------------------------------------------------------------------------
 * Platform hooks — implemented by the application.
 *
 * Each has a weak default in the driver that returns
 * ACT2861_ERR_NO_PLATFORM_READ / ACT2861_ERR_NO_PLATFORM_WRITE, so a build
 * that forgets to provide them fails loudly at run time instead of silently
 * reading zeros.
 * ------------------------------------------------------------------------- */

/**
 * @brief Read @p len bytes starting at register @p reg from the device at
 *        7-bit address @p dev_addr.
 * @return ACT2861_OK on success, ACT2861_ERR_I2C_READ on any bus error.
 */
act2861_error_t
act2861_i2c_read(const uint8_t dev_addr, const uint8_t reg, uint8_t *rx, const uint16_t len);

/**
 * @brief Write @p len bytes to register @p reg of the device at 7-bit
 *        address @p dev_addr.
 * @return ACT2861_OK on success, ACT2861_ERR_I2C_WRITE on any bus error.
 */
act2861_error_t act2861_i2c_write(const uint8_t  dev_addr,
                                  const uint8_t  reg,
                                  const uint8_t *tx,
                                  const uint16_t len);

/**
 * @brief Block for at least @p ms milliseconds.
 *
 * Used for the register-reset settle and as the poll interval while waiting
 * on ADC_DATA_READY; the driver never uses it for convenience alone.
 */
void act2861_delay_ms(const uint32_t ms);

/* -------------------------------------------------------------------------
 * Initialisation and raw register access
 * ------------------------------------------------------------------------- */

/**
 * @brief Latch the board configuration and confirm the device responds.
 *
 * Reads Main Control 2 to prove the device acknowledges, then stores the
 * resistances used by every current conversion.  Does not modify any device
 * configuration.
 *
 * @param[in] cfg Board configuration; copied internally.
 * @return ACT2861_OK, ACT2861_ERR_NULL_PARAM,
 *         ACT2861_ERR_BAD_SENSE_RESISTOR or a transport error.
 */
act2861_error_t act2861_init(const act2861_config_t *cfg);

/** @brief Read one register. */
act2861_error_t act2861_read_register(const uint8_t reg, uint8_t *value);

/** @brief Write one register. */
act2861_error_t act2861_write_register(const uint8_t reg, const uint8_t value);

/**
 * @brief Write a register then read it back, retrying up to three times.
 *
 * Not usable on read-to-clear or self-clearing registers.
 */
act2861_error_t act2861_write_verify_register(const uint8_t reg, const uint8_t value);

/** @brief Read @p count consecutive registers starting at @p reg. */
act2861_error_t act2861_read_registers(const uint8_t reg, uint8_t *values, const uint8_t count);

/** @brief Read-modify-write a register, replacing only the bits in @p mask. */
act2861_error_t
act2861_update_register(const uint8_t reg, const uint8_t mask, const uint8_t value);

/**
 * @brief Restore every register to its power-on default (REGISTER_RESET).
 *
 * The bit is self-clearing; this polls until it clears or the timeout runs
 * out.
 */
act2861_error_t act2861_reset(void);

/* -------------------------------------------------------------------------
 * Status
 * ------------------------------------------------------------------------- */

act2861_error_t act2861_get_general_status(act2861_general_status_t *status);
act2861_error_t act2861_get_charger_status(act2861_charger_status_t *status);
act2861_error_t act2861_get_temperature_status(act2861_temp_status_t *status);
act2861_error_t act2861_get_otg_status(act2861_otg_status_t *status);

/**
 * @brief Read and decode both fault registers.
 *
 * This clears the latched fault bits as a side effect of reading them, so
 * call it once per service and keep the result.
 */
act2861_error_t act2861_get_faults(act2861_faults_t *faults);

/** @brief Read the operating mode without decoding the rest of the status. */
act2861_error_t act2861_get_mode(act2861_mode_t *mode);

/** @brief Assert nIRQ_Clear to release the nIRQ pin. */
act2861_error_t act2861_clear_irq(void);

/* -------------------------------------------------------------------------
 * Mode and power control
 * ------------------------------------------------------------------------- */

/**
 * @brief Enter or leave HIZ mode.
 *
 * HIZ overrides everything: neither EN_CHG nor nOTG, nor their register
 * overrides, start the converter while it is set.
 */
act2861_error_t act2861_set_hiz(const uint8_t enabled);

/**
 * @brief Start or stop charging over I2C.
 *
 * Sets OVERRIDE_EN_CHG so the EN_CHG pin is ignored.  Refuses with
 * ACT2861_ERR_HIZ_ACTIVE if HIZ is set, since the write would not take
 * effect.
 */
act2861_error_t act2861_set_charging_enabled(const uint8_t enabled);

/**
 * @brief Enable or disable the OTG output over I2C.
 *
 * Sets OTG_EN together with OTG_EN_OVERRIDE so the nOTG pin is ignored.
 * Refuses with ACT2861_ERR_HIZ_ACTIVE if HIZ is set.
 */
act2861_error_t act2861_set_otg_enabled(const uint8_t enabled);

/**
 * @brief Begin the one-second countdown into ship mode.
 *
 * Ship mode disables everything and can only be left via the SHIPM pin or by
 * reapplying VIN.  Call act2861_cancel_ship_mode() within the second to
 * abort.
 */
act2861_error_t act2861_enter_ship_mode(void);

/** @brief Abort a pending ship-mode entry during the one-second countdown. */
act2861_error_t act2861_cancel_ship_mode(void);

/** @brief Enable or disable the VREG LDO. */
act2861_error_t act2861_set_vreg_enabled(const uint8_t enabled);

/** @brief Set the VREG LDO output, 2.0 V to 5.1 V in 100 mV steps. */
act2861_error_t act2861_set_vreg_voltage(const float volts);

/** @brief Enable or disable the nCHG status output in charge mode. */
act2861_error_t act2861_set_nchg_enabled(const uint8_t enabled);

/** @brief Limit the minimum switching frequency to 31.25 kHz. */
act2861_error_t act2861_set_audio_frequency_limit(const uint8_t enabled);

/**
 * @brief Select the converter switching frequency.
 *
 * The datasheet warns that this must not be changed while the converter is
 * running, and that each setting needs different magnetics.
 */
act2861_error_t act2861_set_switching_frequency(const act2861_frequency_t frequency);

/* -------------------------------------------------------------------------
 * Watchdog
 * ------------------------------------------------------------------------- */

/** @brief Select the I2C watchdog timeout, or disable it. */
act2861_error_t act2861_set_watchdog(const act2861_watchdog_t timeout);

/** @brief Pet the I2C watchdog (WATCHDOG_RESET, self-clearing). */
act2861_error_t act2861_kick_watchdog(void);

/* -------------------------------------------------------------------------
 * Charge configuration
 * ------------------------------------------------------------------------- */

/**
 * @brief Set the battery regulation (termination) voltage, 5 V to 22.5 V.
 *
 * Spans both VBAT_REG registers and preserves the VREG LDO setting.
 */
act2861_error_t act2861_set_charge_voltage(const float volts);

/** @brief Read the programmed battery regulation voltage, volts. */
act2861_error_t act2861_get_charge_voltage(float *volts);

/**
 * @brief Set the fast-charge current as a percentage of the OLIM-programmed
 *        maximum, 1 to 100.
 */
act2861_error_t act2861_set_fast_charge_percent(const uint8_t percent);

/** @brief Set the pre-charge current, 5 to 20 percent of the OLIM maximum. */
act2861_error_t act2861_set_precharge_percent(const uint8_t percent);

/** @brief Set the termination current, 5 to 20 percent of the OLIM maximum. */
act2861_error_t act2861_set_termination_percent(const uint8_t percent);

/** @brief Enable or disable charge termination (EN_TERM). */
act2861_error_t act2861_set_termination_enabled(const uint8_t enabled);

/**
 * @brief Set the input current limit as a percentage of the ILIM-programmed
 *        maximum, 1 to 100.
 */
act2861_error_t act2861_set_input_current_percent(const uint8_t percent);

/** @brief Enable or disable the input current limit loop entirely. */
act2861_error_t act2861_set_input_current_limit_enabled(const uint8_t enabled);

/** @brief Set the input voltage limit, 4.0 V to 16.7 V in 100 mV steps. */
act2861_error_t act2861_set_input_voltage_limit(const float volts);

/** @brief Enable or disable the input voltage limit loop. */
act2861_error_t act2861_set_input_voltage_limit_enabled(const uint8_t enabled);

/** @brief Set VBAT_LOW, 2.5 V to 15.2 V in 100 mV steps. */
act2861_error_t act2861_set_battery_low_voltage(const float volts);

/** @brief Set the battery-good threshold relative to VBAT_LOW. */
act2861_error_t act2861_set_battery_good_threshold(const act2861_vbatgood_t threshold);

/** @brief Set the battery-short threshold and the current used below it. */
act2861_error_t act2861_set_battery_short(const act2861_vbat_short_t    threshold,
                                          const act2861_short_current_t current);

/** @brief Set the recharge threshold below VTERM. */
act2861_error_t act2861_set_recharge_threshold(const act2861_vrecharge_t threshold);

/** @brief Set the delay between enabling the charger and starting it. */
act2861_error_t act2861_set_start_delay(const act2861_start_delay_t delay);

/** @brief Configure battery path impedance compensation and its clamp. */
act2861_error_t act2861_set_path_compensation(const act2861_path_comp_t resistance,
                                              const act2861_vclamp_t    clamp);

/** @brief Select the cycle-by-cycle FET current limit. */
act2861_error_t act2861_set_fet_current_limit(const act2861_fet_limit_t limit);

/** @brief Select the low 5.7 A current limit range (ILIM_LOW). */
act2861_error_t act2861_set_low_current_range(const uint8_t enabled);

/* -------------------------------------------------------------------------
 * Safety timer
 * ------------------------------------------------------------------------- */

/** @brief Set the fast-charge safety timer, 0.5 to 16 hours. */
act2861_error_t act2861_set_safety_timer_hours(const float hours);

/** @brief Enable or disable (and reset) both safety timers. */
act2861_error_t act2861_set_safety_timer_enabled(const uint8_t enabled);

/** @brief Suspend or resume both safety timers without resetting them. */
act2861_error_t act2861_set_safety_timer_suspended(const uint8_t suspended);

/* -------------------------------------------------------------------------
 * Thermal and JEITA
 * ------------------------------------------------------------------------- */

/** @brief Enable or disable the TH input and JEITA control. */
act2861_error_t act2861_set_thermistor_enabled(const uint8_t enabled);

/** @brief Enable or disable the JEITA charging profile. */
act2861_error_t act2861_set_jeita_enabled(const uint8_t enabled);

/**
 * @brief Configure the JEITA warm and cool zone responses.
 *
 * @param[in] warm_voltage      Charge voltage reduction in the 45-60 C zone.
 * @param[in] warm_full_current Non-zero for 100 percent of ICHG when warm,
 *                              zero for 50 percent.
 * @param[in] cool_current      Charge current in the 0-10 C zone.
 */
act2861_error_t act2861_set_jeita_profile(const act2861_jeita_vseth_t warm_voltage,
                                          const uint8_t               warm_full_current,
                                          const act2861_jeita_isetc_t cool_current);

/** @brief Set the OTG hot and cold thermistor thresholds. */
act2861_error_t act2861_set_otg_temperature_limits(const act2861_otg_hot_t hot,
                                                   const uint8_t           cold_minus_10c);

/** @brief Set the die temperature regulation threshold. */
act2861_error_t act2861_set_thermal_regulation(const act2861_treg_t threshold);

/* -------------------------------------------------------------------------
 * OTG configuration
 * ------------------------------------------------------------------------- */

/** @brief Set the OTG output voltage, 2.96 V to 23.42 V in 20 mV steps. */
act2861_error_t act2861_set_otg_voltage(const float volts);

/** @brief Read the programmed OTG output voltage, volts. */
act2861_error_t act2861_get_otg_voltage(float *volts);

/**
 * @brief Select internal register control or external IFB divider control of
 *        the OTG output voltage.
 */
act2861_error_t act2861_set_otg_external_feedback(const uint8_t external);

/** @brief Set the OTG output current limit, 1 to 100 percent of ILIM. */
act2861_error_t act2861_set_otg_current_percent(const uint8_t percent);

/** @brief Enable or disable the OTG output constant-current loop. */
act2861_error_t act2861_set_otg_current_limit_enabled(const uint8_t enabled);

/** @brief Set the OTG battery-side current limit scaling. */
act2861_error_t act2861_set_otg_battery_current_limit(const act2861_otg_bat_ilim_t scaling);

/** @brief Set the OTG battery cutoff threshold relative to VBAT_LOW. */
act2861_error_t act2861_set_otg_battery_cutoff(const act2861_otg_cutoff_t cutoff);

/** @brief Select the 1.5 ms or 5 ms OTG soft-start time. */
act2861_error_t act2861_set_otg_soft_start_slow(const uint8_t slow);

/** @brief Set the delay between the OTG request and the output turning on. */
act2861_error_t act2861_set_otg_enable_delay(const act2861_otg_en_delay_t delay);

/** @brief Set the light-load shutdown delay, or disable the feature. */
act2861_error_t act2861_set_otg_light_load_delay(const act2861_otg_off_delay_t delay);

/** @brief Set the OTG output voltage slew rate used for QC and PD ramps. */
act2861_error_t act2861_set_otg_slew_rate(const act2861_otg_slew_t slew);

/** @brief Set the OTG cord compensation applied at 2.4 A. */
act2861_error_t act2861_set_otg_cord_compensation(const act2861_cord_comp_t compensation);

/** @brief Enable or disable the nCHG indication while in OTG mode. */
act2861_error_t act2861_set_otg_nchg_enabled(const uint8_t enabled);

/* -------------------------------------------------------------------------
 * Interrupts
 * ------------------------------------------------------------------------- */

/**
 * @brief Set the IRQ Control 1 mask.
 *
 * A set bit masks that source off the nIRQ pin.  Compose from the
 * ACT2861_IRQ1_* macros.
 */
act2861_error_t act2861_set_irq_mask_1(const uint8_t mask);

/** @brief Set the IRQ Control 2 mask, composed from ACT2861_IRQ2_* macros. */
act2861_error_t act2861_set_irq_mask_2(const uint8_t mask);

/** @brief Mask or unmask the I2C fault source (register 0x20 bit 7). */
act2861_error_t act2861_set_irq_mask_i2c(const uint8_t masked);

/* -------------------------------------------------------------------------
 * ADC
 * ------------------------------------------------------------------------- */

/**
 * @brief Perform a single-shot conversion on one channel and return the raw
 *        12-bit code.
 *
 * Follows the datasheet single-shot sequence: sets ADC_ONE_SHOT with
 * ADC_CH_SCAN clear, points both the conversion and read channel selects at
 * @p channel, starts the conversion, then polls ADC_DATA_READY.
 *
 * @param[in]  channel    Input channel to convert.
 * @param[in]  timeout_ms Upper bound on the wait for ADC_DATA_READY.
 * @param[out] code       ADC_OUT[13:2], the 12-bit conversion result.
 * @return ACT2861_OK, ACT2861_ERR_ADC_TIMEOUT, ACT2861_ERR_BAD_CHANNEL or a
 *         transport error.
 */
act2861_error_t act2861_adc_convert(const act2861_adc_channel_t channel,
                                    const uint32_t              timeout_ms,
                                    uint16_t                   *code);

/**
 * @brief Start free-running conversion of all channels in a loop.
 *
 * In this mode nIRQ is not asserted; poll act2861_adc_data_ready() before
 * reading a channel with act2861_adc_read_channel().
 */
act2861_error_t act2861_adc_start_continuous(void);

/** @brief Stop the ADC (clears EN_ADC). */
act2861_error_t act2861_adc_stop(void);

/** @brief Report whether a conversion result is waiting to be read. */
act2861_error_t act2861_adc_data_ready(uint8_t *ready);

/**
 * @brief Read the latest result for @p channel without starting a new
 *        conversion.  Intended for continuous mode.
 */
act2861_error_t act2861_adc_read_channel(const act2861_adc_channel_t channel, uint16_t *code);

/**
 * @brief Convert a raw ADC code into engineering units for @p channel.
 *
 * Current channels are scaled by the configured sense and limit resistors.
 * The battery-current channel additionally needs the klim term, which is
 * read from the device unless @p klim is positive.
 *
 * @param[in]  channel Channel the code came from.
 * @param[in]  code    ADC_OUT[13:2] value.
 * @param[in]  klim    Override for the OTG scaling term, or 0 to read it.
 * @param[out] value   Volts, amps or degrees Celsius depending on channel.
 */
act2861_error_t act2861_adc_scale(const act2861_adc_channel_t channel,
                                  const uint16_t              code,
                                  const float                 klim,
                                  float                      *value);

/* Single-shot convenience readings, each in engineering units. */
act2861_error_t act2861_read_input_current(float *amps);
act2861_error_t act2861_read_input_voltage(float *volts);
act2861_error_t act2861_read_battery_voltage(float *volts);
act2861_error_t act2861_read_battery_current(float *amps);
act2861_error_t act2861_read_thermistor_voltage(float *volts);
act2861_error_t act2861_read_die_temperature(float *celsius);
act2861_error_t act2861_read_external_voltage(float *volts);

/**
 * @brief Convert every channel once and return the full set of readings.
 *
 * Uses single-shot conversions so the result is coherent with the current
 * operating mode; takes seven conversions to complete.
 */
act2861_error_t act2861_read_measurements(act2861_measurements_t *out);

#endif /* ACT2861_H */
