#ifndef STUSB4500_H
#define STUSB4500_H

#include <stdint.h>

/* -------------------------------------------------------------------------
 * STUSB4500 — standalone USB Type-C and Power Delivery 2.0 sink controller
 * with nonvolatile configuration memory (STMicroelectronics).
 *
 * The part negotiates an explicit power contract on its own out of reset,
 * using the three sink PDOs held in its NVM, and needs no host at all.  A
 * host is useful for three things, and this driver covers all three:
 *
 *   1. observing what was negotiated (RDO, VBUS state, CC orientation),
 *   2. overriding the sink PDOs in RAM and forcing a renegotiation, which is
 *      how you pick a contract from what the attached source actually offers,
 *   3. reprogramming the NVM so the standalone behaviour changes permanently.
 *
 * Portable driver: all register and NVM logic lives here.  The application
 * supplies the platform layer by implementing the weakly defined hooks
 * declared below (two I2C transfers, a millisecond delay, and an optional
 * RESET-pin control).  The driver performs no printing, logging or
 * allocation; every failure is reported through a distinct
 * stusb4500_error_t code.
 *
 * Register space is a flat 8-bit byte space behind one 2-wire address, set by
 * the ADDR0/ADDR1 pins to 0x28..0x2B.  Multi-byte quantities — the 16-bit
 * message headers, the 32-bit PDOs and RDO, and the NVM sectors — are
 * transferred least-significant byte first.
 *
 * The NVM is 5 sectors of 8 bytes reached through the FTP controller at
 * 0x95-0x97 and the 8-byte window at 0x53.  It has a finite number of
 * program cycles, so stusb4500_nvm_write() is a deliberate, verified,
 * infrequent operation and never part of bring-up.
 *
 * Register addresses and bit positions follow ST's own definitions
 * (USB_PD_defines_STUSB-GEN1S.h in the STUSB4500 reference firmware).  The
 * NVM bank layout follows ST's NVM technical note and has been checked
 * field-by-field against the factory-default sector contents in
 * stusb4500_nvm_factory_default().
 * ------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
 * I2C addresses.  ADDR1/ADDR0 select one of four; 0x28 is both the reset
 * default and the value used when a config asks for address 0.
 * ------------------------------------------------------------------------- */
#define STUSB4500_I2C_ADDR_DEFAULT (0x28U)
#define STUSB4500_I2C_ADDR_MIN     (0x28U)
#define STUSB4500_I2C_ADDR_MAX     (0x2BU)

/* -------------------------------------------------------------------------
 * Register addresses
 * ------------------------------------------------------------------------- */

/* Identification and capability */
#define STUSB4500_REG_BCD_TYPEC_REV_LOW   (0x06U)
#define STUSB4500_REG_BCD_TYPEC_REV_HIGH  (0x07U)
#define STUSB4500_REG_BCD_USBPD_REV_LOW   (0x08U)
#define STUSB4500_REG_BCD_USBPD_REV_HIGH  (0x09U)
#define STUSB4500_REG_DEVICE_CAPAB_HIGH   (0x0AU)
#define STUSB4500_REG_DEVICE_ID           (0x2FU)

/* Interrupt (ALERT) */
#define STUSB4500_REG_ALERT_STATUS_1      (0x0BU) /* read-clear */
#define STUSB4500_REG_ALERT_STATUS_1_MASK (0x0CU)

/* Connection detection */
#define STUSB4500_REG_PORT_STATUS_0       (0x0DU) /* transitions, read-clear */
#define STUSB4500_REG_PORT_STATUS_1       (0x0EU) /* current state           */

/* VBUS / VCONN monitoring */
#define STUSB4500_REG_MONITORING_STATUS_0 (0x0FU) /* transitions, read-clear */
#define STUSB4500_REG_MONITORING_STATUS_1 (0x10U) /* current state           */

/* Type-C and hardware fault */
#define STUSB4500_REG_CC_STATUS           (0x11U)
#define STUSB4500_REG_HW_FAULT_STATUS_0   (0x12U) /* transitions, read-clear */
#define STUSB4500_REG_HW_FAULT_STATUS_1   (0x13U) /* current state           */
#define STUSB4500_REG_PD_TYPEC_STATUS     (0x14U)
#define STUSB4500_REG_TYPEC_STATUS        (0x15U)

/* Protocol and physical layer */
#define STUSB4500_REG_PRT_STATUS          (0x16U)
#define STUSB4500_REG_PHY_STATUS          (0x17U)
#define STUSB4500_REG_CC_CAPABILITY_CTRL  (0x18U)
#define STUSB4500_REG_PRT_TX_CTRL         (0x19U)
#define STUSB4500_REG_PD_COMMAND_CTRL     (0x1AU)
#define STUSB4500_REG_DEVICE_CTRL         (0x1DU)

/* Control */
#define STUSB4500_REG_MONITORING_CTRL_0   (0x20U)
#define STUSB4500_REG_MONITORING_CTRL_1   (0x21U) /* undocumented, 100 mV/LSB */
#define STUSB4500_REG_MONITORING_CTRL_2   (0x22U)
#define STUSB4500_REG_RESET_CTRL          (0x23U)
#define STUSB4500_REG_VBUS_DISCHARGE_TIME (0x25U)
#define STUSB4500_REG_VBUS_DISCHARGE_CTRL (0x26U)
#define STUSB4500_REG_VBUS_CTRL           (0x27U)
#define STUSB4500_REG_PE_FSM              (0x29U)
#define STUSB4500_REG_GPIO3_SW            (0x2DU)

/* Received message: byte count, 16-bit header, then up to 7 32-bit objects
 * at RX_DATA_OBJ + 4*n. */
#define STUSB4500_REG_RX_BYTE_CNT         (0x30U)
#define STUSB4500_REG_RX_HEADER           (0x31U)
#define STUSB4500_REG_RX_DATA_OBJ         (0x33U)

/* Transmitted message.  TX_DATA_OBJ shares its address with the NVM
 * read/write window; only one of the two is in use at a time. */
#define STUSB4500_REG_TX_BYTE_CNT         (0x50U)
#define STUSB4500_REG_TX_HEADER           (0x51U)
#define STUSB4500_REG_TX_DATA_OBJ         (0x53U)

/* Device policy manager: the sink PDOs the device advertises, and the
 * request object describing the contract in force. */
#define STUSB4500_REG_DPM_PDO_NUMB        (0x70U)
#define STUSB4500_REG_DPM_SNK_PDO1        (0x85U)
#define STUSB4500_REG_DPM_SNK_PDO2        (0x89U)
#define STUSB4500_REG_DPM_SNK_PDO3        (0x8DU)
#define STUSB4500_REG_RDO_REG_STATUS      (0x91U)

/* NVM (FTP) controller */
#define STUSB4500_REG_RW_BUFFER           (0x53U)
#define STUSB4500_REG_FTP_CUST_PASSWORD   (0x95U)
#define STUSB4500_REG_FTP_CTRL_0          (0x96U)
#define STUSB4500_REG_FTP_CTRL_1          (0x97U)

/* -------------------------------------------------------------------------
 * DEVICE_ID (2Fh).  Both silicon variants are accepted by
 * stusb4500_init(); nothing in this driver depends on which one it is.
 * ------------------------------------------------------------------------- */
#define STUSB4500_DEVICE_ID_A (0x25U) /* STUSB4500  */
#define STUSB4500_DEVICE_ID_B (0x21U) /* STUSB4500B */

/* -------------------------------------------------------------------------
 * ALERT_STATUS_1 (0Bh) and ALERT_STATUS_1_MASK (0Ch).
 *
 * A bit is set when the matching transition register changes.  Setting the
 * bit in the mask register suppresses the ALERT pin for that source; the
 * status bit still latches.
 * ------------------------------------------------------------------------- */
#define STUSB4500_ALERT_PHY_STATUS        (0x01U)
#define STUSB4500_ALERT_PRT_STATUS        (0x02U)
#define STUSB4500_ALERT_PD_TYPEC_STATUS   (0x08U)
#define STUSB4500_ALERT_HW_FAULT_STATUS   (0x10U)
#define STUSB4500_ALERT_MONITORING_STATUS (0x20U)
#define STUSB4500_ALERT_CC_DETECTION      (0x40U)
#define STUSB4500_ALERT_HARD_RESET        (0x80U)

/* -------------------------------------------------------------------------
 * PORT_STATUS_0 (0Dh) / PORT_STATUS_1 (0Eh)
 * ------------------------------------------------------------------------- */
#define STUSB4500_PORT_TRANS_ATTACH      (0x01U)

#define STUSB4500_PORT_ATTACHED          (0x01U)
#define STUSB4500_PORT_VCONN_SUPPLIED    (0x02U)
#define STUSB4500_PORT_DATA_ROLE_DFP     (0x04U)
#define STUSB4500_PORT_POWER_ROLE_SOURCE (0x08U)
#define STUSB4500_PORT_STARTUP_POWER     (0x10U)
#define STUSB4500_PORT_ATTACH_MODE_MASK  (0xE0U)
#define STUSB4500_PORT_ATTACH_MODE_SHIFT (5U)

/* PORT_STATUS_1 CC_ATTACH_MODE field values. */
#define STUSB4500_ATTACH_MODE_NONE        (0U)
#define STUSB4500_ATTACH_MODE_SINK        (1U)
#define STUSB4500_ATTACH_MODE_SOURCE      (2U)
#define STUSB4500_ATTACH_MODE_DEBUG       (3U)
#define STUSB4500_ATTACH_MODE_AUDIO       (4U)
#define STUSB4500_ATTACH_MODE_POWERED     (5U)

/* -------------------------------------------------------------------------
 * MONITORING_STATUS_0 (0Fh) / MONITORING_STATUS_1 (10h)
 *
 * The two registers do NOT share a layout: 0Fh holds four transition flags
 * plus two live comparator outputs, 10h holds the four live states.
 * ------------------------------------------------------------------------- */
#define STUSB4500_MONITOR_TRANS_VCONN_VALID    (0x01U)
#define STUSB4500_MONITOR_TRANS_VBUS_VALID_SNK (0x02U)
#define STUSB4500_MONITOR_TRANS_VBUS_VSAFE0V   (0x04U)
#define STUSB4500_MONITOR_TRANS_VBUS_READY     (0x08U)
#define STUSB4500_MONITOR_VBUS_LOW             (0x10U)
#define STUSB4500_MONITOR_VBUS_HIGH            (0x20U)

#define STUSB4500_MONITOR_VCONN_VALID          (0x01U)
#define STUSB4500_MONITOR_VBUS_VALID_SNK       (0x02U)
#define STUSB4500_MONITOR_VBUS_VSAFE0V         (0x04U)
#define STUSB4500_MONITOR_VBUS_READY           (0x08U)

/* -------------------------------------------------------------------------
 * CC_STATUS (11h)
 * ------------------------------------------------------------------------- */
#define STUSB4500_CC_CC1_STATE_MASK      (0x03U)
#define STUSB4500_CC_CC1_STATE_SHIFT     (0U)
#define STUSB4500_CC_CC2_STATE_MASK      (0x0CU)
#define STUSB4500_CC_CC2_STATE_SHIFT     (2U)
#define STUSB4500_CC_CONNECT_RESULT      (0x10U) /* 1: presenting Rd (sink) */
#define STUSB4500_CC_LOOKING_4_CONNECTION (0x20U)

/* -------------------------------------------------------------------------
 * HW_FAULT_STATUS_0 (12h) / HW_FAULT_STATUS_1 (13h).  Again not a shared
 * layout: bit 7 of 12h is a live thermal flag, not a transition.
 * ------------------------------------------------------------------------- */
#define STUSB4500_FAULT_TRANS_VCONN_SW_OVP   (0x01U)
#define STUSB4500_FAULT_TRANS_VCONN_SW_OCP   (0x02U)
#define STUSB4500_FAULT_TRANS_VCONN_SW_RVP   (0x04U)
#define STUSB4500_FAULT_TRANS_VSRC_DISCH     (0x08U)
#define STUSB4500_FAULT_TRANS_VPU_PRESENCE   (0x10U)
#define STUSB4500_FAULT_TRANS_VPU_OVP        (0x20U)
#define STUSB4500_FAULT_THERMAL              (0x80U)

#define STUSB4500_FAULT_VCONN_SW_OVP         (0x01U)
#define STUSB4500_FAULT_VCONN_SW_OCP         (0x02U)
#define STUSB4500_FAULT_VCONN_SW_RVP         (0x04U)
#define STUSB4500_FAULT_VSRC_DISCH           (0x08U)
#define STUSB4500_FAULT_VBUS_DISCH           (0x20U)
#define STUSB4500_FAULT_VPU_PRESENCE         (0x40U)
#define STUSB4500_FAULT_VPU_OVP              (0x80U)

/* -------------------------------------------------------------------------
 * PD_TYPEC_STATUS (14h) and TYPEC_STATUS (15h)
 * ------------------------------------------------------------------------- */
#define STUSB4500_PD_TYPEC_HAND_CHECK_MASK (0x0FU)

#define STUSB4500_TYPEC_FSM_STATE_MASK     (0x1FU)
#define STUSB4500_TYPEC_PD_SNK_TX_RP       (0x20U)
#define STUSB4500_TYPEC_PD_SRC_TX_RP       (0x40U)
#define STUSB4500_TYPEC_REVERSE            (0x80U) /* 0: CC1, 1: CC2 attached */

/* TYPEC_STATUS FSM states. */
#define STUSB4500_TYPEC_SNK_UNATTACHED        (0x00U)
#define STUSB4500_TYPEC_SNK_ATTACHWAIT        (0x01U)
#define STUSB4500_TYPEC_SNK_ATTACHED          (0x02U)
#define STUSB4500_TYPEC_SNK_2_SRC_PR_SWAP     (0x06U)
#define STUSB4500_TYPEC_SNK_TRYWAIT           (0x07U)
#define STUSB4500_TYPEC_SRC_UNATTACHED        (0x08U)
#define STUSB4500_TYPEC_SRC_ATTACHWAIT        (0x09U)
#define STUSB4500_TYPEC_SRC_ATTACHED          (0x0AU)
#define STUSB4500_TYPEC_SRC_2_SNK_PR_SWAP     (0x0BU)
#define STUSB4500_TYPEC_SRC_TRY               (0x0CU)
#define STUSB4500_TYPEC_ACCESSORY_UNATTACHED  (0x0DU)
#define STUSB4500_TYPEC_ACCESSORY_ATTACHWAIT  (0x0EU)
#define STUSB4500_TYPEC_ACCESSORY_AUDIO       (0x0FU)
#define STUSB4500_TYPEC_ACCESSORY_DEBUG       (0x10U)
#define STUSB4500_TYPEC_ACCESSORY_POWERED     (0x11U)
#define STUSB4500_TYPEC_ACCESSORY_UNSUPPORTED (0x12U)
#define STUSB4500_TYPEC_ERROR_RECOVERY        (0x13U)

/* -------------------------------------------------------------------------
 * PRT_STATUS (16h) and PHY_STATUS (17h)
 * ------------------------------------------------------------------------- */
#define STUSB4500_PRT_HWRESET_RECEIVED (0x01U)
#define STUSB4500_PRT_HWRESET_DONE     (0x02U)
#define STUSB4500_PRT_MSG_RECEIVED     (0x04U)
#define STUSB4500_PRT_MSG_SENT         (0x08U)
#define STUSB4500_PRT_BIST_RECEIVED    (0x10U)
#define STUSB4500_PRT_BIST_SENT        (0x20U)
#define STUSB4500_PRT_TX_ERROR         (0x80U)

#define STUSB4500_PHY_TX_MSG_FAIL      (0x01U)
#define STUSB4500_PHY_TX_MSG_DISC      (0x02U)
#define STUSB4500_PHY_TX_MSG_SUCCESS   (0x04U)
#define STUSB4500_PHY_IDLE             (0x08U)
#define STUSB4500_PHY_SOP_RX_TYPE_MASK (0xE0U)
#define STUSB4500_PHY_SOP_RX_TYPE_SHIFT (5U)

/* -------------------------------------------------------------------------
 * Control register fields
 * ------------------------------------------------------------------------- */
#define STUSB4500_MONITORING_VBUS_SNK_DISC_THRESHOLD (0x08U)

#define STUSB4500_MONITORING_VSHIFT_LOW_MASK   (0x0FU)
#define STUSB4500_MONITORING_VSHIFT_LOW_SHIFT  (0U)
#define STUSB4500_MONITORING_VSHIFT_HIGH_MASK  (0xF0U)
#define STUSB4500_MONITORING_VSHIFT_HIGH_SHIFT (4U)

#define STUSB4500_RESET_SW_EN          (0x01U)
#define STUSB4500_VBUS_DISCHARGE_EN    (0x80U)
#define STUSB4500_VBUS_SINK_EN         (0x02U)
#define STUSB4500_GPIO3_SW_EN          (0x01U)

/* PD_COMMAND_CTRL (1Ah): the value that tells the protocol layer to
 * transmit whatever TX_HEADER currently holds. */
#define STUSB4500_PD_CMD_SEND_MESSAGE  (0x26U)

/* -------------------------------------------------------------------------
 * Policy engine states (PE_FSM, 29h)
 * ------------------------------------------------------------------------- */
#define STUSB4500_PE_INIT                     (0x00U)
#define STUSB4500_PE_SOFT_RESET               (0x01U)
#define STUSB4500_PE_HARD_RESET               (0x02U)
#define STUSB4500_PE_SEND_SOFT_RESET          (0x03U)
#define STUSB4500_PE_BIST                     (0x04U)
#define STUSB4500_PE_DISABLED                 (0x0DU)
#define STUSB4500_PE_SNK_STARTUP              (0x12U)
#define STUSB4500_PE_SNK_DISCOVERY            (0x13U)
#define STUSB4500_PE_SNK_WAIT_FOR_CAPABILITIES (0x14U)
#define STUSB4500_PE_SNK_EVALUATE_CAPABILITIES (0x15U)
#define STUSB4500_PE_SNK_SELECT_CAPABILITIES  (0x16U)
#define STUSB4500_PE_SNK_TRANSITION_SINK      (0x17U)
#define STUSB4500_PE_SNK_READY                (0x18U)
#define STUSB4500_PE_SNK_READY_SENDING        (0x19U)
#define STUSB4500_PE_HARD_RESET_SHUTDOWN      (0x3AU)
#define STUSB4500_PE_HARD_RESET_RECOVERY      (0x3BU)
#define STUSB4500_PE_ERROR_RECOVERY           (0x40U)

/* -------------------------------------------------------------------------
 * USB PD message header (spec Table 6-1) and message types (Table 6-5).
 *
 * The device fills in message ID, roles and revision itself; only the
 * message type has to be supplied when asking it to transmit.
 * ------------------------------------------------------------------------- */
#define STUSB4500_HEADER_MSG_TYPE_MASK    (0x001FU)
#define STUSB4500_HEADER_MSG_TYPE_SHIFT   (0U)
#define STUSB4500_HEADER_PORT_DATA_ROLE   (0x0020U)
#define STUSB4500_HEADER_SPEC_REV_MASK    (0x00C0U)
#define STUSB4500_HEADER_PORT_POWER_ROLE  (0x0100U)
#define STUSB4500_HEADER_MSG_ID_MASK      (0x0E00U)
#define STUSB4500_HEADER_MSG_ID_SHIFT     (9U)
#define STUSB4500_HEADER_NUM_DATA_OBJ_MASK  (0x7000U)
#define STUSB4500_HEADER_NUM_DATA_OBJ_SHIFT (12U)
#define STUSB4500_HEADER_EXTENDED         (0x8000U)

/* Control messages (no data objects). */
#define STUSB4500_CTRL_MSG_GOOD_CRC       (0x01U)
#define STUSB4500_CTRL_MSG_GOTO_MIN       (0x02U)
#define STUSB4500_CTRL_MSG_ACCEPT         (0x03U)
#define STUSB4500_CTRL_MSG_REJECT         (0x04U)
#define STUSB4500_CTRL_MSG_PING           (0x05U)
#define STUSB4500_CTRL_MSG_PS_RDY         (0x06U)
#define STUSB4500_CTRL_MSG_GET_SOURCE_CAP (0x07U)
#define STUSB4500_CTRL_MSG_GET_SINK_CAP   (0x08U)
#define STUSB4500_CTRL_MSG_DR_SWAP        (0x09U)
#define STUSB4500_CTRL_MSG_PR_SWAP        (0x0AU)
#define STUSB4500_CTRL_MSG_VCONN_SWAP     (0x0BU)
#define STUSB4500_CTRL_MSG_WAIT           (0x0CU)
#define STUSB4500_CTRL_MSG_SOFT_RESET     (0x0DU)

/* Data messages (one or more data objects). */
#define STUSB4500_DATA_MSG_SOURCE_CAP     (0x01U)
#define STUSB4500_DATA_MSG_REQUEST        (0x02U)
#define STUSB4500_DATA_MSG_BIST           (0x03U)
#define STUSB4500_DATA_MSG_SINK_CAP       (0x04U)
#define STUSB4500_DATA_MSG_VENDOR_DEFINED (0x0FU)

/* -------------------------------------------------------------------------
 * Power data object encoding (USB PD spec section 6.4.1).
 *
 * Only fixed-supply objects carry a single voltage; variable and battery
 * objects are decoded as far as their type and left to the caller.
 * ------------------------------------------------------------------------- */
#define STUSB4500_PDO_TYPE_MASK         (0xC0000000UL)
#define STUSB4500_PDO_TYPE_SHIFT        (30U)
#define STUSB4500_PDO_TYPE_FIXED        (0U)
#define STUSB4500_PDO_TYPE_BATTERY      (1U)
#define STUSB4500_PDO_TYPE_VARIABLE     (2U)
#define STUSB4500_PDO_TYPE_AUGMENTED    (3U)

#define STUSB4500_PDO_CURRENT_MASK      (0x000003FFUL)
#define STUSB4500_PDO_CURRENT_SHIFT     (0U)
#define STUSB4500_PDO_VOLTAGE_MASK      (0x000FFC00UL)
#define STUSB4500_PDO_VOLTAGE_SHIFT     (10U)

/* Fixed-supply flags, meaningful in both source and sink objects. */
#define STUSB4500_PDO_FAST_ROLE_SWAP_MASK  (0x01800000UL)
#define STUSB4500_PDO_FAST_ROLE_SWAP_SHIFT (23U)
#define STUSB4500_PDO_DUAL_ROLE_DATA       (0x02000000UL)
#define STUSB4500_PDO_USB_COMM_CAPABLE     (0x04000000UL)
#define STUSB4500_PDO_UNCONSTRAINED_POWER  (0x08000000UL)
#define STUSB4500_PDO_HIGHER_CAPABILITY    (0x10000000UL)
#define STUSB4500_PDO_DUAL_ROLE_POWER      (0x20000000UL)

/* Request data object encoding (spec section 6.4.2). */
#define STUSB4500_RDO_MAX_CURRENT_MASK  (0x000003FFUL)
#define STUSB4500_RDO_MAX_CURRENT_SHIFT (0U)
#define STUSB4500_RDO_OP_CURRENT_MASK   (0x000FFC00UL)
#define STUSB4500_RDO_OP_CURRENT_SHIFT  (10U)
#define STUSB4500_RDO_UNCHUNKED_SUPPORT (0x00800000UL)
#define STUSB4500_RDO_NO_USB_SUSPEND    (0x01000000UL)
#define STUSB4500_RDO_USB_COMM_CAPABLE  (0x02000000UL)
#define STUSB4500_RDO_CAPABILITY_MISMATCH (0x04000000UL)
#define STUSB4500_RDO_GIVE_BACK         (0x08000000UL)
#define STUSB4500_RDO_OBJECT_POS_MASK   (0x70000000UL)
#define STUSB4500_RDO_OBJECT_POS_SHIFT  (28U)

/* PDO/RDO field resolutions. */
#define STUSB4500_PDO_VOLTAGE_LSB_MV (50U)
#define STUSB4500_PDO_CURRENT_LSB_MA (10U)

/* The device advertises at most three sink PDOs; a source may offer seven. */
#define STUSB4500_SINK_PDO_COUNT   (3U)
#define STUSB4500_SOURCE_PDO_MAX   (7U)

/* PDO1 is required by the spec to be the 5 V vSafe5V object, and its
 * voltage is neither stored in NVM nor meaningfully writable. */
#define STUSB4500_PDO1_VOLTAGE_MV (5000U)

/* -------------------------------------------------------------------------
 * Nonvolatile memory geometry and FTP controller
 * ------------------------------------------------------------------------- */
#define STUSB4500_NVM_SECTOR_COUNT (5U)
#define STUSB4500_NVM_SECTOR_SIZE  (8U)
#define STUSB4500_NVM_SIZE         (STUSB4500_NVM_SECTOR_COUNT * STUSB4500_NVM_SECTOR_SIZE)

/* Written to FTP_CUST_PASSWORD (95h) to unlock the FTP controller, and
 * written back to zero to lock it again. */
#define STUSB4500_FTP_PASSWORD (0x47U)

/* FTP_CTRL_0 (96h) */
#define STUSB4500_FTP_CUST_PWR   (0x80U)
#define STUSB4500_FTP_CUST_RST_N (0x40U)
#define STUSB4500_FTP_CUST_REQ   (0x10U)
#define STUSB4500_FTP_CUST_SECT  (0x07U)

/* FTP_CTRL_1 (97h) */
#define STUSB4500_FTP_CUST_SER_MASK  (0xF8U)
#define STUSB4500_FTP_CUST_SER_SHIFT (3U)
#define STUSB4500_FTP_CUST_OPCODE    (0x07U)

/* FTP_CUST_OPCODE values. */
#define STUSB4500_FTP_OP_READ_SECTOR (0x00U)
#define STUSB4500_FTP_OP_WRITE_PL    (0x01U)
#define STUSB4500_FTP_OP_WRITE_SER   (0x02U)
#define STUSB4500_FTP_OP_READ_PL     (0x03U)
#define STUSB4500_FTP_OP_READ_SER    (0x04U)
#define STUSB4500_FTP_OP_ERASE       (0x05U)
#define STUSB4500_FTP_OP_PROGRAM     (0x06U)
#define STUSB4500_FTP_OP_SOFT_PROG   (0x07U)

/* Sector-erase register bitmap, one bit per sector. */
#define STUSB4500_FTP_SECTOR_0   (0x01U)
#define STUSB4500_FTP_SECTOR_1   (0x02U)
#define STUSB4500_FTP_SECTOR_2   (0x04U)
#define STUSB4500_FTP_SECTOR_3   (0x08U)
#define STUSB4500_FTP_SECTOR_4   (0x10U)
#define STUSB4500_FTP_SECTOR_ALL (0x1FU)

/* -------------------------------------------------------------------------
 * Timings, milliseconds.
 *
 * The datasheet gives the RESET pulse minimum in microseconds; this
 * driver's delay hook is millisecond-granular, so one millisecond is used
 * and is comfortably above it.  The FTP and startup figures are not
 * datasheet numbers: every operation that uses them is polled to
 * completion, and they only bound how long a dead device is waited on.
 * ------------------------------------------------------------------------- */
#define STUSB4500_T_RESET_MS           (1U)
#define STUSB4500_T_POLL_INTERVAL_MS   (1U)
#define STUSB4500_T_STARTUP_TIMEOUT_MS (100U)
#define STUSB4500_T_FTP_TIMEOUT_MS     (100U)

/* Bound on each wait in stusb4500_negotiate().  Generous against the PD
 * specification's own timers: tSenderResponse is 30 ms and tPSTransition
 * 550 ms, so a source that has not answered by here is not going to. */
#define STUSB4500_T_NEGOTIATION_TIMEOUT_MS (600U)

/* -------------------------------------------------------------------------
 * Error codes.  Every distinct failure condition has its own value so the
 * caller can act on it without the driver emitting any diagnostic output.
 * ------------------------------------------------------------------------- */
typedef enum
{
    STUSB4500_OK = 0,

    /* Platform / transport */
    STUSB4500_ERR_I2C_READ,           /* stusb4500_i2c_read reported failure  */
    STUSB4500_ERR_I2C_WRITE,          /* stusb4500_i2c_write reported failure */
    STUSB4500_ERR_NO_PLATFORM_READ,   /* weak I2C read hook not overridden    */
    STUSB4500_ERR_NO_PLATFORM_WRITE,  /* weak I2C write hook not overridden   */
    STUSB4500_ERR_NO_PLATFORM_RESET,  /* weak RESET-pin hook not overridden   */

    /* Argument validation */
    STUSB4500_ERR_NULL_PARAM,         /* mandatory output pointer was NULL    */
    STUSB4500_ERR_BAD_ADDRESS,        /* I2C address outside 0x28-0x2B        */
    STUSB4500_ERR_BAD_LENGTH,         /* zero-length or oversized transfer    */
    STUSB4500_ERR_BAD_PDO_INDEX,      /* sink PDO index outside 1-3           */
    STUSB4500_ERR_BAD_SECTOR,         /* NVM sector index outside 0-4         */
    STUSB4500_ERR_BAD_PARAM,          /* value outside the encodable range    */
    STUSB4500_ERR_NOT_INITIALISED,    /* stusb4500_init has not succeeded yet */

    /* Identity */
    STUSB4500_ERR_DEVICE_ID,          /* DEVICE_ID did not match an STUSB4500 */

    /* Type-C / PD state */
    STUSB4500_ERR_NOT_ATTACHED,       /* nothing attached on CC               */
    STUSB4500_ERR_NOT_READY,          /* policy engine never reached SNK_READY*/
    STUSB4500_ERR_NO_MESSAGE,         /* expected message did not arrive      */
    STUSB4500_ERR_MESSAGE_TRUNCATED,  /* RX byte count disagreed with header  */
    STUSB4500_ERR_NO_SUITABLE_PDO,    /* no offered PDO met the constraints   */

    /* Reset */
    STUSB4500_ERR_RESET_TIMEOUT,      /* device did not answer after reset    */

    /* Nonvolatile memory */
    STUSB4500_ERR_FTP_TIMEOUT,        /* FTP_CUST_REQ never cleared           */
    STUSB4500_ERR_NVM_VERIFY          /* NVM read-back did not match          */
} stusb4500_error_t;

/* -------------------------------------------------------------------------
 * Decoded register views
 * ------------------------------------------------------------------------- */

/** @brief ALERT_STATUS_1 (0Bh) — which transition register changed. */
typedef struct
{
    uint8_t raw;
    uint8_t phy_status;
    uint8_t prt_status;
    uint8_t pd_typec_status;
    uint8_t hw_fault_status;
    uint8_t monitoring_status;
    uint8_t cc_detection;
    uint8_t hard_reset;
} stusb4500_alert_t;

/** @brief PORT_STATUS_1 (0Eh) — the live connection state. */
typedef struct
{
    uint8_t raw;
    uint8_t attached;
    uint8_t vconn_supplied;
    uint8_t data_role_dfp;    /* 0: UFP    */
    uint8_t power_role_source; /* 0: sink   */
    uint8_t startup_power_mode;
    uint8_t attach_mode;      /* STUSB4500_ATTACH_MODE_* */
} stusb4500_port_status_t;

/** @brief MONITORING_STATUS_1 (10h) plus the two comparators in 0Fh. */
typedef struct
{
    uint8_t raw;             /* MONITORING_STATUS_1 */
    uint8_t raw_transitions; /* MONITORING_STATUS_0 */
    uint8_t vconn_valid;
    uint8_t vbus_valid_snk;
    uint8_t vbus_vsafe0v;
    uint8_t vbus_ready;
    uint8_t vbus_low;        /* below the low monitoring window  */
    uint8_t vbus_high;       /* above the high monitoring window */
} stusb4500_monitoring_t;

/** @brief CC_STATUS (11h). */
typedef struct
{
    uint8_t raw;
    uint8_t cc1_state;
    uint8_t cc2_state;
    uint8_t presenting_rd;        /* CONNECT_RESULT: attached as a sink */
    uint8_t looking_for_connection;
} stusb4500_cc_status_t;

/** @brief HW_FAULT_STATUS_1 (13h) plus the thermal flag from 12h. */
typedef struct
{
    uint8_t raw;             /* HW_FAULT_STATUS_1 */
    uint8_t raw_transitions; /* HW_FAULT_STATUS_0 */
    uint8_t vconn_sw_ovp;
    uint8_t vconn_sw_ocp;
    uint8_t vconn_sw_rvp;
    uint8_t vsrc_discharge;
    uint8_t vbus_discharge;
    uint8_t vpu_presence;
    uint8_t vpu_ovp;
    uint8_t thermal;         /* over-temperature, from 12h bit 7 */
} stusb4500_hw_fault_t;

/** @brief PRT_STATUS (16h) — protocol layer events. */
typedef struct
{
    uint8_t raw;
    uint8_t hard_reset_received;
    uint8_t hard_reset_done;
    uint8_t message_received;
    uint8_t message_sent;
    uint8_t bist_received;
    uint8_t bist_sent;
    uint8_t tx_error;
} stusb4500_prt_status_t;

/** @brief PHY_STATUS (17h) — physical layer state. */
typedef struct
{
    uint8_t raw;
    uint8_t tx_message_failed;
    uint8_t tx_message_discarded;
    uint8_t tx_message_succeeded;
    uint8_t idle;
    uint8_t sop_rx_type;
} stusb4500_phy_status_t;

/** @brief TYPEC_STATUS (15h). */
typedef struct
{
    uint8_t raw;
    uint8_t fsm_state;   /* STUSB4500_TYPEC_* */
    uint8_t snk_tx_rp;
    uint8_t src_tx_rp;
    uint8_t cc2_attached; /* REVERSE: the flipped orientation */
} stusb4500_typec_status_t;

/** @brief One power data object, decoded into engineering units. */
typedef struct
{
    uint32_t raw;
    uint8_t  type;        /* STUSB4500_PDO_TYPE_*                        */
    uint16_t voltage_mv;  /* fixed supplies only; 0 otherwise            */
    uint16_t current_ma;  /* fixed supplies only; 0 otherwise            */
    uint8_t  dual_role_power;
    uint8_t  higher_capability;
    uint8_t  unconstrained_power;
    uint8_t  usb_comm_capable;
    uint8_t  dual_role_data;
} stusb4500_pdo_t;

/** @brief The request object describing the contract currently in force. */
typedef struct
{
    uint32_t raw;
    uint8_t  object_position;  /* 1-based index into the source's PDOs; 0 = none */
    uint16_t operating_current_ma;
    uint16_t max_current_ma;
    uint8_t  capability_mismatch;
    uint8_t  give_back;
    uint8_t  usb_comm_capable;
    uint8_t  no_usb_suspend;
    uint8_t  unchunked_supported;
} stusb4500_rdo_t;

/* -------------------------------------------------------------------------
 * NVM configuration
 * ------------------------------------------------------------------------- */

/** @brief NVM GPIO_CFG field — what the GPIO3 pin does. */
typedef enum
{
    STUSB4500_GPIO_SW_CTRL       = 0U, /* driven by the GPIO3_SW register    */
    STUSB4500_GPIO_ERROR_RECOVERY = 1U, /* default: low in error recovery    */
    STUSB4500_GPIO_DEBUG          = 2U, /* low when a debug accessory is on  */
    STUSB4500_GPIO_SINK_POWER     = 3U  /* low while sinking >= 1.5 A Type-C */
} stusb4500_gpio_cfg_t;

/**
 * @brief NVM POWER_OK_CFG field — what the POWER_OK2/3 pins report.
 *
 * The field is two bits and value 0 behaves as configuration 1, so a decoded
 * config may report either 0 or 1 for it.
 */
typedef enum
{
    STUSB4500_POWER_OK_CFG_1  = 1U, /* both pins unused (high impedance)     */
    STUSB4500_POWER_OK_CFG_2  = 2U, /* default: which sink PDO is in force   */
    STUSB4500_POWER_OK_CFG_3  = 3U  /* Type-C current advertised by the source */
} stusb4500_power_ok_cfg_t;

/**
 * @brief The NVM contents that are documented and worth naming.
 *
 * Everything in here is a view onto the 40 raw bytes; the sectors also hold
 * reserved fields whose values must be preserved, which is why the config
 * setters are read-modify-write over a sector image rather than a fresh
 * build.  Fields the driver cannot express are left untouched.
 */
typedef struct
{
    /* Identity (sector 0), read-only in practice. */
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t bcd_device_id;

    /* Pin behaviour and VBUS discharge (sector 1). */
    uint8_t  gpio_cfg;                  /* stusb4500_gpio_cfg_t            */
    uint8_t  vbus_discharge_masked;     /* 1: suppress automatic discharge */
    uint16_t discharge_to_pdo_ms;       /* 20 ms per LSB, 0..300           */
    uint16_t discharge_to_0v_ms;        /* 84 ms per LSB, 0..1260          */

    /* Sink policy (sectors 3 and 4). */
    uint8_t  pdo_count;                 /* DPM_SNK_PDO_NUMB, 1..3          */
    uint8_t  usb_comm_capable;
    uint8_t  unconstrained_power;
    uint8_t  power_ok_cfg;              /* stusb4500_power_ok_cfg_t        */
    uint8_t  power_only_above_5v;       /* 1: VBUS_EN_SNK only above 5 V   */
    uint8_t  req_src_current;           /* 1: request the source's current */
    uint8_t  alert_mask;                /* boot value of ALERT_STATUS_1_MASK */

    /**
     * Per-PDO voltage in millivolts.  Index 0 is PDO1 and is always
     * STUSB4500_PDO1_VOLTAGE_MV: the spec fixes it and the NVM has no field
     * for it, so it is reported for completeness and ignored on encode.
     */
    uint16_t pdo_voltage_mv[STUSB4500_SINK_PDO_COUNT];

    /**
     * Per-PDO operating current in milliamps.  The NVM stores these as a
     * 4-bit index into a fixed table, so only the values in that table can
     * be expressed; 0 means "take the current from flex_current_ma".
     * stusb4500_nvm_encode() rounds a request up to the next table entry so
     * the sink never advertises less than it needs.
     */
    uint16_t pdo_current_ma[STUSB4500_SINK_PDO_COUNT];

    /** Current used by any PDO whose table index is 0.  10 mA per LSB. */
    uint16_t flex_current_ma;

    /**
     * Per-PDO voltage acceptance window, percent below and above the
     * negotiated voltage, outside which the device drops the contract.
     */
    uint8_t under_voltage_pct[STUSB4500_SINK_PDO_COUNT];
    uint8_t over_voltage_pct[STUSB4500_SINK_PDO_COUNT];
} stusb4500_nvm_config_t;

/** @brief Static application configuration supplied at init. */
typedef struct
{
    /**
     * 7-bit I2C address, 0x28..0x2B as strapped on ADDR0/ADDR1.  Zero
     * selects STUSB4500_I2C_ADDR_DEFAULT.
     */
    uint8_t i2c_address;
} stusb4500_config_t;

/** @brief Constraints used to pick a contract from a source's offer. */
typedef struct
{
    uint16_t min_voltage_mv;
    uint16_t max_voltage_mv;
    uint16_t min_current_ma;
} stusb4500_request_t;

/* -------------------------------------------------------------------------
 * Platform hooks — implemented by the application.
 *
 * The two transfer hooks and the delay have weak defaults in the driver that
 * report STUSB4500_ERR_NO_PLATFORM_*, so a build that forgets to provide
 * them fails loudly at run time instead of silently reading zeros.
 * ------------------------------------------------------------------------- */

/**
 * @brief Read @p len bytes starting at register @p reg from the 2-wire
 *        device at 7-bit address @p dev_addr.
 *
 * The transfer is a write of the single register-address byte followed by a
 * repeated START and @p len read bytes.
 *
 * @param[in]  dev_addr 7-bit slave address, 0x28..0x2B.
 * @param[in]  reg      Register address byte.
 * @param[out] rx       Destination buffer.
 * @param[in]  len      Number of bytes to read; never zero.
 * @return STUSB4500_OK on success, STUSB4500_ERR_I2C_READ on any bus error.
 */
stusb4500_error_t
stusb4500_i2c_read(const uint8_t dev_addr, const uint8_t reg, uint8_t *rx, const uint16_t len);

/**
 * @brief Write @p len bytes to register @p reg of the device at @p dev_addr.
 * @return STUSB4500_OK on success, STUSB4500_ERR_I2C_WRITE on any bus error.
 */
stusb4500_error_t stusb4500_i2c_write(const uint8_t  dev_addr,
                                      const uint8_t  reg,
                                      const uint8_t *tx,
                                      const uint16_t len);

/**
 * @brief Block for at least @p ms milliseconds.
 */
void stusb4500_delay_ms(const uint32_t ms);

/**
 * @brief Drive the device's active-low RESET pin.
 *
 * Optional: only stusb4500_hardware_reset() uses it, and that returns
 * STUSB4500_ERR_NO_PLATFORM_RESET if the default is still in place.  Boards
 * that tie RESET low should use stusb4500_software_reset() instead.
 *
 * @param[in] asserted Non-zero to hold the device in reset (pin low).
 * @return STUSB4500_OK if the pin was driven.
 */
stusb4500_error_t stusb4500_set_reset_pin(const uint8_t asserted);

/* -------------------------------------------------------------------------
 * Initialisation and raw register access
 * ------------------------------------------------------------------------- */

/**
 * @brief Latch the application configuration and confirm an STUSB4500
 *        answers.
 *
 * Reads DEVICE_ID and checks it against both silicon variants.  Does not
 * modify device configuration and does not disturb any contract already
 * negotiated, so it is safe to call on a port that is already powered.
 *
 * @param[in] cfg Application configuration; copied internally.  NULL is
 *                accepted and selects the default address.
 * @return STUSB4500_OK, STUSB4500_ERR_BAD_ADDRESS, STUSB4500_ERR_DEVICE_ID
 *         or a transport error.
 */
stusb4500_error_t stusb4500_init(const stusb4500_config_t *cfg);

/** @brief Read the 7-bit address the driver is talking to. */
stusb4500_error_t stusb4500_get_i2c_address(uint8_t *address);

stusb4500_error_t stusb4500_read_register(const uint8_t reg, uint8_t *value);
stusb4500_error_t stusb4500_write_register(const uint8_t reg, const uint8_t value);

/** @brief Read @p len consecutive registers using the device's auto-increment. */
stusb4500_error_t stusb4500_read_registers(const uint8_t reg, uint8_t *rx, const uint16_t len);
stusb4500_error_t
stusb4500_write_registers(const uint8_t reg, const uint8_t *tx, const uint16_t len);

/** @brief Read-modify-write a register, replacing only the bits in @p mask. */
stusb4500_error_t
stusb4500_update_register(const uint8_t reg, const uint8_t mask, const uint8_t value);

/** @brief Read a little-endian 16-bit register pair. */
stusb4500_error_t stusb4500_read_word(const uint8_t reg, uint16_t *value);
stusb4500_error_t stusb4500_write_word(const uint8_t reg, const uint16_t value);

/** @brief Read a little-endian 32-bit register quad. */
stusb4500_error_t stusb4500_read_dword(const uint8_t reg, uint32_t *value);
stusb4500_error_t stusb4500_write_dword(const uint8_t reg, const uint32_t value);

/* -------------------------------------------------------------------------
 * Identity
 * ------------------------------------------------------------------------- */

stusb4500_error_t stusb4500_get_device_id(uint8_t *device_id);

/**
 * @brief Read the supported Type-C and USB PD specification revisions.
 *
 * Both are BCD, e.g. 0x0120 for revision 1.2.  Either pointer may be NULL.
 */
stusb4500_error_t stusb4500_get_revisions(uint16_t *typec_bcd, uint16_t *usbpd_bcd);

/* -------------------------------------------------------------------------
 * Status
 * ------------------------------------------------------------------------- */

/**
 * @brief Read and decode ALERT_STATUS_1.
 *
 * The register is read-clear: the flags are consumed by this call, which is
 * what an interrupt handler wants.
 */
stusb4500_error_t stusb4500_get_alerts(stusb4500_alert_t *alerts);

/**
 * @brief Read the ALERT pin mask.  A set bit suppresses that source.
 */
stusb4500_error_t stusb4500_get_alert_mask(uint8_t *mask);
stusb4500_error_t stusb4500_set_alert_mask(const uint8_t mask);

stusb4500_error_t stusb4500_get_port_status(stusb4500_port_status_t *status);
stusb4500_error_t stusb4500_get_monitoring_status(stusb4500_monitoring_t *status);
stusb4500_error_t stusb4500_get_cc_status(stusb4500_cc_status_t *status);
stusb4500_error_t stusb4500_get_hw_faults(stusb4500_hw_fault_t *faults);
stusb4500_error_t stusb4500_get_prt_status(stusb4500_prt_status_t *status);
stusb4500_error_t stusb4500_get_phy_status(stusb4500_phy_status_t *status);
stusb4500_error_t stusb4500_get_typec_status(stusb4500_typec_status_t *status);

/** @brief Read the policy engine state, one of STUSB4500_PE_*. */
stusb4500_error_t stusb4500_get_pe_state(uint8_t *state);

/** @brief Shorthand for PORT_STATUS_1.CC_ATTACH_STATE. */
stusb4500_error_t stusb4500_is_attached(uint8_t *attached);

/** @brief Shorthand for MONITORING_STATUS_1.VBUS_READY. */
stusb4500_error_t stusb4500_is_vbus_ready(uint8_t *ready);

/**
 * @brief Wait until the policy engine reaches PE_SNK_READY.
 *
 * @param[in] timeout_ms Upper bound on the wait.
 * @return STUSB4500_OK or STUSB4500_ERR_NOT_READY.
 */
stusb4500_error_t stusb4500_wait_sink_ready(const uint32_t timeout_ms);

/* -------------------------------------------------------------------------
 * Sink power data objects and the negotiated contract
 * ------------------------------------------------------------------------- */

/**
 * @brief Read or set how many of the three sink PDOs are advertised.
 *
 * This is the live DPM_PDO_NUMB register, not the NVM default, so it is lost
 * on reset.
 */
stusb4500_error_t stusb4500_get_pdo_count(uint8_t *count);
stusb4500_error_t stusb4500_set_pdo_count(const uint8_t count);

/**
 * @brief Read one advertised sink PDO, decoded.  @p index is 1..3.
 */
stusb4500_error_t stusb4500_get_sink_pdo(const uint8_t index, stusb4500_pdo_t *pdo);

/** @brief Write one advertised sink PDO verbatim.  @p index is 1..3. */
stusb4500_error_t stusb4500_set_sink_pdo_raw(const uint8_t index, const uint32_t pdo);

/**
 * @brief Write one advertised sink PDO as a fixed supply.
 *
 * Preserves the flag bits already in the object and replaces only the
 * voltage and current fields, so the USB-communications and
 * unconstrained-power advertisements survive.  Writing PDO1 with a voltage
 * other than 5 V is rejected: the spec fixes it.
 *
 * @param[in] index      1..3.
 * @param[in] voltage_mv 50 mV resolution, up to 51150 mV.
 * @param[in] current_ma 10 mA resolution, up to 10230 mA.
 */
stusb4500_error_t stusb4500_set_sink_pdo(const uint8_t  index,
                                         const uint16_t voltage_mv,
                                         const uint16_t current_ma);

/** @brief Read and decode the request object for the contract in force. */
stusb4500_error_t stusb4500_get_rdo(stusb4500_rdo_t *rdo);

/**
 * @brief Read the source's advertised capabilities.
 *
 * Waits for a Source_Capabilities message, then reads its data objects.  A
 * source sends these unsolicited on attach and again after a soft reset; if
 * none is pending, ask for one with stusb4500_send_pd_soft_reset() first.
 *
 * The read must follow the message closely — the source's next message
 * partially overwrites the receive buffer — so run the bus at 300 kHz or
 * above.
 *
 * @param[out] pdos     Destination array.
 * @param[in]  max_pdos Capacity of @p pdos, at most STUSB4500_SOURCE_PDO_MAX.
 * @param[out] count    Number of objects written.
 * @param[in]  timeout_ms Upper bound on the wait for the message.
 * @return STUSB4500_OK, STUSB4500_ERR_NO_MESSAGE,
 *         STUSB4500_ERR_MESSAGE_TRUNCATED or a transport error.
 */
stusb4500_error_t stusb4500_read_source_capabilities(stusb4500_pdo_t *pdos,
                                                     const uint8_t    max_pdos,
                                                     uint8_t         *count,
                                                     const uint32_t   timeout_ms);

/**
 * @brief Pick the highest-power offer inside @p request and contract for it.
 *
 * Reads the source's capabilities, chooses the fixed-supply object with the
 * greatest voltage*current product that satisfies every constraint, writes
 * it into sink PDO 3, and issues a PD soft reset so the pair renegotiate.
 *
 * PDO3 is used because the device tries its sink PDOs highest-index first,
 * so PDO1's mandatory 5 V object survives as the fallback.  The advertised
 * PDO count is raised to three, which also exposes whatever PDO2 holds.
 * Both are live-register changes and are lost on reset.
 *
 * @param[in]  request  Voltage window and minimum current.
 * @param[out] selected The chosen object; may be NULL.
 * @return STUSB4500_OK, STUSB4500_ERR_NOT_ATTACHED,
 *         STUSB4500_ERR_NO_SUITABLE_PDO, STUSB4500_ERR_NOT_READY,
 *         STUSB4500_ERR_NO_MESSAGE or a transport error.
 */
stusb4500_error_t stusb4500_negotiate(const stusb4500_request_t *request,
                                      stusb4500_pdo_t           *selected);

/* -------------------------------------------------------------------------
 * PD messaging and reset
 * ------------------------------------------------------------------------- */

/**
 * @brief Ask the protocol layer to transmit a message.
 *
 * Only the message type is supplied; the device fills in the message ID,
 * roles and specification revision.  In practice Soft_Reset is the only
 * message the STUSB4500 will originate.
 */
stusb4500_error_t stusb4500_send_pd_message(const uint8_t message_type);

/**
 * @brief Send a PD Soft_Reset, which makes the source re-send its
 *        capabilities and renegotiate against the current sink PDOs.
 */
stusb4500_error_t stusb4500_send_pd_soft_reset(void);

/**
 * @brief Reset the device over I2C.
 *
 * Asserts RESET_CTRL.SW_RESET_EN, releases it, then polls DEVICE_ID until
 * the device answers again.  Equivalent to pulsing the RESET pin: the NVM is
 * reloaded and any RAM PDO overrides are lost.
 *
 * @return STUSB4500_OK or STUSB4500_ERR_RESET_TIMEOUT.
 */
stusb4500_error_t stusb4500_software_reset(void);

/**
 * @brief Reset the device by pulsing its RESET pin.
 *
 * @return STUSB4500_OK, STUSB4500_ERR_NO_PLATFORM_RESET if the board did not
 *         provide stusb4500_set_reset_pin(), or STUSB4500_ERR_RESET_TIMEOUT.
 */
stusb4500_error_t stusb4500_hardware_reset(void);

/* -------------------------------------------------------------------------
 * VBUS monitoring, discharge and GPIO3
 * ------------------------------------------------------------------------- */

/**
 * @brief Set the acceptance window around the negotiated VBUS voltage.
 *
 * Below @p under_pct or above @p over_pct the device flags VBUS_LOW /
 * VBUS_HIGH and drops the contract.  Both are 0..15 percent; zero disables
 * that side.  This is the live register — the NVM holds the boot values.
 */
stusb4500_error_t
stusb4500_set_voltage_window(const uint8_t under_pct, const uint8_t over_pct);
stusb4500_error_t stusb4500_get_voltage_window(uint8_t *under_pct, uint8_t *over_pct);

/** @brief Enable or disable the VBUS discharge path. */
stusb4500_error_t stusb4500_set_vbus_discharge(const uint8_t enabled);

/** @brief Enable or disable the VBUS_EN_SNK output. */
stusb4500_error_t stusb4500_set_vbus_sink_enabled(const uint8_t enabled);

/**
 * @brief Drive the GPIO3 pin.
 *
 * Only takes effect when the NVM GPIO_CFG field is
 * STUSB4500_GPIO_SW_CTRL.  The pin is open-drain: @p low pulls it down,
 * otherwise it floats.
 */
stusb4500_error_t stusb4500_set_gpio3(const uint8_t low);

/* -------------------------------------------------------------------------
 * PDO / RDO encoding helpers
 *
 * Pure functions; usable without a device and without init.
 * ------------------------------------------------------------------------- */

/** @brief Decode a raw 32-bit power data object. */
void stusb4500_decode_pdo(const uint32_t raw, stusb4500_pdo_t *pdo);

/** @brief Decode a raw 32-bit request data object. */
void stusb4500_decode_rdo(const uint32_t raw, stusb4500_rdo_t *rdo);

/**
 * @brief Build a fixed-supply object from millivolts and milliamps.
 *
 * @return STUSB4500_OK or STUSB4500_ERR_BAD_PARAM if either value cannot be
 *         expressed in its 10-bit field.
 */
stusb4500_error_t stusb4500_encode_fixed_pdo(const uint16_t voltage_mv,
                                             const uint16_t current_ma,
                                             uint32_t      *raw);

/* -------------------------------------------------------------------------
 * Nonvolatile memory
 *
 * The NVM has a finite program-cycle budget.  Read freely; write only when
 * the standalone behaviour genuinely has to change, and prefer the RAM
 * PDO registers for anything a running host can set.
 * ------------------------------------------------------------------------- */

/**
 * @brief Read all five sectors into a 40-byte buffer.
 *
 * @param[out] nvm Buffer of STUSB4500_NVM_SIZE bytes, sector 0 first.
 */
stusb4500_error_t stusb4500_nvm_read(uint8_t *nvm);

/** @brief Read one 8-byte sector.  @p sector is 0..4. */
stusb4500_error_t stusb4500_nvm_read_sector(const uint8_t sector, uint8_t *data);

/**
 * @brief Erase and reprogram all five sectors, then verify.
 *
 * Consumes one program cycle.  Returns without touching the device if the
 * requested image already matches what is stored.  The new contents only
 * govern behaviour after a reset, which this function performs.
 *
 * @param[in] nvm Buffer of STUSB4500_NVM_SIZE bytes.
 * @return STUSB4500_OK, STUSB4500_ERR_NVM_VERIFY, STUSB4500_ERR_FTP_TIMEOUT
 *         or a transport error.
 */
stusb4500_error_t stusb4500_nvm_write(const uint8_t *nvm);

/** @brief Decode a 40-byte NVM image into named fields. */
stusb4500_error_t stusb4500_nvm_decode(const uint8_t *nvm, stusb4500_nvm_config_t *cfg);

/**
 * @brief Apply named fields to an existing 40-byte NVM image in place.
 *
 * Read-modify-write: reserved fields and anything the config cannot express
 * are left as they were, so @p nvm must be a real image (from
 * stusb4500_nvm_read() or stusb4500_nvm_factory_default()) rather than a
 * zeroed buffer.
 *
 * @return STUSB4500_OK or STUSB4500_ERR_BAD_PARAM if a value cannot be
 *         encoded.
 */
stusb4500_error_t stusb4500_nvm_encode(uint8_t *nvm, const stusb4500_nvm_config_t *cfg);

/** @brief Read the NVM and decode it in one call. */
stusb4500_error_t stusb4500_nvm_get_config(stusb4500_nvm_config_t *cfg);

/**
 * @brief Read the NVM, apply @p cfg, and write it back if anything changed.
 *
 * The read-modify-write that stusb4500_nvm_encode() documents, done against
 * the device's own current contents.
 */
stusb4500_error_t stusb4500_nvm_set_config(const stusb4500_nvm_config_t *cfg);

/**
 * @brief Copy ST's factory-default 40-byte image into @p nvm.
 *
 * A starting point for a fresh configuration, and the way back from a
 * board that has been programmed into a corner.  Writing it discards
 * whatever was there.
 */
void stusb4500_nvm_factory_default(uint8_t *nvm);

/**
 * @brief Convert between the NVM's 4-bit sink current index and milliamps.
 *
 * The index selects from a fixed 16-entry table; index 0 means the current
 * is taken from the flex field instead, and decodes to 0 mA.
 * stusb4500_nvm_current_to_index() rounds up to the next table entry, and
 * maps 0 mA to index 0.
 */
uint16_t          stusb4500_nvm_index_to_current(const uint8_t index);
stusb4500_error_t stusb4500_nvm_current_to_index(const uint16_t current_ma, uint8_t *index);

#endif /* STUSB4500_H */
