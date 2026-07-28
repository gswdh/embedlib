#ifndef MAX17320_H
#define MAX17320_H

#include <stdint.h>

/* -------------------------------------------------------------------------
 * MAX17320 — 2S-4S ModelGauge m5 EZ fuel gauge with protector, internal
 * self-discharge detection and SHA-256 authentication (Analog Devices).
 *
 * Portable driver: all register logic lives here.  The application supplies
 * the platform layer by implementing the three weakly defined hooks declared
 * below (two I2C transfers plus a millisecond delay used for the documented
 * nonvolatile-memory and reset timings).  The driver performs no printing,
 * logging or allocation; every failure is reported through a distinct
 * max17320_error_t code.
 *
 * The device presents a 9-bit internal register space (000h-1FFh) behind two
 * 2-wire slave addresses:
 *
 *   internal 000h-0FFh  ->  slave 6Ch (7-bit 36h), memory byte = addr & FFh
 *   internal 100h-1FFh  ->  slave 16h (7-bit 0Bh), memory byte = addr & FFh
 *
 * All register words are transferred least-significant byte first.
 * ------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
 * I2C slave addresses
 * ------------------------------------------------------------------------- */
#define MAX17320_I2C_ADDR_LOW_7B  (0x36U) /* internal 000h-0FFh (8-bit 6Ch) */
#define MAX17320_I2C_ADDR_HIGH_7B (0x0BU) /* internal 100h-1FFh (8-bit 16h) */

/* Highest internal address served by the low slave address. */
#define MAX17320_ADDR_SPACE_SPLIT (0x0FFU)
/* Highest internal address that exists at all. */
#define MAX17320_ADDR_MAX         (0x1FFU)

/* -------------------------------------------------------------------------
 * ModelGauge m5 register addresses (slave 6Ch, internal 000h-0FFh)
 * ------------------------------------------------------------------------- */
#define MAX17320_REG_STATUS           (0x000U)
#define MAX17320_REG_VALRTTH          (0x001U)
#define MAX17320_REG_TALRTTH          (0x002U)
#define MAX17320_REG_SALRTTH          (0x003U)
#define MAX17320_REG_ATRATE           (0x004U)
#define MAX17320_REG_REPCAP           (0x005U)
#define MAX17320_REG_REPSOC           (0x006U)
#define MAX17320_REG_AGE              (0x007U)
#define MAX17320_REG_MAXMINVOLT       (0x008U)
#define MAX17320_REG_MAXMINTEMP       (0x009U)
#define MAX17320_REG_MAXMINCURR       (0x00AU)
#define MAX17320_REG_CONFIG           (0x00BU)
#define MAX17320_REG_QRESIDUAL        (0x00CU)
#define MAX17320_REG_MIXSOC           (0x00DU)
#define MAX17320_REG_AVSOC            (0x00EU)
#define MAX17320_REG_MISCCFG          (0x00FU)

#define MAX17320_REG_FULLCAPREP       (0x010U)
#define MAX17320_REG_TTE              (0x011U)
#define MAX17320_REG_QRTABLE00        (0x012U)
#define MAX17320_REG_FULLSOCTHR       (0x013U)
#define MAX17320_REG_RCELL            (0x014U)
#define MAX17320_REG_AVGTA            (0x016U)
#define MAX17320_REG_CYCLES           (0x017U)
#define MAX17320_REG_DESIGNCAP        (0x018U)
#define MAX17320_REG_AVGVCELL         (0x019U)
#define MAX17320_REG_VCELL            (0x01AU)
#define MAX17320_REG_TEMP             (0x01BU)
#define MAX17320_REG_CURRENT          (0x01CU)
#define MAX17320_REG_AVGCURRENT       (0x01DU)
#define MAX17320_REG_ICHGTERM         (0x01EU)
#define MAX17320_REG_AVCAP            (0x01FU)

#define MAX17320_REG_TTF              (0x020U)
#define MAX17320_REG_DEVNAME          (0x021U)
#define MAX17320_REG_QRTABLE10        (0x022U)
#define MAX17320_REG_FULLCAPNOM       (0x023U)
#define MAX17320_REG_CHARGINGCURRENT  (0x028U)
#define MAX17320_REG_FILTERCFG        (0x029U)
#define MAX17320_REG_CHARGINGVOLTAGE  (0x02AU)
#define MAX17320_REG_MIXCAP           (0x02BU)

#define MAX17320_REG_QRTABLE20        (0x032U)
#define MAX17320_REG_DIETEMP          (0x034U)
#define MAX17320_REG_FULLCAP          (0x035U)
#define MAX17320_REG_IAVGEMPTY        (0x036U)
#define MAX17320_REG_RCOMP0           (0x038U)
#define MAX17320_REG_TEMPCO           (0x039U)
#define MAX17320_REG_VEMPTY           (0x03AU)
#define MAX17320_REG_FSTAT            (0x03DU)
#define MAX17320_REG_TIMER            (0x03EU)

#define MAX17320_REG_AVGDIETEMP       (0x040U)
#define MAX17320_REG_QRTABLE30        (0x042U)
#define MAX17320_REG_VFREMCAP         (0x04AU)
#define MAX17320_REG_QH               (0x04DU)
#define MAX17320_REG_QL               (0x04EU)

#define MAX17320_REG_COMMAND          (0x060U)
#define MAX17320_REG_COMMSTAT         (0x061U)
#define MAX17320_REG_LOCK             (0x07FU)

#define MAX17320_REG_RELAXCFG         (0x0A0U)
#define MAX17320_REG_LEARNCFG         (0x0A1U)
#define MAX17320_REG_MAXPEAKPOWER     (0x0A4U)
#define MAX17320_REG_SUSPEAKPOWER     (0x0A5U)
#define MAX17320_REG_PACKRESISTANCE   (0x0A6U)
#define MAX17320_REG_SYSRESISTANCE    (0x0A7U)
#define MAX17320_REG_MINSYSVOLTAGE    (0x0A8U)
#define MAX17320_REG_MPPCURRENT       (0x0A9U)
#define MAX17320_REG_SPPCURRENT       (0x0AAU)
#define MAX17320_REG_CONFIG2          (0x0ABU)
#define MAX17320_REG_IALRTTH          (0x0ACU)
#define MAX17320_REG_MINVOLT          (0x0ADU)
#define MAX17320_REG_MINCURR          (0x0AEU)
#define MAX17320_REG_PROTALRT         (0x0AFU)

#define MAX17320_REG_STATUS2          (0x0B0U)
#define MAX17320_REG_POWER            (0x0B1U)
#define MAX17320_REG_VRIPPLE          (0x0B2U)
#define MAX17320_REG_AVGPOWER         (0x0B3U)
#define MAX17320_REG_TTFCFG           (0x0B5U)
#define MAX17320_REG_CVMIXCAP         (0x0B6U)
#define MAX17320_REG_CVHALFTIME       (0x0B7U)
#define MAX17320_REG_CGTEMPCO         (0x0B8U)
#define MAX17320_REG_AGEFORECAST      (0x0B9U)
#define MAX17320_REG_FOTPSTAT         (0x0BBU)
#define MAX17320_REG_TIMERH           (0x0BEU)

#define MAX17320_REG_SOCHOLD          (0x0D0U)
#define MAX17320_REG_AVGCELL4         (0x0D1U)
#define MAX17320_REG_AVGCELL3         (0x0D2U)
#define MAX17320_REG_AVGCELL2         (0x0D3U)
#define MAX17320_REG_AVGCELL1         (0x0D4U)
#define MAX17320_REG_CELL4            (0x0D5U)
#define MAX17320_REG_CELL3            (0x0D6U)
#define MAX17320_REG_CELL2            (0x0D7U)
#define MAX17320_REG_CELL1            (0x0D8U)
#define MAX17320_REG_PROTSTATUS       (0x0D9U)
#define MAX17320_REG_BATT             (0x0DAU)
#define MAX17320_REG_PCKP             (0x0DBU)
#define MAX17320_REG_ATQRESIDUAL      (0x0DCU)
#define MAX17320_REG_ATTTE            (0x0DDU)
#define MAX17320_REG_ATAVSOC          (0x0DEU)
#define MAX17320_REG_ATAVCAP          (0x0DFU)

#define MAX17320_REG_HPROTCFG2        (0x0F1U)
#define MAX17320_REG_VFOCV            (0x0FBU)
#define MAX17320_REG_VFSOC            (0x0FFU)

/* -------------------------------------------------------------------------
 * Thermistor / leakage registers (slave 16h, internal 100h-17Fh).
 * These must be accessed one word at a time.
 * ------------------------------------------------------------------------- */
#define MAX17320_REG_AVGTEMP4         (0x133U)
#define MAX17320_REG_AVGTEMP3         (0x134U)
#define MAX17320_REG_AVGTEMP2         (0x135U)
#define MAX17320_REG_AVGTEMP1         (0x136U)
#define MAX17320_REG_TEMP4            (0x137U)
#define MAX17320_REG_TEMP3            (0x138U)
#define MAX17320_REG_TEMP2            (0x139U)
#define MAX17320_REG_TEMP1            (0x13AU)
#define MAX17320_REG_LEAKCURRREP      (0x16FU)

/* -------------------------------------------------------------------------
 * Nonvolatile (shadow RAM) register addresses (slave 16h, 180h-1FFh)
 * ------------------------------------------------------------------------- */
#define MAX17320_REG_NXTABLE0         (0x180U) /* .. nXTable11 at 18Bh */
#define MAX17320_REG_NVALRTTH         (0x18CU)
#define MAX17320_REG_NTALRTTH         (0x18DU)
#define MAX17320_REG_NIALRTTH         (0x18EU)
#define MAX17320_REG_NSALRTTH         (0x18FU)

#define MAX17320_REG_NOCVTABLE0       (0x190U) /* .. nOCVTable11 at 19Bh */
#define MAX17320_REG_NICHGTERM        (0x19CU)
#define MAX17320_REG_NFILTERCFG       (0x19DU)
#define MAX17320_REG_NVEMPTY          (0x19EU)
#define MAX17320_REG_NLEARNCFG        (0x19FU)

#define MAX17320_REG_NQRTABLE00       (0x1A0U)
#define MAX17320_REG_NQRTABLE10       (0x1A1U)
#define MAX17320_REG_NQRTABLE20       (0x1A2U)
#define MAX17320_REG_NQRTABLE30       (0x1A3U)
#define MAX17320_REG_NCYCLES          (0x1A4U)
#define MAX17320_REG_NFULLCAPNOM      (0x1A5U)
#define MAX17320_REG_NRCOMP0          (0x1A6U)
#define MAX17320_REG_NTEMPCO          (0x1A7U)
#define MAX17320_REG_NBATTSTATUS      (0x1A8U)
#define MAX17320_REG_NFULLCAPREP      (0x1A9U)
#define MAX17320_REG_NVOLTTEMP        (0x1AAU)
#define MAX17320_REG_NMAXMINCURR      (0x1ABU)
#define MAX17320_REG_NMAXMINVOLT      (0x1ACU)
#define MAX17320_REG_NMAXMINTEMP      (0x1ADU)
#define MAX17320_REG_NFAULTLOG        (0x1AEU)
#define MAX17320_REG_NTIMERH          (0x1AFU)

#define MAX17320_REG_NCONFIG          (0x1B0U)
#define MAX17320_REG_NRIPPLECFG       (0x1B1U)
#define MAX17320_REG_NMISCCFG         (0x1B2U)
#define MAX17320_REG_NDESIGNCAP       (0x1B3U)
#define MAX17320_REG_NSBSCFG          (0x1B4U)
#define MAX17320_REG_NPACKCFG         (0x1B5U)
#define MAX17320_REG_NRELAXCFG        (0x1B6U)
#define MAX17320_REG_NCONVGCFG        (0x1B7U)
#define MAX17320_REG_NNVCFG0          (0x1B8U)
#define MAX17320_REG_NNVCFG1          (0x1B9U)
#define MAX17320_REG_NNVCFG2          (0x1BAU)
#define MAX17320_REG_NHIBCFG          (0x1BBU)
#define MAX17320_REG_NROMID0          (0x1BCU)
#define MAX17320_REG_NROMID1          (0x1BDU)
#define MAX17320_REG_NROMID2          (0x1BEU)
#define MAX17320_REG_NROMID3          (0x1BFU)

#define MAX17320_REG_NPRESERVED0      (0x1C0U)
#define MAX17320_REG_NPRESERVED1      (0x1C1U)
#define MAX17320_REG_NCHGCFG          (0x1C2U)
#define MAX17320_REG_NCHGCTL          (0x1C3U)
#define MAX17320_REG_NRGAIN           (0x1C4U)
#define MAX17320_REG_NPACKRESISTANCE  (0x1C5U)
#define MAX17320_REG_NFULLSOCTHR      (0x1C6U)
#define MAX17320_REG_NTTFCFG          (0x1C7U)
#define MAX17320_REG_NCGAIN           (0x1C8U)
#define MAX17320_REG_NCGTEMPCO        (0x1C9U) /* also named nTCurve */
#define MAX17320_REG_NTHERMCFG        (0x1CAU)
#define MAX17320_REG_NPROTMISCTH2     (0x1CBU)
#define MAX17320_REG_NMANFCTRNAME0    (0x1CCU)
#define MAX17320_REG_NMANFCTRNAME1    (0x1CDU)
#define MAX17320_REG_NMANFCTRNAME2    (0x1CEU)
#define MAX17320_REG_NRSENSE          (0x1CFU)

#define MAX17320_REG_NUVPRTTH         (0x1D0U)
#define MAX17320_REG_NTPRTTH1         (0x1D1U)
#define MAX17320_REG_NTPRTTH3         (0x1D2U)
#define MAX17320_REG_NIPRTTH1         (0x1D3U)
#define MAX17320_REG_NBALTH           (0x1D4U)
#define MAX17320_REG_NTPRTTH2         (0x1D5U)
#define MAX17320_REG_NPROTMISCTH      (0x1D6U)
#define MAX17320_REG_NPROTCFG         (0x1D7U)
#define MAX17320_REG_NJEITAC          (0x1D8U)
#define MAX17320_REG_NJEITAV          (0x1D9U)
#define MAX17320_REG_NOVPRTTH         (0x1DAU)
#define MAX17320_REG_NSTEPCHG         (0x1DBU)
#define MAX17320_REG_NDELAYCFG        (0x1DCU)
#define MAX17320_REG_NODSCTH          (0x1DDU)
#define MAX17320_REG_NODSCCFG         (0x1DEU)
#define MAX17320_REG_NPROTCFG2        (0x1DFU)

#define MAX17320_REG_NDPLIMIT         (0x1E0U)
#define MAX17320_REG_NSCOCVLIM        (0x1E1U)
#define MAX17320_REG_NAGEFCCFG        (0x1E2U)
#define MAX17320_REG_NDESIGNVOLTAGE   (0x1E3U)
#define MAX17320_REG_NMANFCTRDATE     (0x1E6U)
#define MAX17320_REG_NFIRSTUSED       (0x1E7U)
#define MAX17320_REG_NSERIALNUMBER0   (0x1E8U)
#define MAX17320_REG_NSERIALNUMBER1   (0x1E9U)
#define MAX17320_REG_NSERIALNUMBER2   (0x1EAU)
#define MAX17320_REG_NDEVICENAME0     (0x1EBU)
#define MAX17320_REG_NDEVICENAME1     (0x1ECU)
#define MAX17320_REG_NDEVICENAME2     (0x1EDU)
#define MAX17320_REG_NDEVICENAME3     (0x1EEU)
#define MAX17320_REG_NDEVICENAME4     (0x1EFU)

/* Nonvolatile history page (recalled by the HISTORY RECALL command). */
#define MAX17320_REG_HISTORY_BASE     (0x1F0U)
#define MAX17320_REG_HISTORY_UPDATES  (0x1FDU)

/* -------------------------------------------------------------------------
 * Commands written to the Command register (060h)
 * ------------------------------------------------------------------------- */
#define MAX17320_CMD_COPY_NV_BLOCK       (0xE904U)
#define MAX17320_CMD_NV_RECALL           (0xE001U)
#define MAX17320_CMD_HISTORY_CFG_UPDATES (0xE29BU)
#define MAX17320_CMD_HISTORY_LIFELOG     (0xE29CU)
#define MAX17320_CMD_HISTORY_SECRET      (0xE29DU)
#define MAX17320_CMD_HARDWARE_RESET      (0x000FU)
#define MAX17320_CMD_NV_LOCK_BASE        (0x6A00U)

/* Written to Config2 (0ABh) to restart the fuel gauge firmware. */
#define MAX17320_CMD_FUEL_GAUGE_RESET    (0x8000U)

/* -------------------------------------------------------------------------
 * CommStat register (061h)
 * ------------------------------------------------------------------------- */
#define MAX17320_COMMSTAT_WPGLOBAL     (0x0001U)
#define MAX17320_COMMSTAT_NVBUSY       (0x0002U)
#define MAX17320_COMMSTAT_NVERROR      (0x0004U)
#define MAX17320_COMMSTAT_WP1          (0x0008U)
#define MAX17320_COMMSTAT_WP2          (0x0010U)
#define MAX17320_COMMSTAT_WP3          (0x0020U)
#define MAX17320_COMMSTAT_WP4          (0x0040U)
#define MAX17320_COMMSTAT_WP5          (0x0080U)
#define MAX17320_COMMSTAT_CHGOFF       (0x0100U)
#define MAX17320_COMMSTAT_DISOFF       (0x0200U)

#define MAX17320_COMMSTAT_UNLOCKED     (0x0000U)
#define MAX17320_COMMSTAT_LOCKED       (0x00F9U)

/* -------------------------------------------------------------------------
 * Status register (000h)
 * ------------------------------------------------------------------------- */
#define MAX17320_STATUS_POR            (0x0002U)
#define MAX17320_STATUS_IMN            (0x0004U)
#define MAX17320_STATUS_IMX            (0x0040U)
#define MAX17320_STATUS_DSOCI          (0x0080U)
#define MAX17320_STATUS_VMN            (0x0100U)
#define MAX17320_STATUS_TMN            (0x0200U)
#define MAX17320_STATUS_SMN            (0x0400U)
#define MAX17320_STATUS_VMX            (0x1000U)
#define MAX17320_STATUS_TMX            (0x2000U)
#define MAX17320_STATUS_SMX            (0x4000U)
#define MAX17320_STATUS_PA             (0x8000U)

/* Status2 register (0B0h) */
#define MAX17320_STATUS2_HIB           (0x0002U)

/* -------------------------------------------------------------------------
 * ProtStatus (0D9h) and ProtAlrt (0AFh).  The two registers share a layout;
 * bit 0 is Ship in ProtStatus and LDet in ProtAlrt.
 * ------------------------------------------------------------------------- */
#define MAX17320_PROT_SHIP             (0x0001U) /* ProtStatus only */
#define MAX17320_PROT_LDET             (0x0001U) /* ProtAlrt only   */
#define MAX17320_PROT_RESDFAULT        (0x0002U)
#define MAX17320_PROT_ODCP             (0x0004U)
#define MAX17320_PROT_UVP              (0x0008U)
#define MAX17320_PROT_TOOHOTD          (0x0010U)
#define MAX17320_PROT_DIEHOT           (0x0020U)
#define MAX17320_PROT_PERMFAIL         (0x0040U)
#define MAX17320_PROT_IMBALANCE        (0x0080U)
#define MAX17320_PROT_PREQF            (0x0100U)
#define MAX17320_PROT_QOVFLW           (0x0200U)
#define MAX17320_PROT_OCCP             (0x0400U)
#define MAX17320_PROT_OVP              (0x0800U)
#define MAX17320_PROT_TOOCOLDC         (0x1000U)
#define MAX17320_PROT_FULL             (0x2000U)
#define MAX17320_PROT_TOOHOTC          (0x4000U)
#define MAX17320_PROT_CHGWDT           (0x8000U)

/* -------------------------------------------------------------------------
 * nBattStatus register (1A8h)
 * ------------------------------------------------------------------------- */
#define MAX17320_BATTSTAT_LEAKCURR_MASK (0x00FFU)
#define MAX17320_BATTSTAT_CHKSUMF_UVPF  (0x0100U)
#define MAX17320_BATTSTAT_LDET          (0x0200U)
#define MAX17320_BATTSTAT_FETFO         (0x0400U)
#define MAX17320_BATTSTAT_DFETFS        (0x0800U)
#define MAX17320_BATTSTAT_CFETFS        (0x1000U)
#define MAX17320_BATTSTAT_OTPF          (0x2000U)
#define MAX17320_BATTSTAT_OVPF          (0x4000U)
#define MAX17320_BATTSTAT_PERMFAIL      (0x8000U)

/* -------------------------------------------------------------------------
 * nFaultLog register (1AEh), low byte, when nNVCfg2.enFL = 1
 * ------------------------------------------------------------------------- */
#define MAX17320_FAULTLOG_ODCP         (0x0001U)
#define MAX17320_FAULTLOG_UVP          (0x0002U)
#define MAX17320_FAULTLOG_IMBALANCE    (0x0004U)
#define MAX17320_FAULTLOG_DIEHOT       (0x0008U)
#define MAX17320_FAULTLOG_OCCP         (0x0010U)
#define MAX17320_FAULTLOG_OVP          (0x0020U)
#define MAX17320_FAULTLOG_TOOCOLDC     (0x0040U)
#define MAX17320_FAULTLOG_TOOHOTC      (0x0080U)

/* -------------------------------------------------------------------------
 * FStat register (03Dh)
 * ------------------------------------------------------------------------- */
#define MAX17320_FSTAT_DNR             (0x0001U)
#define MAX17320_FSTAT_RELDT2          (0x0040U)
#define MAX17320_FSTAT_EDET            (0x0100U)
#define MAX17320_FSTAT_RELDT           (0x0200U)

/* -------------------------------------------------------------------------
 * Config register (00Bh)
 * ------------------------------------------------------------------------- */
#define MAX17320_CONFIG_PAEN           (0x0001U)
#define MAX17320_CONFIG_AEN            (0x0004U)
#define MAX17320_CONFIG_FTHRM          (0x0008U)
#define MAX17320_CONFIG_ETHRM          (0x0010U)
#define MAX17320_CONFIG_COMMSH         (0x0040U)
#define MAX17320_CONFIG_SHIP           (0x0080U)
#define MAX17320_CONFIG_DISBLOCKREAD   (0x0200U)
#define MAX17320_CONFIG_PBEN           (0x0400U)
#define MAX17320_CONFIG_DISLDO         (0x0800U)
#define MAX17320_CONFIG_VS             (0x1000U)
#define MAX17320_CONFIG_TS             (0x2000U)
#define MAX17320_CONFIG_SS             (0x4000U)

/* -------------------------------------------------------------------------
 * Config2 register (0ABh)
 * ------------------------------------------------------------------------- */
#define MAX17320_CONFIG2_DRCFG_MASK    (0x0018U)
#define MAX17320_CONFIG2_TALRTEN       (0x0040U)
#define MAX17320_CONFIG2_DSOCEN        (0x0080U)
#define MAX17320_CONFIG2_POWR_MASK     (0x0F00U)
#define MAX17320_CONFIG2_ADCFIFOEN     (0x2000U)
#define MAX17320_CONFIG2_ATRTEN        (0x4000U)
#define MAX17320_CONFIG2_POR_CMD       (0x8000U)

/* -------------------------------------------------------------------------
 * nProtCfg register (1D7h)
 * ------------------------------------------------------------------------- */
#define MAX17320_NPROTCFG_BLOCKDISCEN  (0x0002U)
#define MAX17320_NPROTCFG_FETPFEN      (0x0004U)
#define MAX17320_NPROTCFG_UVRDY        (0x0008U)
#define MAX17320_NPROTCFG_OVRDEN       (0x0010U)
#define MAX17320_NPROTCFG_DEEPSHPEN    (0x0020U)
#define MAX17320_NPROTCFG_PFEN         (0x0040U)
#define MAX17320_NPROTCFG_PREQEN       (0x0100U)
#define MAX17320_NPROTCFG_CMOVRDEN     (0x0400U)
#define MAX17320_NPROTCFG_SCTEST_MASK  (0x1000U)
#define MAX17320_NPROTCFG_CHGWDTEN     (0x8000U)

/* -------------------------------------------------------------------------
 * HProtCfg2 register (0F1h)
 * ------------------------------------------------------------------------- */
#define MAX17320_HPROTCFG2_CHGS        (0x0001U)
#define MAX17320_HPROTCFG2_DISS        (0x0002U)
#define MAX17320_HPROTCFG2_PBEN        (0x0008U)
#define MAX17320_HPROTCFG2_CPCFG_MASK  (0x0030U)
#define MAX17320_HPROTCFG2_COMMOVRD    (0x0100U)
#define MAX17320_HPROTCFG2_AOLDO_MASK  (0xC000U)

/* -------------------------------------------------------------------------
 * nPackCfg register (1B5h)
 * ------------------------------------------------------------------------- */
#define MAX17320_NPACKCFG_NCELLS_MASK   (0x0003U)
#define MAX17320_NPACKCFG_NTHRMS_MASK   (0x001CU)
#define MAX17320_NPACKCFG_NTHRMS_SHIFT  (2U)
#define MAX17320_NPACKCFG_CPCFG_MASK    (0x0300U)
#define MAX17320_NPACKCFG_CPCFG_SHIFT   (8U)
#define MAX17320_NPACKCFG_THTYPE        (0x0800U)
#define MAX17320_NPACKCFG_BTPKEN        (0x4000U)
#define MAX17320_NPACKCFG_AOCFG_MASK    (0xC000U)
#define MAX17320_NPACKCFG_AOCFG_SHIFT   (14U)

/* -------------------------------------------------------------------------
 * Lock register (07Fh) / NV LOCK command bits
 * ------------------------------------------------------------------------- */
#define MAX17320_LOCK1                 (0x0001U) /* pages 1Ah, 1Bh, 1Eh */
#define MAX17320_LOCK2                 (0x0002U) /* pages 01h-04h, 0Bh, 0Dh */
#define MAX17320_LOCK3                 (0x0004U) /* pages 18h, 19h */
#define MAX17320_LOCK4                 (0x0008U) /* page 1Ch */
#define MAX17320_LOCK5                 (0x0010U) /* page 1Dh */

/* -------------------------------------------------------------------------
 * DevName register (021h): revision in D15-D4, device code in D3-D0.
 * ------------------------------------------------------------------------- */
#define MAX17320_DEVNAME_REVISION_MASK (0xFFF0U)
#define MAX17320_DEVNAME_DEVICE_MASK   (0x000FU)
#define MAX17320_DEVNAME_REVISION      (0x4200U) /* 4209h / 420Ah / 420Bh */

/* -------------------------------------------------------------------------
 * Register resolutions (Table 15).  Values marked "/R" must additionally be
 * divided by the pack sense resistance in ohms.
 * ------------------------------------------------------------------------- */
#define MAX17320_LSB_VOLTAGE_V        (78.125e-6F)  /* VCell, Cell1-4, ... */
#define MAX17320_LSB_PACK_VOLTAGE_V   (312.5e-6F)   /* Batt, PCKP          */
#define MAX17320_LSB_CURRENT_VR       (1.5625e-6F)  /* /R -> amps          */
#define MAX17320_LSB_CAPACITY_VHR     (5.0e-6F)     /* /R -> amp-hours     */
#define MAX17320_LSB_POWER_VVR        (8.0e-6F)     /* /R -> watts         */
#define MAX17320_LSB_PERCENT         (1.0F / 256.0F)
#define MAX17320_LSB_TEMPERATURE_C   (1.0F / 256.0F)
#define MAX17320_LSB_RESISTANCE_OHM  (1.0F / 4096.0F)
#define MAX17320_LSB_TIME_S           (5.625F)
#define MAX17320_LSB_TIMERH_S         (11520.0F)    /* 3.2 hours           */
#define MAX17320_LSB_TIMER_S          (0.1758F)
#define MAX17320_LSB_CYCLES           (0.0025F)     /* 25 % per LSB        */
#define MAX17320_LSB_VRIPPLE_V        (1.25e-3F / 128.0F)
#define MAX17320_LSB_MAXMIN_VOLT_V    (20.0e-3F)
#define MAX17320_LSB_MAXMIN_CURR_VR   (400.0e-6F)   /* /R -> amps          */
#define MAX17320_LSB_ALERT_VOLT_V     (20.0e-3F)
#define MAX17320_LSB_ALERT_CURR_VR    (400.0e-6F)   /* /R -> amps          */
#define MAX17320_LSB_LEAKCURR_REP_VR  (1.5625e-6F / 16.0F)
#define MAX17320_LSB_LEAKCURR_STAT_VR (3.125e-6F)
#define MAX17320_LSB_NRSENSE_OHM      (10.0e-6F)

/* -------------------------------------------------------------------------
 * Datasheet timings (ms) used by the reset and nonvolatile sequences.
 * ------------------------------------------------------------------------- */
#define MAX17320_T_POR_MS             (10U)   /* tPOR    */
#define MAX17320_T_RECALL_MS          (5U)    /* tRECALL */
#define MAX17320_T_BLOCK_MAX_MS       (7360U) /* tBLOCK  */
#define MAX17320_T_UPDATE_MAX_MS      (1280U) /* tUPDATE */
#define MAX17320_T_TASK_PERIOD_MS     (352U)  /* tTP     */
#define MAX17320_T_FG_VALID_MS        (480U)  /* outputs valid after reset */

/* Total configuration-memory writes the part supports over its lifetime. */
#define MAX17320_NVM_MAX_UPDATES      (8U)

/* -------------------------------------------------------------------------
 * Error codes.  Every distinct failure condition has its own value so the
 * caller can act on it without the driver emitting any diagnostic output.
 * ------------------------------------------------------------------------- */
typedef enum
{
    MAX17320_OK = 0,

    /* Platform / transport */
    MAX17320_ERR_I2C_READ,          /* max17320_i2c_read reported failure   */
    MAX17320_ERR_I2C_WRITE,         /* max17320_i2c_write reported failure  */
    MAX17320_ERR_NO_PLATFORM_READ,  /* weak I2C read hook not overridden    */
    MAX17320_ERR_NO_PLATFORM_WRITE, /* weak I2C write hook not overridden   */

    /* Argument validation */
    MAX17320_ERR_NULL_PARAM,        /* mandatory output pointer was NULL    */
    MAX17320_ERR_BAD_ADDRESS,       /* register address outside 000h-1FFh   */
    MAX17320_ERR_BAD_LENGTH,        /* zero-length or wrapping burst        */
    MAX17320_ERR_BAD_CELL_INDEX,    /* cell index outside 1-4               */
    MAX17320_ERR_BAD_THERM_INDEX,   /* thermistor index outside 1-4         */
    MAX17320_ERR_BAD_SENSE_RESISTOR,/* sense resistance not > 0             */
    MAX17320_ERR_BAD_PARAM,         /* value outside the encodable range    */
    MAX17320_ERR_NOT_INITIALISED,   /* max17320_init has not succeeded yet  */

    /* Identity */
    MAX17320_ERR_DEVICE_NAME,       /* DevName did not match a MAX17320     */

    /* Write protection */
    MAX17320_ERR_UNLOCK_FAILED,     /* CommStat did not clear write protect */
    MAX17320_ERR_LOCK_FAILED,       /* CommStat did not re-arm write protect*/
    MAX17320_ERR_WRITE_VERIFY,      /* register read-back never matched     */

    /* Nonvolatile memory */
    MAX17320_ERR_NVM_BUSY_TIMEOUT,  /* CommStat.NVBusy never cleared        */
    MAX17320_ERR_NVM_ERROR,         /* CommStat.NVError set after command   */
    MAX17320_ERR_NVM_NO_UPDATES,    /* configuration write budget exhausted */
    MAX17320_ERR_NVM_RECALL_FAILED, /* NV recall command did not complete   */

    /* Reset */
    MAX17320_ERR_POR_TIMEOUT,       /* Config2.POR_CMD never self-cleared   */
    MAX17320_ERR_RESET_TIMEOUT,     /* device did not answer after reset    */
    MAX17320_ERR_FG_NOT_READY,      /* FStat.DNR still set after the wait   */

    /* Device state */
    MAX17320_ERR_PERMANENT_FAIL,    /* nBattStatus.PermFail is latched      */
    MAX17320_ERR_PROTECTION_ALERT,  /* a protection fault is pending        */
    MAX17320_ERR_FET_OVERRIDE_OFF,  /* nProtCfg.CmOvrdEn is not enabled     */
    MAX17320_ERR_MEMORY_LOCKED      /* target page is permanently locked    */
} max17320_error_t;

/* -------------------------------------------------------------------------
 * Decoded register views
 * ------------------------------------------------------------------------- */

/** @brief Status register (000h) alert flags. */
typedef struct
{
    uint16_t raw;
    uint8_t  power_on_reset;  /* POR   */
    uint8_t  current_min;     /* Imn   */
    uint8_t  current_max;     /* Imx   */
    uint8_t  soc_change;      /* dSOCi */
    uint8_t  voltage_min;     /* Vmn   */
    uint8_t  temp_min;        /* Tmn   */
    uint8_t  soc_min;         /* Smn   */
    uint8_t  voltage_max;     /* Vmx   */
    uint8_t  temp_max;        /* Tmx   */
    uint8_t  soc_max;         /* Smx   */
    uint8_t  protection_alert;/* PA    */
} max17320_status_t;

/** @brief ProtStatus register (0D9h) — live protector state machine faults. */
typedef struct
{
    uint16_t raw;
    uint8_t  ship;
    uint8_t  resd_fault;
    uint8_t  overdischarge_current; /* ODCP    */
    uint8_t  undervoltage;          /* UVP     */
    uint8_t  too_hot_discharge;     /* TooHotD */
    uint8_t  die_hot;               /* DieHot  */
    uint8_t  permanent_fail;        /* PermFail*/
    uint8_t  imbalance;             /* Imbal   */
    uint8_t  prequal_timeout;       /* PreqF   */
    uint8_t  capacity_overflow;     /* Qovflw  */
    uint8_t  overcharge_current;    /* OCCP    */
    uint8_t  overvoltage;           /* OVP     */
    uint8_t  too_cold_charge;       /* TooColdC*/
    uint8_t  full;                  /* Full    */
    uint8_t  too_hot_charge;        /* TooHotC */
    uint8_t  charge_watchdog;       /* ChgWDT  */
} max17320_prot_status_t;

/** @brief ProtAlrt register (0AFh) — latched history of protection events. */
typedef struct
{
    uint16_t raw;
    uint8_t  leak_detect;           /* LDet    */
    uint8_t  resd_fault;
    uint8_t  overdischarge_current;
    uint8_t  undervoltage;
    uint8_t  too_hot_discharge;
    uint8_t  die_hot;
    uint8_t  permanent_fail;
    uint8_t  imbalance;
    uint8_t  prequal_timeout;
    uint8_t  capacity_overflow;
    uint8_t  overcharge_current;
    uint8_t  overvoltage;
    uint8_t  too_cold_charge;
    uint8_t  full;
    uint8_t  too_hot_charge;
    uint8_t  charge_watchdog;
} max17320_prot_alert_t;

/** @brief nBattStatus register (1A8h) — permanent battery status. */
typedef struct
{
    uint16_t raw;
    uint8_t  permanent_fail;      /* PermFail */
    uint8_t  overvoltage_fail;    /* OVPF     */
    uint8_t  overtemperature_fail;/* OTPF     */
    uint8_t  chg_fet_short;       /* CFETFs   */
    uint8_t  dis_fet_short;       /* DFETFs   */
    uint8_t  fet_open;            /* FETFo    */
    uint8_t  leak_detect;         /* LDet     */
    uint8_t  checksum_or_uvpf;    /* ChksumF/UVPF */
    float    leak_current_a;      /* LeakCurr, amps */
} max17320_batt_status_t;

/** @brief FStat register (03Dh) — ModelGauge algorithm status. */
typedef struct
{
    uint16_t raw;
    uint8_t  data_not_ready;  /* DNR    */
    uint8_t  empty_detect;    /* EDet   */
    uint8_t  relaxed;         /* RelDt  */
    uint8_t  long_relaxed;    /* RelDt2 */
} max17320_fstat_t;

/** @brief CommStat register (061h). */
typedef struct
{
    uint16_t raw;
    uint8_t  write_protect_global;
    uint8_t  write_protect[5]; /* WP1..WP5 */
    uint8_t  nv_busy;
    uint8_t  nv_error;
    uint8_t  chg_fet_off;
    uint8_t  dis_fet_off;
} max17320_comm_stat_t;

/** @brief FET drive state from HProtCfg2 (0F1h). */
typedef struct
{
    uint16_t raw;
    uint8_t  chg_fet_on;
    uint8_t  dis_fet_on;
    uint8_t  pushbutton_enabled;
    uint8_t  command_override_enabled;
} max17320_fet_state_t;

/** @brief Full analog + fuel-gauge snapshot in engineering units. */
typedef struct
{
    /* Voltage, volts */
    float cell_v[4];        /* Cell1..Cell4         */
    float avg_cell_v[4];    /* AvgCell1..AvgCell4   */
    float vcell_v;          /* lowest cell reading  */
    float avg_vcell_v;
    float batt_v;           /* total pack, inside protector */
    float pckp_v;           /* PACK+ to GND         */
    float vfocv_v;          /* modelled open-circuit voltage */
    float vripple_v;

    /* Current, amps (positive = charging) */
    float current_a;
    float avg_current_a;
    float max_current_a;
    float min_current_a;

    /* Power, watts */
    float power_w;
    float avg_power_w;

    /* Temperature, degrees Celsius */
    float temperature_c;      /* Temp    */
    float avg_temperature_c;  /* AvgTA   */
    float die_temperature_c;  /* DieTemp */
    float max_temperature_c;
    float min_temperature_c;

    /* Capacity, amp-hours */
    float rep_capacity_ah;
    float full_capacity_ah;
    float full_capacity_nom_ah;
    float design_capacity_ah;
    float available_capacity_ah;

    /* State of charge / health, percent */
    float rep_soc_pct;
    float av_soc_pct;
    float mix_soc_pct;
    float vf_soc_pct;
    float age_pct;

    /* Timing */
    float time_to_empty_s;
    float time_to_full_s;
    float cycles;             /* equivalent full cycles */

    /* Charging prescription */
    float charging_voltage_v;
    float charging_current_a;

    /* Cell model */
    float cell_resistance_ohm;
} max17320_measurements_t;

/** @brief Static application configuration supplied at init. */
typedef struct
{
    /**
     * Pack sense resistance in ohms (e.g. 0.005F for a 5 mOhm shunt).  All
     * current, capacity and power conversions are scaled by this value.
     */
    float r_sense_ohms;
    /**
     * Number of series cells populated, 2..4.  Used to bound the per-cell
     * accessors; set to 0 to read the value back from nPackCfg during init.
     */
    uint8_t cell_count;
} max17320_config_t;

/* -------------------------------------------------------------------------
 * Platform hooks — implemented by the application.
 *
 * Each has a weak default in the driver that returns
 * MAX17320_ERR_NO_PLATFORM_READ / MAX17320_ERR_NO_PLATFORM_WRITE, so a build
 * that forgets to provide them fails loudly at run time instead of silently
 * reading zeros.
 * ------------------------------------------------------------------------- */

/**
 * @brief Read @p len bytes starting at memory byte @p mem_addr from the 2-wire
 *        device at 7-bit address @p dev_addr.
 *
 * The transfer is a write of the single memory-address byte followed by a
 * repeated START and @p len read bytes.
 *
 * @param[in]  dev_addr 7-bit slave address, MAX17320_I2C_ADDR_LOW_7B or
 *                      MAX17320_I2C_ADDR_HIGH_7B.
 * @param[in]  mem_addr Memory address byte (internal address & 0xFF).
 * @param[out] rx       Destination buffer.
 * @param[in]  len      Number of bytes to read; always even.
 * @return MAX17320_OK on success, MAX17320_ERR_I2C_READ on any bus error.
 */
max17320_error_t
max17320_i2c_read(const uint8_t dev_addr, const uint8_t mem_addr, uint8_t *rx, const uint16_t len);

/**
 * @brief Write @p len bytes to memory byte @p mem_addr of the 2-wire device at
 *        7-bit address @p dev_addr.
 *
 * @param[in] dev_addr 7-bit slave address.
 * @param[in] mem_addr Memory address byte (internal address & 0xFF).
 * @param[in] tx       Source buffer.
 * @param[in] len      Number of bytes to write; always even.
 * @return MAX17320_OK on success, MAX17320_ERR_I2C_WRITE on any bus error.
 */
max17320_error_t max17320_i2c_write(const uint8_t  dev_addr,
                                    const uint8_t  mem_addr,
                                    const uint8_t *tx,
                                    const uint16_t len);

/**
 * @brief Block for at least @p ms milliseconds.
 *
 * Required by the datasheet reset and nonvolatile sequences (tPOR, tRECALL,
 * tBLOCK, tUPDATE); the driver never uses it for polling convenience alone.
 */
void max17320_delay_ms(const uint32_t ms);

/* -------------------------------------------------------------------------
 * Initialisation and raw register access
 * ------------------------------------------------------------------------- */

/**
 * @brief Latch the application configuration and confirm a MAX17320 answers.
 *
 * Reads DevName and checks the revision field, then stores the sense
 * resistance used by every scaled conversion.  Does not modify device
 * configuration.
 *
 * @param[in] cfg Application configuration; copied internally.
 * @return MAX17320_OK, MAX17320_ERR_NULL_PARAM,
 *         MAX17320_ERR_BAD_SENSE_RESISTOR, MAX17320_ERR_BAD_PARAM,
 *         MAX17320_ERR_DEVICE_NAME or a transport error.
 */
max17320_error_t max17320_init(const max17320_config_t *cfg);

/**
 * @brief Update the sense resistance used for current/capacity/power scaling.
 * @return MAX17320_OK or MAX17320_ERR_BAD_SENSE_RESISTOR.
 */
max17320_error_t max17320_set_sense_resistor(const float ohms);

/**
 * @brief Read the sense resistance the driver is currently scaling with.
 * @return MAX17320_OK or MAX17320_ERR_NULL_PARAM.
 */
max17320_error_t max17320_get_sense_resistor(float *ohms);

/**
 * @brief Read one 16-bit register from either address space.
 * @param[in]  addr  Internal register address, 000h-1FFh.
 * @param[out] value Register contents.
 */
max17320_error_t max17320_read_register(const uint16_t addr, uint16_t *value);

/**
 * @brief Write one 16-bit register.
 *
 * The device arms write protection by default, so a raw write only lands if
 * max17320_unlock_write_protection() has already been called.  The typed
 * setters in this header handle the unlock/lock bracket themselves.
 */
max17320_error_t max17320_write_register(const uint16_t addr, const uint16_t value);

/**
 * @brief Write a register then read it back, retrying up to three times.
 *
 * Recommended by the datasheet for nonvolatile shadow-RAM locations, where a
 * write may be dropped while the device is busy.
 * @return MAX17320_OK or MAX17320_ERR_WRITE_VERIFY.
 */
max17320_error_t max17320_write_verify_register(const uint16_t addr, const uint16_t value);

/**
 * @brief Read @p count consecutive registers starting at @p addr.
 *
 * Uses the device's address auto-increment.  Addresses 100h-17Fh must be read
 * one word at a time and are handled transparently.
 *
 * @return MAX17320_OK, MAX17320_ERR_NULL_PARAM, MAX17320_ERR_BAD_ADDRESS,
 *         MAX17320_ERR_BAD_LENGTH or a transport error.
 */
max17320_error_t
max17320_read_registers(const uint16_t addr, uint16_t *values, const uint16_t count);

/**
 * @brief Read-modify-write a register, replacing only the bits in @p mask.
 */
max17320_error_t
max17320_update_register(const uint16_t addr, const uint16_t mask, const uint16_t value);

/* -------------------------------------------------------------------------
 * Write protection
 * ------------------------------------------------------------------------- */

/**
 * @brief Clear the global and per-page write protection (CommStat = 0000h
 *        written twice) and verify it took effect.
 * @return MAX17320_OK or MAX17320_ERR_UNLOCK_FAILED.
 */
max17320_error_t max17320_unlock_write_protection(void);

/**
 * @brief Re-arm write protection (CommStat = 00F9h written twice).
 * @return MAX17320_OK or MAX17320_ERR_LOCK_FAILED.
 */
max17320_error_t max17320_lock_write_protection(void);

/**
 * @brief Read and decode the CommStat register.
 */
max17320_error_t max17320_get_comm_status(max17320_comm_stat_t *status);

/**
 * @brief Read the permanent Lock register (07Fh) bitmap.
 *
 * Test the result against MAX17320_LOCK1..MAX17320_LOCK5.
 */
max17320_error_t max17320_get_lock_state(uint16_t *locks);

/* -------------------------------------------------------------------------
 * Commands, reset and nonvolatile memory
 * ------------------------------------------------------------------------- */

/**
 * @brief Write a raw command to the Command register (060h).
 *
 * Write protection must already be cleared.
 */
max17320_error_t max17320_send_command(const uint16_t command);

/**
 * @brief Full reset: hardware reset command, POR wait, fuel-gauge restart.
 *
 * Follows the datasheet FULL RESET sequence including the tPOR wait and the
 * Config2.POR_CMD poll.  Leaves write protection re-armed.
 */
max17320_error_t max17320_full_reset(void);

/**
 * @brief Fuel-gauge reset: restart the algorithm without recalling
 *        nonvolatile memory and without disturbing the protection FETs.
 */
max17320_error_t max17320_fuel_gauge_reset(void);

/**
 * @brief Recall the whole nonvolatile block into shadow RAM (NV RECALL).
 */
max17320_error_t max17320_nv_recall(void);

/**
 * @brief Copy shadow RAM 180h-1EFh into nonvolatile memory (COPY NV BLOCK).
 *
 * Consumes one of the part's limited configuration writes, waits tBLOCK,
 * checks CommStat.NVError, then performs the full reset the datasheet
 * requires for the new values to take effect.
 *
 * @param[in] check_budget Non-zero to refuse the copy when
 *                         max17320_get_remaining_nvm_updates() reports none
 *                         left.
 * @return MAX17320_OK, MAX17320_ERR_NVM_NO_UPDATES, MAX17320_ERR_NVM_ERROR,
 *         MAX17320_ERR_NVM_BUSY_TIMEOUT or a transport error.
 */
max17320_error_t max17320_nv_block_copy(const uint8_t check_budget);

/**
 * @brief Report how many configuration-memory writes remain (0..7).
 *
 * Issues the history recall command for the configuration update flags and
 * decodes address 1FDh.  Manufacturing test consumes the first write.
 */
max17320_error_t max17320_get_remaining_nvm_updates(uint8_t *remaining);

/**
 * @brief Permanently lock the memory blocks selected in @p lock_mask.
 *
 * Irreversible.  @p lock_mask is a combination of MAX17320_LOCK1..LOCK5.
 */
max17320_error_t max17320_lock_memory_blocks(const uint16_t lock_mask);

/* -------------------------------------------------------------------------
 * Status
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_status(max17320_status_t *status);
max17320_error_t max17320_get_protection_status(max17320_prot_status_t *status);
max17320_error_t max17320_get_protection_alert(max17320_prot_alert_t *alert);
max17320_error_t max17320_get_battery_status(max17320_batt_status_t *status);
max17320_error_t max17320_get_fuel_gauge_status(max17320_fstat_t *status);

/**
 * @brief Read the latched fault log (nFaultLog low byte).
 *
 * Only meaningful when nNVCfg2.enFL is set.  Test against the
 * MAX17320_FAULTLOG_* masks.
 */
max17320_error_t max17320_get_fault_log(uint16_t *faults);

/**
 * @brief Clear the Status.POR flag after servicing a power-on reset.
 */
max17320_error_t max17320_clear_por(void);

/**
 * @brief Clear a pending protection alert.
 *
 * Writes ProtAlrt to 0000h first, as required, then clears Status.PA.
 */
max17320_error_t max17320_clear_protection_alert(void);

/**
 * @brief Report whether the pack is safe to use.
 *
 * @return MAX17320_OK when no fault is pending,
 *         MAX17320_ERR_PERMANENT_FAIL when nBattStatus.PermFail is latched
 *         (unrecoverable), MAX17320_ERR_PROTECTION_ALERT when a recoverable
 *         protection event is pending, or a transport error.
 */
max17320_error_t max17320_check_health(void);

/**
 * @brief Report whether the device is in hibernate mode (Status2.Hib).
 */
max17320_error_t max17320_is_hibernating(uint8_t *hibernating);

/**
 * @brief Wait until the ModelGauge outputs are valid (FStat.DNR clear).
 *
 * @param[in] timeout_ms Upper bound on the wait.
 * @return MAX17320_OK or MAX17320_ERR_FG_NOT_READY.
 */
max17320_error_t max17320_wait_data_ready(const uint32_t timeout_ms);

/* -------------------------------------------------------------------------
 * Identity
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_device_name(uint16_t *dev_name);

/**
 * @brief Read the unique 64-bit ROM ID from nROMID0-3.
 */
max17320_error_t max17320_get_rom_id(uint64_t *rom_id);

/* -------------------------------------------------------------------------
 * Voltage measurements (volts)
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_cell_voltage(const uint8_t cell, float *volts);
max17320_error_t max17320_get_avg_cell_voltage(const uint8_t cell, float *volts);
max17320_error_t max17320_get_vcell(float *volts);
max17320_error_t max17320_get_avg_vcell(float *volts);
max17320_error_t max17320_get_pack_voltage(float *volts);
max17320_error_t max17320_get_pckp_voltage(float *volts);
max17320_error_t max17320_get_open_circuit_voltage(float *volts);
max17320_error_t max17320_get_ripple_voltage(float *volts);

/**
 * @brief Read the max/min cell voltages logged since the last reset.
 */
max17320_error_t max17320_get_max_min_voltage(float *max_volts, float *min_volts);

/**
 * @brief Reset the MaxMinVolt log to its power-up value (00FFh).
 */
max17320_error_t max17320_reset_max_min_voltage(void);

/* -------------------------------------------------------------------------
 * Current measurements (amps, positive while charging)
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_current(float *amps);
max17320_error_t max17320_get_avg_current(float *amps);
max17320_error_t max17320_get_max_min_current(float *max_amps, float *min_amps);
max17320_error_t max17320_reset_max_min_current(void);

/* -------------------------------------------------------------------------
 * Temperature measurements (degrees Celsius)
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_temperature(float *celsius);
max17320_error_t max17320_get_avg_temperature(float *celsius);
max17320_error_t max17320_get_die_temperature(float *celsius);
max17320_error_t max17320_get_avg_die_temperature(float *celsius);
max17320_error_t max17320_get_thermistor_temperature(const uint8_t channel, float *celsius);
max17320_error_t max17320_get_avg_thermistor_temperature(const uint8_t channel, float *celsius);
max17320_error_t max17320_get_max_min_temperature(float *max_celsius, float *min_celsius);
max17320_error_t max17320_reset_max_min_temperature(void);

/* -------------------------------------------------------------------------
 * Power (watts)
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_power(float *watts);
max17320_error_t max17320_get_avg_power(float *watts);

/* -------------------------------------------------------------------------
 * Capacity (amp-hours), state of charge (percent) and timing (seconds)
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_reported_capacity(float *amp_hours);
max17320_error_t max17320_get_full_capacity(float *amp_hours);
max17320_error_t max17320_get_full_capacity_nominal(float *amp_hours);
max17320_error_t max17320_get_design_capacity(float *amp_hours);
max17320_error_t max17320_get_available_capacity(float *amp_hours);
max17320_error_t max17320_get_mix_capacity(float *amp_hours);
max17320_error_t max17320_get_residual_capacity(float *amp_hours);
max17320_error_t max17320_get_coulomb_count(float *amp_hours);

max17320_error_t max17320_get_reported_soc(float *percent);
max17320_error_t max17320_get_available_soc(float *percent);
max17320_error_t max17320_get_mix_soc(float *percent);
max17320_error_t max17320_get_vf_soc(float *percent);
max17320_error_t max17320_get_age(float *percent);

max17320_error_t max17320_get_time_to_empty(float *seconds);
max17320_error_t max17320_get_time_to_full(float *seconds);
max17320_error_t max17320_get_cycles(float *cycles);
max17320_error_t max17320_get_age_timer(float *seconds);
max17320_error_t max17320_get_cell_resistance(float *ohms);

/**
 * @brief Read the reported internal self-discharge leakage current, amps.
 *
 * Valid only when the ISD feature is enabled through nProtCfg2.CEEn.
 */
max17320_error_t max17320_get_leakage_current(float *amps);

/* -------------------------------------------------------------------------
 * Charging prescription
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_charging_voltage(float *volts);
max17320_error_t max17320_get_charging_current(float *amps);

/* -------------------------------------------------------------------------
 * At-rate estimation
 * ------------------------------------------------------------------------- */

/**
 * @brief Program a hypothetical load current and read back the estimates.
 *
 * Writes AtRate (negative for a discharge load), waits two task periods as
 * required, then reads AtTTE, AtAvSOC and AtAvCap.  Any output pointer may be
 * NULL.
 *
 * @param[in]  load_amps        Hypothetical load, amps; negative discharges.
 * @param[out] time_to_empty_s  Estimated time to empty, seconds.
 * @param[out] soc_percent      Estimated state of charge, percent.
 * @param[out] capacity_ah      Estimated available capacity, amp-hours.
 */
max17320_error_t max17320_estimate_at_rate(const float load_amps,
                                           float      *time_to_empty_s,
                                           float      *soc_percent,
                                           float      *capacity_ah);

/* -------------------------------------------------------------------------
 * Alert thresholds
 * ------------------------------------------------------------------------- */

/**
 * @brief Set the VAlrtTh cell-voltage alert window, volts (20 mV resolution).
 *
 * Pass 0.0F and 5.1F to disable.
 */
max17320_error_t max17320_set_voltage_alert(const float min_volts, const float max_volts);

/**
 * @brief Set the TAlrtTh temperature alert window, degrees Celsius.
 */
max17320_error_t max17320_set_temperature_alert(const float min_celsius, const float max_celsius);

/**
 * @brief Set the SAlrtTh state-of-charge alert window, percent.
 */
max17320_error_t max17320_set_soc_alert(const float min_percent, const float max_percent);

/**
 * @brief Set the IAlrtTh current alert window, amps (400 uV/Rsense resolution).
 */
max17320_error_t max17320_set_current_alert(const float min_amps, const float max_amps);

/**
 * @brief Enable or disable the ALRT pin and the protection-alert source.
 *
 * @param[in] alerts_enabled     Config.Aen — fuel-gauge threshold alerts.
 * @param[in] protection_enabled Config.PAen — protection faults raise Status.PA.
 */
max17320_error_t max17320_set_alerts_enabled(const uint8_t alerts_enabled,
                                             const uint8_t protection_enabled);

/* -------------------------------------------------------------------------
 * FET and power-mode control
 * ------------------------------------------------------------------------- */

/**
 * @brief Read the live CHG/DIS FET drive state from HProtCfg2.
 */
max17320_error_t max17320_get_fet_state(max17320_fet_state_t *state);

/**
 * @brief Force either protection FET off, or release both back to the
 *        protector state machine.
 *
 * Requires nProtCfg.CmOvrdEn to be set in nonvolatile configuration;
 * otherwise MAX17320_ERR_FET_OVERRIDE_OFF is returned and nothing is written.
 *
 * @param[in] chg_off Non-zero to hold the charge FET off.
 * @param[in] dis_off Non-zero to hold the discharge FET off.
 */
max17320_error_t max17320_set_fet_override(const uint8_t chg_off, const uint8_t dis_off);

/**
 * @brief Enter ship or deepship mode (Config.SHIP).
 *
 * Which of the two is entered depends on the nonvolatile nProtCfg.DeepShpEn
 * bit.  The FETs open within 1.4 s and the part shuts down after the
 * nDelayCfg.UVPTimer shutdown timeout.
 */
max17320_error_t max17320_enter_ship_mode(void);

/**
 * @brief Enable or disable hibernate mode (nHibCfg.EnHib in shadow RAM).
 *
 * Affects the shadow-RAM copy only; call max17320_nv_block_copy() to make it
 * survive a reset.
 */
max17320_error_t max17320_set_hibernate_enabled(const uint8_t enabled);

/* -------------------------------------------------------------------------
 * Pack configuration helpers
 * ------------------------------------------------------------------------- */

/**
 * @brief Read the configured number of series cells (2..4) from nPackCfg.
 */
max17320_error_t max17320_get_cell_count(uint8_t *cells);

/**
 * @brief Read the number of enabled thermistor channels (0..4) from nPackCfg.
 */
max17320_error_t max17320_get_thermistor_count(uint8_t *thermistors);

/**
 * @brief Write the design capacity, amp-hours, into shadow RAM.
 *
 * Updates both nDesignCap and the live DesignCap register.
 */
max17320_error_t max17320_set_design_capacity(const float amp_hours);

/**
 * @brief Store the nominal sense resistance in nRSense for host software.
 *
 * Does not change the driver's own scaling; use
 * max17320_set_sense_resistor() for that.
 */
max17320_error_t max17320_set_nv_sense_resistor(const float ohms);

/* -------------------------------------------------------------------------
 * Aggregate read
 * ------------------------------------------------------------------------- */

/**
 * @brief Populate a full measurement snapshot in engineering units.
 *
 * Per-cell entries beyond the configured cell count are left at 0.0F.
 */
max17320_error_t max17320_read_measurements(max17320_measurements_t *out);

#endif /* MAX17320_H */
