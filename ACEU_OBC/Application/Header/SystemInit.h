#ifndef HEADER_SYSTEMINIT_H_
#define HEADER_SYSTEMINIT_H_

/***********************************************************************************************************************
 * Include Files
 **********************************************************************************************************************/
#include <stdint.h>                  /* uint8_t / uint16_t / uint32_t */
#include <string.h>                  /* memcpy for ICD frame copy */
#include <stdio.h>                   /* sprintf debug (optional) */
#include <stdbool.h>                 /* bool received */
#include "math.h"                    /* PID float math */
#include "device.h"                  /* DEVICE_SYSCLK_FREQ, DEVICE_DELAY_US */
#include "board.h"                   /* SysConfig pins, RS422_BASE, LED */
#include "F021_F2838x_C28x.h"        /* Flash API kept; firmware/flash ICD not updated */
#include "AdcProcess.h"              /* adc_channels, ReadIntADC */

/***********************************************************************************************************************
 * Macros and Definitions  (ACEU_ICD_V1_AMS.pdf, ACEU POV only)
 * Frame: STX EB90 | MSGID | MSGLEN | MSGTXT[n] | CHKSUM (XOR Method1)
 * Firmware/flash MSGID 0xA5 / 0xB5 / 0xC5 are not implemented here.
 **********************************************************************************************************************/
#define PWM_PERIOD                  (3000u) /* ePWM TBPRD. Duty = (PWM_PERIOD - CMPA) / PWM_PERIOD. */
#define INTEGRAL_MAX                (1000u) /* PID I-term clamp (legacy). Live clamp is I_t +/-700. */

/* NEW ICD STX and live MSGID (Table 3 / Table 4). Do not add flash 0xA5/0xB5/0xC5. */
#define ICD_STX0                    (0xEBu) /* STX byte 0, all messages */
#define ICD_STX1                    (0x90u) /* STX byte 1, all messages */
#define ICD_FIN_DEFLECT_CMD         (0x90u) /* GUI/FCU -> ACEU Fin Deflection, MSGLEN 12 */
#define ICD_FIN_STATUS_RSP          (0x9Au) /* ACEU -> GUI/FCU status, MSGLEN 74 (real-time) */
#define ICD_CFG_READ_CMD            (0xB0u) /* GUI -> ACEU Read Configuration, MSGLEN 0 */
#define ICD_CFG_READ_RSP            (0x80u) /* ACEU -> GUI config, MSGLEN 102 */
#define ICD_CFG_WRITE_CMD           (0xB1u) /* GUI -> ACEU Write Configuration, MSGLEN 102 */
#define ICD_CFG_WRITE_RSP           (0x81u) /* ACEU -> GUI write status, MSGLEN 1 (RAM only) */
#define ICD_FIN_POS_CMD             (0xB2u) /* GUI -> ACEU Read Fin Positions, MSGLEN 0 */
#define ICD_FIN_POS_RSP             (0x82u) /* ACEU -> GUI 4x2-byte positions, body 8 */
#define ICD_HOME_ALL_CMD            (0x91u) /* GUI -> ACEU Home All, MSGLEN 0 */
#define ICD_HOME_ALL_RSP            (0x61u) /* ACEU -> GUI 4x2-byte positions, body 8 */
#define ICD_FUNC_TEST_CMD           (0x93u) /* GUI -> ACEU Functional Test, body 12 */
#define ICD_FUNC_TEST_RSP           (0x63u) /* ACEU -> GUI test telemetry, MSGLEN 46 */
#define ICD_ADC_ACQ_CMD             (0xAAu) /* GUI -> ACEU ADC Data Acquisition, MSGLEN 0 */
#define ICD_ADC_ACQ_RSP             (0xBBu) /* ACEU -> GUI ADC snapshot, MSGLEN 32 */

/* NEW ICD payload sizes (field sums; Table 4 10/14 vs 8/12 uses field sums). */
#define ICD_FIN_DEFLECT_LEN         (12u)   /* ICD 4 slots; ACEU uses Fin1 only */
#define ICD_STATUS_LEN              (74u)   /* 0x9A MSGTXT */
#define ICD_CFG_LEN                 (102u)  /* 0xB0/0xB1 MSGTXT */
#define ICD_FIN_POS_LEN             (8u)    /* 4 x uint16 positions */
#define ICD_FUNC_TEST_CMD_LEN       (12u)   /* fin mask, type, 4x2 defl, cycles, freq */
#define ICD_FUNC_TEST_RSP_LEN       (46u)   /* 0x63 MSGTXT */
#define ICD_ADC_LEN                 (32u)   /* 0xBB MSGTXT */
#define ICD_HOME_COUNT              (0x800000UL) /* Fin1 zero / home count */
#define ICD_NO_MOVE_COUNT           (0x000000UL) /* 0x90: Fin1 is not moved */

/* This ACEU is Channel 1 only (Fin1 / Motor1 / EPWM1).
 * ICD MSGLEN still has Fin2-4 slots so the GUI/FCU frame stays valid.
 * Those slots are padded on TX and ignored on RX. Never applied to a motor. */
#define ICD_CH1_ONLY                (1u)    /* Board uses Channel 1 */
#define ICD_FIN1_MSK                (0x01u) /* 0x93 bit 0 = Fin1 / Channel 1 */

/* System Status Flags (board POST). NEW ICD has no 0xE2 Health/POST packet. */
#define Hall_Sensor                 (1u)    /* Local POST bit: Hall Pass */
#define ALU                         (1u)    /* Local POST bit: ALU Pass (overlaps Hall) */
#define ADC_M5V                     (2u)    /* Local POST bit: -5 V */
#define ADC_5V                      (4u)    /* +5 V ADC rail */
#define ADC_1V2                     (8u)    /* +1.2 V ADC rail */
#define ADC_3V3                     (16u)   /* +3.3 V rail */
#define ADC_P15V                    (32u)   /* Board +15 V */
#define ADC_M15V                    (64u)   /* Local POST bit: -15 V */
#define ADC_3V                      (128u)  /* Local POST bit: +3 V */

/* Boolean Logic and Status */
#define SET                         (1u)    /* GPIO high / LED on */
#define RESET                       (0u)    /* GPIO low / LED off */
#define Success                     (1u)    /* POST pass */
#define Failure                     (0u)    /* POST fail; main sends 0x9A status */

/* Utility Macros */
#define TICK_TIMER(timer, limit)    if(++timer >= limit) timer = 0 /* Unused wrap helper */

/***********************************************************************************************************************
 * External Global Variables
 **********************************************************************************************************************/
extern uint8_t  rxData[8];           /* CAN RX object 2 (8-byte CAN frame, not an ICD packet). */
extern bool     received;            /* CAN message received flag */
extern char     G_ch_OutBuffer[512]; /* Buffer for string formatting and printing */
extern uint32_t AdcRawData[14];      /* 16-bit ADC samples, indexed by adc_channels */
extern uint16_t HEALTH;              /* Local fail flag; NEW ICD has no Health 0x00/0x01 byte */
extern uint16_t Temp;                /* I2C TMP100 deg C (0x63 Temperature1) */

/***********************************************************************************************************************
 * Function Prototypes
 **********************************************************************************************************************/

/* System and Hardware Initialisation */
uint8_t SystemInit(void);            /* Clocks, pins, ADC, PWM, IRQ. Returns POST Success/Failure. */
void ADC_Init(void);                 /* IntAdcInit + InitAdcSocs (rails / pos / Imot). */
void PWM_Init(void);                 /* ePWM1 only (Channel 1 / Motor1). */
void InitEPWM(uint32_t n_base);      /* One ePWM instance: period PWM_PERIOD, up-count. */
void Interrupt_Init(void);           /* SCI-A RX, CAN-A, Timer0 2 ms (0x90/0x9A class). */
void Timer0Init(uint32_t n_usecPeriod); /* CPU Timer0 period in microseconds. */

/* Self-Test and Diagnostics (board POST; NEW ICD has no dedicated POST packet) */
uint8_t Power_ON_SelfTest(void);     /* Hall, rails, ALU; then send 0x9A status. */
uint8_t HALL_TEST(void);             /* Hall 000/111 = Fail. */
uint8_t Voltage_checks(void);        /* Fail if rails stuck at 0 or 65535. */
uint8_t ALU_Test(void);              /* CPU math. */

/* Communication: NEW ICD STX EB90, MSGID, MSGLEN, MSGTXT, XOR CHKSUM */
void     RSS_422_New_Data(void);     /* Copy a finished EB90 frame into OBC_DATA_ACT[]. */
void     Digital_Data_Validation(void); /* Dispatch NEW ICD MSGID at byte [2]. */
uint16_t CRC16_CCITT(const uint8_t *data, uint16_t length); /* Legacy CCITT; not used on NEW wire. */
uint16_t CRC16_MODBUS(const uint8_t *data, uint16_t length); /* ICD Method2 poly 0xA001, init 0xFFFF. */
uint8_t  IcdXorChecksum(const uint8_t *data, uint16_t length); /* ICD Method1 XOR of STX..MSGTXT. */
void     IcdSendFrame(uint8_t msgid, const uint8_t *msgtxt, uint8_t msglen); /* Build EB90 + XOR + TX. */

uint16_t Checksum_Calculation(uint8_t *Data, uint8_t NoofBytes); /* XOR checksum helper. */
void     PrintString(uint8_t *p_inBuffer, uint8_t Data);         /* SCI-A TX of one ICD reply. */

/* NEW ICD ACEU -> GUI replies (firmware/flash not updated) */
void STEP_SINE_TRAPEZOIDAL_START_Feedback(uint8_t Command); /* 0x63 functional-test stream. */
void STEP_SINE_TRAPEZOIDAL_STOP_Feedback(uint8_t Command);  /* Coast/brake only; no NEW stop MSGID. */
void ACEU_CONFIG_READ(uint8_t Command);                     /* 0x80, 102-byte config. */
void ACEU_CONFIG_WRITE(void);                               /* 0xB1 RAM apply; 0x81 status 1. */
void HEALTH_PACKET_Feedback(uint8_t Command);               /* 0x9A 74-byte fin status (replaces 0xE2). */
void HOME_ALL_Feedback(uint8_t Command);                    /* 0x61 8-byte positions. */
void FIN_POS_READ_Feedback(void);                           /* 0x82 8-byte positions. */
void ADC_ACQ_Feedback(void);                                /* 0xBB 32-byte ADC snapshot. */

/* Motor Control and Sensors */
void     Motor_Control(void);        /* Channel 1 PID: analog (Mode 0) or ICD (Mode 1). */
uint16_t I2c_temp(void);             /* TMP100 0x48, 0.0625 deg C/LSB. */

/* Interrupt Service Handlers (ISR) */
__interrupt void SCIARxISR(void);    /* Parse STX EB90 MSGID MSGLEN MSGTXT CHKSUM on SCI-A. */
__interrupt void canISR(void);       /* CAN-A object 2 receive. */
__interrupt void ControlISR(void);   /* Timer0 ~2 ms; Flag2 for 0x9A, Flag5 for 0x63. */
__interrupt void gpioInterruptHandler(void); /* Hall / spare GPIO; ACK only */

/* Entry Point */
extern void _c_int00(void);          /* C runtime reset vector */

#endif /* HEADER_SYSTEMINIT_H_ */
