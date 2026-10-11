
/***********************************************************************************************************************
 * Include Files
 **********************************************************************************************************************/
#include "SystemInit.h"             /* Types, GPIO, ADC, UART, PWM, NEW ICD prototypes */

/*****************************************************************************
 * File    : SystemInit.c  (ACEU_OBC  -  F2838x)
 * ICD     : ACEU_ICD_V1_AMS.pdf  -  ACEU POV only (not old $ 0xAA CRC16 #)
 * Purpose : Board bring-up, POST, PWM/ADC/timer, NEW ICD UART (STX EB90),
 *           CAN RX, Hall GPIO ISR. Channel 1 only (Fin1 / Motor1 / EPWM1).
 * Frame   : EB90 | MSGID | MSGLEN | MSGTXT | XOR CHKSUM (Method1)
 *           Method2 CRC16-MODBUS is implemented but not used on the wire yet.
 * Firmware/flash MSGID 0xA5 / 0xB5 / 0xC5 are not updated.
 *****************************************************************************/

/* NEW ICD UART parser (MSGID is 3rd byte after STX) */
typedef enum
{
    RX_WAIT_STX0,                       /* Wait for STX0 0xEB */
    RX_WAIT_STX1,                       /* Wait for STX1 0x90 */
    RX_WAIT_MSGID,                      /* Message Type */
    RX_WAIT_MSGLEN,                     /* Length of MSGTXT (0 = no payload) */
    RX_WAIT_MSGTXT,                     /* Payload bytes */
    RX_WAIT_CHKSUM                      /* Method1 XOR of STX..MSGTXT */
} RX_STATE;

RX_STATE rx_state = RX_WAIT_STX0;       /* Parser starts looking for 0xEB */

/* Communication and Buffer Variables */
uint8_t  rx_index       = 0u;           /* Write index into Rx_Char[] */
uint8_t  payload_len    = 0u;           /* Payload size from length byte */
uint8_t  payload_cnt    = 0u;           /* How many payload bytes stored */
uint8_t  Uart_NewData   = 0u;           /* 1 = full EB90 ... CHKSUM packet in Rx_Char[] */
uint8_t  rxData[8]      = {0u};         /* CAN object-2 data (8 bytes max) */
uint16_t rx_crc         = 0u;           /* CRC received from GUI */
uint16_t calc_crc       = 0u;           /* CRC we compute over header+payload */

/* Timing Flags (Timer0 = 2 ms). NEW ICD 0x9A ~2.5 ms class; 0x63 streamed on Flag5. */
uint16_t Timmer2        = 0u;           /* Tick count for Flag2 (2 ms) */
uint16_t Timmer5        = 0u;           /* Tick count for Flag5 (~6 ms) */
uint16_t Timmer40       = 0u;           /* Tick count for Flag40 (40 ms) */
uint16_t Timmer50       = 0u;           /* Tick count for Flag50 (50 ms) */
uint16_t Timmer100      = 0u;           /* Tick count for Flag100 (100 ms) */
uint16_t Timmer500      = 0u;           /* Tick count for Flag500 (500 ms) */
uint16_t Timmer1000     = 0u;           /* Tick count for Flag1000 (1 s) */
uint16_t Timmer2000     = 0u;           /* Tick count for Flag2000 (2 s) */
uint16_t Timmer5000     = 0u;           /* Tick count for Flag5000 (5 s) */

uint16_t Flag2          = 0u;           /* 2 ms - NEW ICD 0x90 / 0x9A class */
uint16_t Flag5          = 0u;           /* ~6 ms - NEW ICD 0x63 functional-test stream */
uint16_t Flag40         = 0u;           /* 40 ms spare tick */
uint16_t Flag50         = 0u;           /* 50 ms spare tick */
uint16_t Flag100        = 0u;           /* 100 ms spare tick */
uint16_t Flag500        = 0u;           /* 500 ms spare tick */
uint16_t Flag1000       = 0u;           /* 1 s spare tick */
uint16_t Flag2000       = 0u;           /* 2 s spare tick */
uint16_t Flag5000       = 0u;           /* 5 s spare tick */

/* System Status and Control Variables */
uint16_t Temp           = 0u;           /* TMP100 deg C; NEW ICD 0x63 Temperature1 */
uint64_t timer_count    = 0u;           /* Unused free-running tick */
int32_t  GET_ERRor      = 0u;           /* ePWM CMPA read-back (debug) */
uint16_t Mode           = 0u;           /* 0 = analog A0, 1 = NEW ICD digital (0x90 / 0x93) */
uint8_t  j              = 0u;           /* CRC bit-loop index (shared) */
uint16_t ACT_POST       = 0xFFu;    /* Local POST mask; NEW ICD has no DSP POST word */
uint16_t HEALTH         = 0x00u;    /* Local fail flag; NEW ICD has no Health 0x00/0x01 */
uint16_t icd_watch      = 0u;       /* 0x9A Watch Variable LSB counter */
uint16_t icd_status     = 0u;       /* 0x9A Status bits 0-11 */
uint8_t  aceu_cfg[ICD_CFG_LEN] = {0u}; /* 102-byte NEW ICD configuration (RAM) */

/* Data Buffers */
uint16_t Feedback_calc_crc      = 0u;   /* CRC16 of telemetry before send */
uint8_t  FBK_Buffer[250 ]        = {0u};   /* Outgoing RS-422 NEW ICD reply */
uint8_t  Rx_Char[250]           = {0u};   /* Incoming frame (MSGID at [2]) */
uint8_t  OBC_DATA_ACT[250]      = {0u};   /* Last accepted frame for Digital_Data_Validation */
uint32_t Analog_Command1        = 500u; /* Fin1 setpoint scaled to 0..1000 (mid = home) */
uint32_t Analog_Command2        = 500u; /* Unused (no Channel 2 on this board) */

bool received = false;                  /* 1 when canISR stored an 8-byte CAN frame */

/* PID  -  NEW ICD 0xB0/0xB1 Kp, Kd, Ki (that order) */
float64_t kp              = 75.0;       /* ICD cfg Kp Position */
float64_t kd              = 0.0;        /* ICD cfg Kd Position */
float64_t ki              = 0.0;        /* ICD cfg Ki Position */
float64_t ki_limit        = 1000.0;     /* ICD cfg Ki Limit Position */

/* PID State Variables */
long double set_value       = 0.0;      /* Target 0..1000 (analog or ICD amp) */
long double feedback_value  = 0.0;      /* Fin position scaled 0..1000 */
long double error_value     = 0.0;      /* set_value - feedback_value */
long double pre_error       = 0.0;      /* Previous error for D term */
long double integral        = 0.0;      /* Legacy I term (unused in live PID) */
long double derivative      = 0.0;      /* Legacy D term (unused in live PID) */
long double output          = 0.0;      /* PWM counts after PID */
long double val             = 0.0;      /* ICD float amp from 0xE4/E6/E8/EA */
long double scaled          = 500.0;    /* Mid-scale spare (0 deg) */


long double Ts              = 0.01;     /* PID sample time, seconds */
long double Pterm           = 0.0;      /* kp * error */
long double Iterm           = 0.0;      /* ki * I_t */
long double Dterm           = 0.0;      /* kd * D_t */
long double I_t             = 0.0;      /* Integrator accumulator */
long double D_t             = 0.0;      /* Derivative (error - pre_error) / Ts */

/* Union for Float-to-Binary Conversion (ICD 0xE0/0xE1 Kp/Ki/Kd bytes) */
union FloatConv {
    uint32_t Conversion_Value;          /* Big-endian 32-bit from GUI */
    float Digital_value;                /* Same bits as float */
};

union FloatConv_read {
    uint32_t Conversion_Value_read;     /* Big-endian 32-bit to GUI */
    float Digital_value_read;           /* Same bits as float */
};

union FloatConv converter;              /* 0xE1 write overlay */

union FloatConv_read converter_read;    /* 0xE0 read overlay */

float var_adc = 0;                      /* Unused ADC scratch */
uint16_t volt_scaled ;                  /* Unused volt scale */

/***********************************************************************************************************************
 * Function Definitions
 **********************************************************************************************************************/

/*****************************************************************************
 * Function : SystemInit
 * Purpose  : First function from main(). Turns on the whole board:
 *            clocks, interrupts, SysConfig pins (Board_init), ADC, PWM.
 *            Power_ON_SelfTest() is present but currently commented out.
 * Input    : none
 * Output   : Success (or Failure if POST is enabled and fails)
 *****************************************************************************/
uint8_t SystemInit(void)
{
    uint8_t Post_status = Success;      /* Assume OK until POST (currently skipped) */

    Device_init();               /* Initialize device clock, watchdog, and core peripherals */

    Interrupt_initModule();      /* Initialize PIE and clear PIE registers; disables CPU interrupts */
    Interrupt_initVectorTable(); /* Populate PIE vector table with default handlers */

    Board_init();                /* Configure GPIOs and Board-level hardware */
    ADC_Init();                  /* Configure ADC modules and SOCs */
    PWM_Init();                  /* Configure ePWM modules for motor control */
    Interrupt_Init();            /* Enable specific peripheral interrupts */

    //    Post_status = Power_ON_SelfTest(); /* Run diagnostic routines */

    return Post_status;
}

/*****************************************************************************
 * Function : ADC_Init
 * Purpose  : ICD 16-bit ADCA/ADCB: supplies, fin position, motor current.
 * Input    : none
 * Output   : converters and SOC map ready
 *****************************************************************************/
void ADC_Init(void)
{
    IntAdcInit();                /* Hardware-specific ADC setup */
    InitAdcSocs();               /* Map ADC channels to specific triggers */
}

/*****************************************************************************
 * Function : PWM_Init
 * Purpose  : ePWM1 for Channel 1 motor only (Fin1 / Motor1).
 *            EPWM2 is not started (no Channel 2/3/4 on this board).
 * Input    : none
 * Output   : EPWM1 running at PWM_PERIOD
 *****************************************************************************/
void PWM_Init(void)
{
    InitEPWM(EPWM1_BASE);        /* Channel 1 PWM only */
}

/*****************************************************************************
 * Function : InitEPWM
 * Purpose  : One ePWM: up-count, period PWM_PERIOD, CMPA/CMPB load on zero.
 *            AQ: low on TB=0, high on up-count compare (active-low duty via CMPA).
 * Input    : n_base  -  EPWM1_BASE or EPWM2_BASE
 * Output   : that ePWM instance programmed
 *****************************************************************************/
void InitEPWM(uint32_t n_base)
{
#if 1
    /* Set Time Base Period and Phase */
    EPWM_setTimeBasePeriod(n_base, PWM_PERIOD);        /* Set frequency via TBPRD register */
    EPWM_setPhaseShift(n_base, 0U);                    /* Disable phase shift */
    EPWM_setTimeBaseCounter(n_base, 0U);               /* Reset the counter to zero */

    /* Synchronization Configuration */
    EPWM_setSyncInPulseSource(n_base, EPWM_SYNC_IN_PULSE_SRC_DISABLE); /* No sync-in pulse */
    EPWM_disablePhaseShiftLoad(n_base);                /* Disable loading counter from phase register */

    /* Set Initial Compare Values (Default to 0% or 100% duty based on logic) */
    EPWM_setCounterCompareValue(n_base, EPWM_COUNTER_COMPARE_A, PWM_PERIOD); /* CMPA = period => 0% high time */
    EPWM_setCounterCompareValue(n_base, EPWM_COUNTER_COMPARE_B, PWM_PERIOD); /* CMPB same */

    /* Counter Mode and Clocking */
    EPWM_setTimeBaseCounterMode(n_base, EPWM_COUNTER_MODE_UP); /* Up-count mode */
    EPWM_setClockPrescaler(n_base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1); /* System Clock / 1 */

    /* Shadow Loading Configuration (Load on Counter = 0) */
    EPWM_setCounterCompareShadowLoadMode(n_base, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO); /* Load CMPA at TB=0 */
    EPWM_setCounterCompareShadowLoadMode(n_base, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO); /* Load CMPB at TB=0 */

    /* Action Qualifier Settings: Output Low on Zero, High on Compare Match */
    EPWM_setActionQualifierAction(n_base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO); /* EPWMxA low at 0 */
    EPWM_setActionQualifierAction(n_base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO); /* EPWMxB low at 0 */
    EPWM_setActionQualifierAction(n_base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA); /* High on CMPA */
    EPWM_setActionQualifierAction(n_base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB); /* High on CMPB */
#endif
}

/*****************************************************************************
 * Function : Interrupt_Init
 * Purpose  : SCI-A RX (NEW ICD RS-422), CAN-A RX, CPU Timer0 at 2 ms
 *            (0x90 / 0x9A class). Then global IE.
 * Input    : none
 * Output   : IRQs enabled, Timer0 running
 *****************************************************************************/
void Interrupt_Init()
{
    /* SCI-A (RS-422) Receive Interrupt */
    Interrupt_register(INT_SCIA_RX, &SCIARxISR); /* NEW ICD RS-422 RX parser */
    Interrupt_enable(INT_SCIA_RX);               /* PIE enable SCI-A RX */

    /* CAN-A Receive Interrupt */
    Interrupt_register(INT_CANA0, &canISR);      /* CAN object-2 RX */
    Interrupt_enable(INT_CANA0);                 /* PIE enable CAN-A */

    /* Control Loop Timer (Timer 0) - 2 ms class tick for 0x9A */
    Interrupt_register(INT_TIMER0, &ControlISR); /* 2 ms flags for 0x9A / 0x63 */
    Timer0Init(2000);                                  /* 2000 us = 2 ms (0x90 / 0x9A class) */
    CPUTimer_startTimer(CPUTIMER0_BASE);         /* Start Timer0 */

    /* Global Interrupt Enable */
    Interrupt_enableMaster();                    /* CPU INTM = 0 */
}

/*****************************************************************************
 * Function : SCIARxISR
 * Purpose  : SCI-A receive. NEW ICD: EB90 MSGID MSGLEN MSGTXT XOR.
 *            When XOR of STX..MSGTXT matches CHKSUM, Uart_NewData = 1.
 *            Firmware/flash frames are accepted but not dispatched.
 * Input    : one UART byte from RS-422
 * Output   : Rx_Char[], rx_state, Uart_NewData
 *****************************************************************************/
__interrupt void SCIARxISR(void)
{
    uint8_t byte = SCI_readCharNonBlocking(RS422_BASE); /* One RS-422 byte from SCI-A RDbuf */

    switch(rx_state)                    /* NEW ICD STX EB90 parser */
    {
    case RX_WAIT_STX0:
        if(byte == ICD_STX0)            /* First STX byte 0xEB */
        {
            rx_index = 0u;              /* Restart the frame */
            Rx_Char[rx_index++] = byte; /* Store 0xEB */
            rx_state = RX_WAIT_STX1;    /* Next must be 0x90 */
        }
        break;

    case RX_WAIT_STX1:
        if(byte == ICD_STX1)            /* Second STX byte 0x90 */
        {
            Rx_Char[rx_index++] = byte; /* Store 0x90 */
            rx_state = RX_WAIT_MSGID;   /* Next is MSGID */
        }
        else
        {
            rx_state = RX_WAIT_STX0;    /* Not 0x90; hunt for 0xEB again */
            rx_index = 0u;              /* Drop partial header */
        }
        break;

    case RX_WAIT_MSGID:
        Rx_Char[rx_index++] = byte;     /* Store MSGID (0x90 / 0xB0 / 0x93 / ...) */
        rx_state = RX_WAIT_MSGLEN;      /* Next is MSGLEN */
        break;

    case RX_WAIT_MSGLEN:
        payload_len = byte;             /* MSGTXT length (0 allowed) */
        payload_cnt = 0u;               /* No payload bytes yet */
        Rx_Char[rx_index++] = byte;     /* Store MSGLEN */
        if(payload_len == 0u)           /* No MSGTXT; next byte is CHKSUM */
        {
            rx_state = RX_WAIT_CHKSUM;  /* Empty payload frame */
        }
        else if(payload_len > 200u)     /* Reject oversized MSGLEN */
        {
            rx_state = RX_WAIT_STX0;    /* Hunt for next STX */
            rx_index = 0u;              /* Drop bad frame */
        }
        else
        {
            rx_state = RX_WAIT_MSGTXT;  /* Collect MSGTXT */
        }
        break;

    case RX_WAIT_MSGTXT:
        Rx_Char[rx_index++] = byte;     /* One MSGTXT byte */
        payload_cnt++;                  /* Count stored payload bytes */
        if(payload_cnt >= payload_len)  /* All MSGTXT bytes in */
        {
            rx_state = RX_WAIT_CHKSUM;  /* Next is Method1 XOR */
        }
        break;

    case RX_WAIT_CHKSUM:
        calc_crc = IcdXorChecksum(&Rx_Char[0], rx_index); /* XOR of STX..MSGTXT */
        if((uint8_t)calc_crc == byte)   /* Method1 match */
        {
            Rx_Char[rx_index++] = byte; /* Store CHKSUM */
            Uart_NewData = 1u;          /* Packet validated */
            memcpy(OBC_DATA_ACT, Rx_Char, sizeof(Rx_Char)); /* Copy for dispatcher */
        }
        rx_state = RX_WAIT_STX0;        /* Always re-arm parser */
        rx_index = 0u;                  /* Ready for next 0xEB */
        break;

    default:
        rx_state = RX_WAIT_STX0;        /* Recover from a bad state */
        rx_index = 0u;                  /* Drop partial data */
        break;
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9); /* ACK PIE group 9 (SCI) */
}

/*****************************************************************************
 * Function : CRC16_CCITT
 * Purpose  : CRC-16 CCITT (poly 0x1021, init 0xFFFF) for ICD frame footer.
 * Input    : data  -  bytes; length  -  count (header through payload)
 * Output   : 16-bit CRC (low byte first on the wire)
 *****************************************************************************/
uint16_t CRC16_CCITT(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;              /* ICD init value */
    uint8_t i = 0u;                     /* Byte index */

    for(i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;  /* Mix next byte into high 8 bits */
        for(j = 0; j < 8; j++) /* 'j' is used from global scope */
        {
            if(crc & 0x8000)
                crc = (crc << 1) ^ 0x1021; /* Shift out 1: XOR poly */
            else
                crc <<= 1;              /* Shift out 0 */
        }
    }
    return crc;                         /* Low byte first on the wire */
}

/*****************************************************************************
 * Function : CRC16_MODBUS
 * Purpose  : NEW ICD Method2 checksum (Appendix A). Poly 0xA001, init 0xFFFF.
 *            Not placed on the wire while Method1 XOR is selected.
 * Input    : data, length
 * Output   : 16-bit MODBUS CRC
 *****************************************************************************/
uint16_t CRC16_MODBUS(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFu;             /* ICD Method2 init */
    uint16_t n = 0u;                    /* Byte index */
    uint8_t bit = 0u;                   /* Bit index */

    for(n = 0u; n < length; n++)        /* Each input byte */
    {
        crc ^= (uint16_t)data[n];       /* Mix data into CRC */
        for(bit = 0u; bit < 8u; bit++)  /* Eight bits LSB first */
        {
            if((crc & 0x0001u) != 0u)   /* LSB is 1 */
            {
                crc = (crc >> 1) ^ 0xA001u; /* Shift and XOR poly */
            }
            else
            {
                crc = crc >> 1;         /* Shift only */
            }
        }
    }
    return crc;                         /* 16-bit MODBUS CRC */
}

/*****************************************************************************
 * Function : IcdXorChecksum
 * Purpose  : NEW ICD Method1: XOR of STX, MSGID, MSGLEN, MSGTXT.
 * Input    : data (starts at STX), length (bytes before CHKSUM)
 * Output   : 1-byte XOR
 *****************************************************************************/
uint8_t IcdXorChecksum(const uint8_t *data, uint16_t length)
{
    uint8_t x = 0u;                     /* Running XOR */
    uint16_t n = 0u;                    /* Byte index */

    for(n = 0u; n < length; n++)        /* XOR every header/payload byte */
    {
        x ^= data[n];                   /* Mix next byte */
    }
    return x;                           /* Method1 CHKSUM */
}

/*****************************************************************************
 * Function : IcdSendFrame
 * Purpose  : Build STX EB90 | MSGID | MSGLEN | MSGTXT | XOR and send on SCI-A.
 * Input    : msgid, msgtxt (may be NULL if msglen 0), msglen
 * Output   : one NEW ICD reply on RS-422
 *****************************************************************************/
void IcdSendFrame(uint8_t msgid, const uint8_t *msgtxt, uint8_t msglen)
{
    FBK_Buffer[0] = ICD_STX0;           /* STX 0xEB */
    FBK_Buffer[1] = ICD_STX1;           /* STX 0x90 */
    FBK_Buffer[2] = msgid;              /* Message Type */
    FBK_Buffer[3] = msglen;             /* Length of MSGTXT */
    if((msglen > 0u) && (msgtxt != 0))  /* Copy payload if present */
    {
        memcpy(&FBK_Buffer[4], msgtxt, msglen); /* MSGTXT */
    }
    FBK_Buffer[4u + msglen] = IcdXorChecksum(FBK_Buffer, (uint16_t)(4u + msglen)); /* Method1 */
    PrintString(FBK_Buffer, (uint8_t)(5u + msglen)); /* Header 4 + payload + CHKSUM */
}

/*****************************************************************************
 * Function : icd_put_u16le
 * Purpose  : Pack one NEW ICD uint16 into MSGTXT as little-endian
 *            (low byte first). Used by 0x9A currents / PWM / POT / ADC,
 *            0x82 / 0x61 positions, 0x63 telemetry, 0xBB ADC snapshot.
 * Input    : p  - dest in MSGTXT (2 bytes)
 *            v  - 16-bit field (current, PWM, POT, ADC, status, watch)
 * Output   : p[0] = LSB, p[1] = MSB
 *****************************************************************************/
static void icd_put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);        /* Command: write LSB (bits 7..0) first */
    p[1] = (uint8_t)((v >> 8) & 0xFFu); /* Command: write MSB (bits 15..8) second */
}

/*****************************************************************************
 * Function : icd_put_u24be
 * Purpose  : Pack one NEW ICD 24-bit fin count as big-endian 3 bytes.
 *            Home / zero = 0x800000. Do-not-move = 0x000000.
 *            Channel 1: live Fin1 count. Fin2-4 slots get home / no-move pad only.
 * Input    : p  - dest in MSGTXT (3 bytes)
 *            v  - 24-bit count (ICD_HOME_COUNT / live Fin1 / ICD_NO_MOVE_COUNT)
 * Output   : p[0] = bits 23..16, p[1] = bits 15..8, p[2] = bits 7..0
 *****************************************************************************/
static void icd_put_u24be(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)((v >> 16) & 0xFFu); /* Command: write MSB of 24-bit count */
    p[1] = (uint8_t)((v >> 8) & 0xFFu);  /* Command: write mid byte of count */
    p[2] = (uint8_t)(v & 0xFFu);         /* Command: write LSB of count */
}

/*****************************************************************************
 * Function : icd_get_u24be
 * Purpose  : Read one NEW ICD 24-bit fin count from MSGTXT (big-endian).
 *            Channel 1 only: Fin1 Deflection at OBC_DATA_ACT[4]. Fin2-4 not read.
 * Input    : p  - 3-byte field (Fin1 Deflection)
 * Output   : 24-bit count; 0x800000 = home, 0x000000 = do not move Fin1
 *****************************************************************************/
static uint32_t icd_get_u24be(const uint8_t *p)
{
    return (((uint32_t)p[0] << 16) |    /* Command: take MSB as bits 23..16 */
            ((uint32_t)p[1] << 8) |     /* Command: take mid as bits 15..8 */
            (uint32_t)p[2]);            /* Command: take LSB as bits 7..0 */
}

/*****************************************************************************
 * Function : icd_put_f32le
 * Purpose  : Pack IEEE-754 float as little-endian 4 bytes into MSGTXT.
 *            NEW ICD 0x80 config fields 1-3: Kp, Kd, Ki (that order).
 * Input    : p  - dest in aceu_cfg[] / MSGTXT (4 bytes)
 *            f  - Kp / Kd / Ki as float
 * Output   : p[0] = bits 7..0, p[3] = bits 31..24
 *****************************************************************************/
static void icd_put_f32le(uint8_t *p, float f)
{
    converter_read.Digital_value_read = f; /* Command: overlay float onto uint32 bits */
    p[0] = (uint8_t)(converter_read.Conversion_Value_read & 0xFFu);         /* Command: write LE byte 0 */
    p[1] = (uint8_t)((converter_read.Conversion_Value_read >> 8) & 0xFFu);  /* Command: write LE byte 1 */
    p[2] = (uint8_t)((converter_read.Conversion_Value_read >> 16) & 0xFFu); /* Command: write LE byte 2 */
    p[3] = (uint8_t)((converter_read.Conversion_Value_read >> 24) & 0xFFu); /* Command: write LE byte 3 */
}

/*****************************************************************************
 * Function : icd_get_f32le
 * Purpose  : Read IEEE-754 float from MSGTXT as little-endian 4 bytes.
 *            NEW ICD 0xB1 config fields 1-3: Kp, Kd, Ki (that order).
 *            Channel 1 PID uses these three values.
 * Input    : p  - source in aceu_cfg[] / MSGTXT (4 bytes)
 * Output   : float Kp / Kd / Ki
 *****************************************************************************/
static float icd_get_f32le(const uint8_t *p)
{
    converter.Conversion_Value = ((uint32_t)p[0]) | /* Command: take LE byte 0 as bits 7..0 */
            ((uint32_t)p[1] << 8) |                 /* Command: take LE byte 1 as bits 15..8 */
            ((uint32_t)p[2] << 16) |                /* Command: take LE byte 2 as bits 23..16 */
            ((uint32_t)p[3] << 24);                 /* Command: take LE byte 3 as bits 31..24 */
    return converter.Digital_value;                 /* Command: return same bits as IEEE float */
}

/*****************************************************************************
 * Function : icd_pos_from_feedback
 * Purpose  : Map Channel 1 PID feedback (0..1000, mid 500 = home) to a
 *            NEW ICD 24-bit fin count. Home / zero = 0x800000.
 *            Used by 0x9A Current Fin1 Posn and Commanded Fin1 Posn.
 * Input    : feedback_value (Channel 1, 0..1000)
 * Output   : 24-bit count for icd_put_u24be (clamped 0..0xFFFFFF)
 *****************************************************************************/
static uint32_t icd_pos_from_feedback(void)
{
    int32_t delta;                      /* Command: signed counts from home */
    int32_t pos;                        /* Command: signed 24-bit position */

    delta = ((int32_t)feedback_value - 500) * 65536 / 500; /* Command: +/- span around mid 500 */
    pos = (int32_t)ICD_HOME_COUNT + delta; /* Command: offset-binary around 0x800000 */
    if(pos < 0)                         /* Command: clamp underflow */
    {
        pos = 0;                        /* Command: minimum 24-bit count */
    }
    if(pos > 0x00FFFFFF)                /* Command: clamp above 24-bit */
    {
        pos = 0x00FFFFFF;               /* Command: maximum 24-bit count */
    }
    return (uint32_t)pos;               /* Command: Channel 1 count for 0x9A */
}

/*****************************************************************************
 * Function : icd_apply_fin1_count
 * Purpose  : Apply NEW ICD 24-bit Fin1 count to Channel 1 setpoint.
 *            Used by 0x90 Fin Deflection (Fin1 only). 0x93 uses Analog_Command1
 *            directly. 0x000000 = do not move. 0x800000 = home.
 * Input    : count - big-endian 24-bit Fin1 from 0x90 MSGTXT
 * Output   : Analog_Command1 0..1000, Mode = 1, icd_status Motor1 bits
 *****************************************************************************/
static void icd_apply_fin1_count(uint32_t count)
{
    int32_t delta;                      /* Command: signed counts from home */

    if(count == ICD_NO_MOVE_COUNT)      /* Command: 0x000000 = do not move Channel 1 */
    {
        return;                         /* Command: keep last Analog_Command1 */
    }
    delta = (int32_t)count - (int32_t)ICD_HOME_COUNT; /* Command: signed deflection from 0x800000 */
    icd_status &= (uint16_t)~(1u << 8); /* Command: clear Fin1 out-of-range bit */
    if(delta > 65536)                   /* Command: above +span */
    {
        delta = 65536;                  /* Command: clamp +span */
        icd_status |= (1u << 8);        /* Command: bit 8 Motor 1 deflection outside allowed */
    }
    if(delta < -65536)                  /* Command: below -span */
    {
        delta = -65536;                 /* Command: clamp -span */
        icd_status |= (1u << 8);        /* Command: bit 8 Motor 1 deflection outside allowed */
    }
    Analog_Command1 = (uint32_t)(500 + ((delta * 500) / 65536)); /* Command: Channel 1 setpoint 0..1000 */
    if(Analog_Command1 > 1000u)         /* Command: saturate high */
    {
        Analog_Command1 = 1000u;        /* Command: max Channel 1 scale */
    }
    Mode = 1u;                          /* Command: digital ICD setpoint on Channel 1 */
    icd_status |= 0x0001u;              /* Command: bit 0 Motor 1 (Channel 1) enabled */
}

/*****************************************************************************
 * Function : canISR
 * Purpose  : CAN-A receive on message object 2. Stores 8 data bytes in rxData[].
 *            Full ICD '$'-'#' frames are not assembled here (RS-422 path is).
 * Input    : CAN interrupt
 * Output   : rxData[], received
 *****************************************************************************/
__interrupt void canISR(void)
{
    uint32_t status;                    /* CAN interrupt source (object number) */

    /* Read the CAN interrupt status to identify the interrupt source */
    status = CAN_getInterruptCause(CANA_BASE);

    /* Check if the interrupt was triggered by Message Object 2 (Receive Object) */
    if(status == 2)
    {
        /* Read the incoming CAN message into rxData buffer */
        CAN_readMessage(CANA_BASE, 2, rxData); /* 8 data bytes into rxData[] */
        received = true;                /* Main may poll this */

        /* Clear the specific message object interrupt flag */
        CAN_clearInterruptStatus(CANA_BASE, 2);
    }

    /* Clear the global interrupt flag for CAN line 0 */
    CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);

    /* Acknowledge PIE Group 9 interrupts */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

/*****************************************************************************
 * Function : ControlISR
 * Purpose  : CPU Timer0 ~every 2 ms (NEW ICD 0x90 / 0x9A class).
 *            Flag5 every 3 ticks (~6 ms) for 0x63 functional-test stream.
 * Input    : Timer0 interrupt
 * Output   : Flag2, Flag5, Flag40, Flag50, Flag100, -
 *****************************************************************************/
__interrupt void ControlISR(void)
{
    /* Increment timers and set software flags based on modulo triggers */
    if ((Timmer2 = (Timmer2 + 1) % 1) == 0)             Flag2 = 1;     /* 2 ms - 0x9A */
    if ((Timmer5 = (Timmer5 + 1) % 3) == 0)             Flag5 = 1;     /* ~6 ms - 0x63 */
    if ((Timmer40 = (Timmer40 + 1) % 20) == 0)          Flag40 = 1;    /* 40 ms spare */
    if ((Timmer50 = (Timmer50 + 1) % 25) == 0)          Flag50 = 1;    /* 50 ms spare */
    if ((Timmer100 = (Timmer100 + 1) % 50) == 0)        Flag100 = 1;   /* 100 ms spare */
    if ((Timmer500 = (Timmer500 + 1) % 250) == 0)       Flag500 = 1;   /* 500 ms spare */
    if ((Timmer1000 = (Timmer1000 + 1) % 500) == 0)     Flag1000 = 1;  /* 1 s spare */
    if ((Timmer2000 = (Timmer2000 + 1) % 1000) == 0)    Flag2000 = 1;  /* 2 s spare */
    if ((Timmer5000 = (Timmer5000 + 1) % 2500) == 0)    Flag5000 = 1;  /* 5 s spare */

    /* Acknowledge PIE Group 1 interrupts (CPU Timers) */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

/*****************************************************************************
 * Function : Timer0Init
 * Purpose  : CPU Timer0 period in microseconds (2000 - ICD 2 ms tick).
 * Input    : n_usecPeriod - period in -s
 * Output   : Timer0 programmed (started in Interrupt_Init)
 *****************************************************************************/
void Timer0Init(uint32_t n_usecPeriod)
{
    uint32_t n_timerPeriod;             /* SYSCLK ticks for n_usecPeriod */

    /* Calculate number of clock cycles for the requested period */
    n_timerPeriod = (DEVICE_SYSCLK_FREQ / 1000000) * n_usecPeriod; /* us * (Hz/1e6) */

    CPUTimer_setPeriod(CPUTIMER0_BASE, n_timerPeriod);             /* Load period into PRD register */
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);                      /* No clock division (SYSCLKOUT) */
    CPUTimer_stopTimer(CPUTIMER0_BASE);                           /* Ensure timer is halted during config */
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);                  /* Load period into counter register */

    /* Ensure timer stops during emulation/debug break */
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT); /* Halt in CCS */

    CPUTimer_enableInterrupt(CPUTIMER0_BASE);                      /* Enable local timer interrupt */
    Interrupt_enable(INT_TIMER0);                                  /* Enable PIE-level interrupt */
}

/*****************************************************************************
 * Function : Power_ON_SelfTest
 * Purpose  : Board POST after boot (NEW ICD has no 0xE2 / DSP POST word).
 *            Rails stuck at 0 or 65535, ALU. Then send 0x9A status.
 * Input    : none
 * Output   : Success or Failure; first 0x9A on RS-422
 *****************************************************************************/
uint8_t Power_ON_SelfTest(void)
{
    uint8_t Post_status = Success;

    /* Run hardware diagnostic routines */
    //    Post_status |= HALL_TEST();         /* Check Hall Effect Sensors */
    Post_status |= Voltage_checks();    /* Validate System Rail Voltages */
    Post_status |= ALU_Test();          /* Verify CPU Arithmetic Logic Unit integrity */

    HEALTH = 0x00;                      /* Local pass flag */
    HEALTH_PACKET_Feedback(ICD_FIN_STATUS_RSP);     /* NEW ICD 0x9A status / rails */

    return Post_status;                 /* Success unless a rail/ALU test failed */
}

/*****************************************************************************
 * Function : HALL_TEST
 * Purpose  : Hall commutation must not be 000 or 111. Not an ICD DSP POST bit.
 * Input    : HA_DSP, HB_DSP, HC_DSP GPIOs
 * Output   : Success/Failure; ACT_POST Hall bit
 *****************************************************************************/
uint8_t HALL_TEST(void)
{
    uint8_t HALL_A = GPIO_readPin(HA_DSP); /* Hall A */
    uint8_t HALL_B = GPIO_readPin(HB_DSP); /* Hall B */
    uint8_t HALL_C = GPIO_readPin(HC_DSP); /* Hall C */

    /* Valid commutation states for 3-phase Hall sensors never include 000 or 111 */
    if(((HALL_A == 0) && (HALL_B == 0) && (HALL_C == 0)) ||
            ((HALL_A == 1) && (HALL_B == 1) && (HALL_C == 1)))
    {
        ACT_POST &= ~Hall_Sensor; /* Log Hall failure in status register */
        return Failure;             /* 000 or 111 is illegal commutation */
    }

    ACT_POST |= Hall_Sensor;      /* Log Hall pass */
    return Success;                 /* Valid 001..110 state */
}

/*****************************************************************************
 * Function : Voltage_checks
 * Purpose  : ICD rail ADCs: fail if stuck at 0 or 65535.
 *            ICD health reports +1.1/+1.2/+3.3/+5/+28 V; DSP POST bits 4-8
 *            are 5 V / 28 V / 1.2 V / 3.3 V / 1.1 V.
 * Input    : AdcRawData[] after ReadIntADC()
 * Output   : Success/Failure; ACT_POST rail bits
 *****************************************************************************/
uint8_t Voltage_checks(void)
{
    ReadIntADC();                       /* Fresh 16-bit ICD samples */

    /* Individual rail checks to pinpoint failure in ACT_POST */
    if ((AdcRawData[Logic_5V] <= 0 || AdcRawData[Logic_5V] >= 65535))
    {
        ACT_POST &= ~ADC_5V;            /* ICD DSP POST bit 4 - 5 V ADC Fail */
        return Failure;
    }

    if ((AdcRawData[Logic_3V3] <= 0 || AdcRawData[Logic_3V3] >= 65535))
    {
        ACT_POST &= ~ADC_3V3;           /* ICD DSP POST bit 7 - 3.3 V Fail */
        return Failure;
    }

    if ((AdcRawData[Logic_1V2] <= 0 || AdcRawData[Logic_1V2] >= 65535))
    {
        ACT_POST &= ~ADC_1V2;           /* ICD DSP POST bit 6 - 1.2 V Fail */
        return Failure;
    }

    if ((AdcRawData[Logic_M_5V] <= 0 || AdcRawData[Logic_M_5V] >= 65535))
    {
        ACT_POST &= ~ADC_M5V;           /* Board -5 V stuck */
        return Failure;
    }

    if ((AdcRawData[LOGIC_PLUS_15] <= 0 || AdcRawData[LOGIC_PLUS_15] >= 65535))
    {
        ACT_POST &= ~ADC_P15V;          /* +15 V (0x9A ADC_28v slot) stuck */
        return Failure;
    }

    if ((AdcRawData[LOGIC_MINUS_15] <= 0 || AdcRawData[LOGIC_MINUS_15] >= 65535))
    {
        ACT_POST &= ~ADC_M15V;          /* -15 V stuck */
        return Failure;
    }

    if ((AdcRawData[LOGIC_3V] <= 0 || AdcRawData[LOGIC_3V] >= 65535))
    {
        ACT_POST &= ~ADC_3V;            /* +3 V stuck */
        return Failure;
    }

    /* Set pass bits for all checked voltage rails */
    ACT_POST |= (ADC_5V | ADC_3V3 | ADC_1V2 | ADC_M5V | ADC_P15V | ADC_M15V | ADC_3V); /* All rails Pass */
    return Success;
}

/*****************************************************************************
 * Function : ALU_Test
 * Purpose  : CPU add/sub/mul/div/shift. Not an ICD DSP POST bit
 *            (ICD lists SRAM/FLASH/STACK/cfg/rails).
 * Input    : none
 * Output   : Success/Failure; ACT_POST ALU bit
 *****************************************************************************/
uint8_t ALU_Test(void)
{
    const uint16_t Var1 = 50, Var2 = 50; /* Changed to 16-bit to handle Mul (2500) */
    uint16_t Result = 0;                /* ALU result under test */

    /* Addition Test: 50 + 50 = 100 */
    Result = (Var1 + Var2);
    if(Result != 100) { ACT_POST &= ~ALU; return Failure; } /* Add fail */

    /* Subtraction Test: 50 - 50 = 0 */
    Result = (Var1 - Var2);
    if(Result != 0) { ACT_POST &= ~ALU; return Failure; } /* Sub fail */

    /* Multiplication Test: 50 * 50 = 2500 */
    Result = (Var1 * Var2);
    if(Result != 2500) { ACT_POST &= ~ALU; return Failure; } /* Mul fail */

    /* Division Test: 50 / 50 = 1 */
    Result = (Var1 / Var2);
    if(Result != 1) { ACT_POST &= ~ALU; return Failure; } /* Div fail */

    /* Bit Shift Test: 50 >> 1 = 25 */
    Result = Var1 >> 1;
    if(Result != 25) { ACT_POST &= ~ALU; return Failure; } /* Shift fail */

    ACT_POST |= ALU;                    /* CPU math Pass */
    return Success;
}

/*****************************************************************************
 * Function : PrintString
 * Purpose  : Send an ICD reply on SCI-A (RS-422 to GUI).
 * Input    : p_inBuffer - bytes; Data - count
 * Output   : those bytes on the UART
 *****************************************************************************/
void PrintString(uint8_t *p_inBuffer, uint8_t Data)
{
    /* Transmit Data to SCI-A */
    SCI_writeCharArray(RS422_BASE, (uint8_t*)p_inBuffer, Data); /* Blocking TX of ICD reply */

    /* Allow time for the physical transmission of bytes */
    DEVICE_DELAY_US(1000);              /* Gap so GUI can take the frame */
}

/*****************************************************************************
 * Function : gpioInterruptHandler
 * Purpose  : External GPIO IRQ (Hall / spare). ACK only in this build.
 * Input    : GPIO interrupt
 * Output   : PIE group 1 ACK
 *****************************************************************************/
__interrupt void gpioInterruptHandler(void)
{
    /* Acknowledge PIE Group 1 */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1); /* Required PIE ACK */
}

/*****************************************************************************
 * Function : Checksum_Calculation
 * Purpose  : XOR checksum helper. NEW ICD Method1 uses IcdXorChecksum.
 * Input    : Data - array; NoofBytes - count
 * Output   : XOR result
 *****************************************************************************/
uint16_t Checksum_Calculation(uint8_t *Data, uint8_t NoofBytes)
{
    uint16_t checksum = 0u;             /* Running XOR */
    uint8_t i;                          /* Byte index */

    for (i = 0u; i < NoofBytes; i++)
    {
        checksum ^= Data[i];            /* XOR into checksum */
    }
    return checksum;                    /* Legacy; ICD uses CRC16 */
}

/*****************************************************************************
 * Function : Motor_Control
 * Purpose  : Drive Channel 1 only (Fin1 / Motor1 / EPWM1).
 *            Mode 0 = analog cmd ADC (A0); Mode 1 = 0x90 / 0x93 Analog_Command1.
 *            Feedback ADC mapped 0.348..2.652 V to 0..1000. PID uses ICD Kp/Ki/Kd.
 * Input    : AdcRawData[Position_command], AdcRawData[Position_Feedback], Analog_Command1
 * Output   : Direction pin + ePWM1 duty; OVCOAST/BREAK released
 *****************************************************************************/
void Motor_Control(void)
{

#if 1
    /* Determine Setpoint: Mode 0 = Analog Command, Mode 1 = Digital Command */
    if(Mode == 0)
    {
        set_value = (AdcRawData[Position_command] / 65.535); /* A0 analog cmd -> 0..1000 */
    }
    else
    {
        set_value = Analog_Command1;    /* NEW ICD digital setpoint from 0x90 / 0x93 */
    }

    /* Convert feedback ADC counts to scaled position value */
    //        feedback_value = (AdcRawData[Position_Feedback] / 65.535);


    /* Convert feedback ADC counts to scaled position value (0.348V to 2.652V mapped to 0-1000) */
    float raw_feedback = (float)AdcRawData[Position_Feedback]; /* A1 fin ADC counts */

    //     0.348V corresponds to ~7602 raw ADC counts
    if (raw_feedback < 7602.0f)
    {
        feedback_value = 0.0f;          /* Below 0.348 V => 0 deg scale */
    }
    else
    {
        // 50.331 is the new scaling divisor for the narrower voltage span
        feedback_value = (raw_feedback - 7602.0f) / 50.331f; /* Map 0.348..2.652 V to 0..1000 */
    }

    //     Clamp upper limit to prevent overshoot values if voltage exceeds 2.652V
    if (feedback_value > 1000.0f)
    {
        feedback_value = 1000.0f;       /* Cap at full-scale deflection */
    }

    /* Calculate the positional error */
    error_value = set_value - feedback_value; /* + => cmd ahead of fin */
    //            error_value = set_value - 500;

    //    if (error_value >= 0.0f && error_value <= 0.1)
    //    {
    //        GPIO_writePin(Direction, SET);
    //
    //        /* =========================================================================
    //         * HARDWARE FIX: Trip Zone Assisted 100% PWM Pulse for Exactly 2ms
    //         * ========================================================================= */
    //
    //        /* 2. Set Counter Compare A to match the full Period value (100% Duty Cycle) */
    //        EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
    //
    //        /* 3. HARDWARE BLOCKING DELAY: Stall execution for exactly 2ms */
    //        DEVICE_DELAY_US(5000);
    //
    //        /* 4. FORCE TRIP ZONE: Digitally clamp EPWM1 Output A to a hard LOW instantly */
    //        EPWM_setTripZoneAction(EPWM1_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    //        EPWM_forceTripZoneEvent(EPWM1_BASE, EPWM_TZ_FORCE_EVENT_OST);
    //
    //        /* 5. Set the baseline compare value back to 0 */
    //        EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
    //
    //        /* ========================================================================= */
    //
    //        /* Initialize background tracking timer back to its 1ms baseline rate */
    ////        Timer0Init(1000);
    //    }

    /* Direction Control Logic and Error Absolute Value */
    if(error_value > 0)
    {
        GPIO_writePin(Direction, RESET); /* One rotation sense */
    }
    else
    {
        GPIO_writePin(Direction, SET);  /* Opposite sense */
        error_value = -error_value;     /* PWM uses |error| */
    }


    /* Proportional Control Calculation */
    //        output = (kp * error_value);

    //    output = (kp * error_value) + integral + derivative;      // Calculate the output, pwm.


    Pterm = (kp * error_value);         /* ICD Kp Position */
    I_t = (I_t +(error_value * Ts));    /* Integrate error */

    // Anti-Windup: Clamp the integrator accumulation to your max PWM range
    if (I_t > 700.0f)
    {
        I_t = 700.0f;                   /* Upper anti-windup */
    }
    else if (I_t < -700.0f)
    {
        I_t = -700.0f;                  /* Lower anti-windup */
    }

    Iterm = ki * I_t;                   /* ICD Ki Position */
    D_t =((error_value - pre_error)/Ts); /* d(error)/dt */
    Dterm = kd * D_t;                   /* ICD Kd Position */

    output = Pterm + Iterm + Dterm;     /* PWM command in counts */


    /* Output Saturation and Deadband Logic */
    if (output > PWM_PERIOD)
    {
        output = PWM_PERIOD; /* Maximize PWM if output exceeds threshold */
    }
    else if (output < 5)   //motor will stop if O/P iessthen 60 and PWM will gen 0%
    {
        output = 0;          /* Apply deadband to prevent motor jitter */
        I_t = 0.0f;                     /* Clear integrator at stop */
    }

    /* Update PWM Duty Cycle (Active Low Logic) */
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(PWM_PERIOD - output)); /* Duty = period - output */

    /* Diagnostics: Store current Compare value */
    //    GET_ERRor = EPWM_getCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A); // not using

    /* Enable Motor Driver outputs (Disable Coast and Brake) */
    GPIO_writePin(OVCOAST, RESET);      /* Release coast (run) */
    GPIO_writePin(BREAK, RESET);        /* Release brake (run) */

    /* Store current error for future Derivative (Kd) calculations */
    pre_error = error_value;            /* For next D term */


#else
    if(Mode == 0)
    {
        set_value = (AdcRawData[Position_command] / 65.535) ;
    }
    else
    {
        set_value = Analog_Command1 ;
    }
    feedback_value = (AdcRawData[Position_Feedback] / 65.535) ;

    error_value = set_value - feedback_value;                               // Calculate the error.


    if(error_value > 0)
    {
        GPIO_writePin(Direction, RESET) ;
    }
    else
    {
        GPIO_writePin(Direction, SET) ;
        error_value = error_value * (-1) ;
    }

    //    derivative = error_value - pre_error;                                   // Calculate derivative.
    //    derivative = ((kd * derivative)/Ts);

    //    integral  = integral + (ki * error_value * Ts);
    //    if (integral > 1.0)
    //    {
    //        integral = 1.0;
    //    }
    //    else if (integral < -1.0)
    //    {
    //        integral = -1.0;
    //    }

    output = (kp * error_value) + integral + derivative;      // Calculate the output, pwm.

    if (output > 700)
    {
        output = PWM_PERIOD;
    }
    else if (output < 10)
    {
        output = 0;
        integral = 0;
    }

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, PWM_PERIOD - output) ;
    GET_ERRor = EPWM_getCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A);
    GPIO_writePin(OVCOAST, RESET) ;
    GPIO_writePin(BREAK, RESET) ;

    pre_error = error_value;            // Save as previous error.

#endif
}

//#include <stdbool.h> // Required for boolean data types (bool, true, false)
//#include <stdint.h>  // Standard integer types
//
///* --- Global Variables for Zero-Point Kickstart Feature --- */
//volatile float zero_pulse_timer = 0.0f;
//volatile bool zero_pulse_active = false;
//
//// Note: Ensure other global variables like Mode, AdcRawData, Position_command,
//// Position_Feedback, set_value, feedback_value, error_value, kp, ki, kd, Ts,
//// I_t, pre_error, PWM_PERIOD, Direction, OVCOAST, BREAK, etc.,
//// are declared globally either in this file or via your shared header headers.
//
///**
// * @brief Primary Motor Control Loop.
// * Implements a PID-controller with direction switching, deadband, and initial zero pulse.
// */
//void Motor_Control(void)
//{
//#if 1
//    /* Determine Setpoint: Mode 0 = Analog Command, Mode 1 = Digital Command */
//    if(Mode == 0)
//    {
//        set_value = (AdcRawData[Position_command] / 65.535f);
//    }
//    else
//    {
//        set_value = Analog_Command1;
//    }
//
//    /* Convert feedback ADC counts to scaled position value (0.348V to 2.652V mapped to 0-1000) */
//    float raw_feedback = (float)AdcRawData[Position_Feedback];
//
//    // 0.348V corresponds to ~7602 raw ADC counts
//    if (raw_feedback < 7602.0f)
//    {
//        feedback_value = 0.0f;
//    }
//    else
//    {
//        // 50.331 is the new scaling divisor for the narrower voltage span
//        feedback_value = (raw_feedback - 7602.0f) / 50.331f;
//    }
//
//    // Clamp upper limit to prevent overshoot values if voltage exceeds 2.652V
//    if (feedback_value > 1000.0f)
//    {
//        feedback_value = 1000.0f;
//    }
//
//    /* --- NEW FEATURE: 100% PWM for 2ms when command & feedback are both 0 --- */
//    if (set_value == 0.0f && feedback_value == 0.0f)
//    {
//        // If the pulse hasn't fired yet and the timer is clear, initialize the 2ms countdown
//        if (!zero_pulse_active && zero_pulse_timer <= 0.0f)
//        {
//            zero_pulse_active = true;
//            zero_pulse_timer = 2.00f; // Set tracking window to 2 milliseconds (0.002 seconds)
//        }
//    }
//    else
//    {
//        // Reset tracking flags immediately if target conditions are broken
//        zero_pulse_active = false;
//        zero_pulse_timer = 0.0f;
//    }
//
//    // Check if the 2ms pulse override condition is active
//    if (zero_pulse_active && zero_pulse_timer > 0.0f)
//    {
//        output = PWM_PERIOD;             // Force 100% full-throttle PWM Duty Cycle
//        GPIO_writePin(Direction, SET); // Maintain a stable base direction
//
//        // Non-blocking reduction based on loop sampling period (Ts)
//        zero_pulse_timer -= Ts;
//
//        if (zero_pulse_timer <= 0.0f)
//        {
//            zero_pulse_active = false; // 2ms elapsed! Smoothly transition control back to PID
//        }
//
//        // Zero out control history to prevent structural integrator jumps or derivative spiking
//        I_t = 0.0f;
//        error_value = 0.0f;
//        pre_error = 0.0f;
//    }
//    else
//    {
//        /* --- STANDARD PID CONTROL LOOP --- */
//
//        /* Calculate the positional error */
//        error_value = set_value - feedback_value;
//
//        if(error_value == 0)
//        {
//            EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
//            GPIO_writePin(Direction, SET);
//        }
//
//        /* Direction Control Logic and Error Absolute Value */
//        if(error_value > 0)
//        {
//            GPIO_writePin(Direction, RESET);
//        }
//        else
//        {
//            GPIO_writePin(Direction, SET);
//            error_value = -error_value;
//        }
//
//        /* Proportional Control Calculation */
//        Pterm = (kp * error_value);
//        I_t = (I_t + (error_value * Ts));
//
//        // Anti-Windup: Clamp the integrator accumulation to your max PWM range
//        if (I_t > 700.0f)
//        {
//            I_t = 700.0f;
//        }
//        else if (I_t < -700.0f)
//        {
//            I_t = -700.0f;
//        }
//
//        Iterm = ki * I_t;
//        D_t = ((error_value - pre_error) / Ts);
//        Dterm = kd * D_t;
//
//        output = Pterm + Iterm + Dterm;
//
//        /* Output Saturation and Deadband Logic */
//        if (output > PWM_PERIOD)
//        {
//            output = PWM_PERIOD; /* Maximize PWM if output exceeds threshold */
//        }
//        else if (output < 5) // motor will stop if O/P less than 5
//        {
//            output = 0; /* Apply deadband to prevent motor jitter */
//            I_t = 0.0f;
//        }
//
//        /* Store current error for future Derivative (Kd) calculations */
//        pre_error = error_value;
//    }
//
//    /* Update PWM Duty Cycle (Active Low Logic) */
//    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(PWM_PERIOD - output));
//
//    /* Enable Motor Driver outputs (Disable Coast and Brake) */
//    GPIO_writePin(OVCOAST, RESET);
//    GPIO_writePin(BREAK, RESET);
//
//#endif
//}



/*****************************************************************************
 * Function : ACEU_CONFIG_READ
 * Purpose  : NEW ICD 0xB0 -> 0x80. Send 102-byte configuration (RAM).
 *            Layout: Kp, Kd, Ki, Kawg, B, C, MaxAcc, MaxVel, Gear, Counts/Rev,
 *            IntLimit, IntCount, Debounce, Sample, Subsample, MotEnLimit,
 *            PWM Divider, HW PID, Discrete PID, Min/Max defl, POT scales, spares.
 *            Flash write is not updated (firmware/flash left alone).
 * Input    : Command unused (reply MSGID is always 0x80)
 * Output   : EB90 0x80 102-byte cfg XOR
 *****************************************************************************/
void ACEU_CONFIG_READ(uint8_t Command)
{
    (void)Command;                      /* Reply MSGID is fixed 0x80 */
    icd_put_f32le(&aceu_cfg[0], (float)kp); /* Field 1 Kp */
    icd_put_f32le(&aceu_cfg[4], (float)kd); /* Field 2 Kd (NEW order, not Ki) */
    icd_put_f32le(&aceu_cfg[8], (float)ki); /* Field 3 Ki */
    IcdSendFrame(ICD_CFG_READ_RSP, aceu_cfg, ICD_CFG_LEN); /* 0x80 MSGLEN 102 */
}

/*****************************************************************************
 * Function : ACEU_CONFIG_WRITE
 * Purpose  : NEW ICD 0xB1. Copy 102-byte MSGTXT to RAM and apply Kp/Kd/Ki.
 *            Reply 0x81 Flash Writing Status: 1 = RAM accept.
 *            Flash programming is not updated.
 * Input    : OBC_DATA_ACT[4..] 102 bytes
 * Output   : live kp/kd/ki; 0x81 status 1
 *****************************************************************************/
void ACEU_CONFIG_WRITE(void)
{
    uint8_t status = 1u;                /* 1 = Success (RAM). Flash not written. */

    memcpy(aceu_cfg, &OBC_DATA_ACT[4], ICD_CFG_LEN); /* Store NEW cfg block */
    kp = (float64_t)icd_get_f32le(&aceu_cfg[0]); /* Apply Kp */
    kd = (float64_t)icd_get_f32le(&aceu_cfg[4]); /* Apply Kd */
    ki = (float64_t)icd_get_f32le(&aceu_cfg[8]); /* Apply Ki */
    IcdSendFrame(ICD_CFG_WRITE_RSP, &status, 1u); /* 0x81 one-byte status */
}

/*****************************************************************************
 * Function : HEALTH_PACKET_Feedback
 * Purpose  : NEW ICD 0x9A 74-byte status (replaces old 0xE2 Health).
 *            Channel 1 only: Fin1 / Motor1 live. Fin2-4 slots are ICD pad.
 * Input    : Command unused (reply MSGID is 0x9A)
 * Output   : EB90 0x9A 74-byte MSGTXT XOR
 *****************************************************************************/
void HEALTH_PACKET_Feedback(uint8_t Command)
{
    uint8_t txt[ICD_STATUS_LEN];        /* 74-byte 0x9A body (ICD length kept) */
    uint32_t fin1;                      /* Current Channel 1 count */
    uint16_t imot;                      /* Channel 1 12-bit offset-binary current */
    uint16_t pwm_rep;                   /* Channel 1: 0 = fastest, 1000 = not moving */
    uint16_t i;                         /* Fill index */

    (void)Command;                      /* Always send 0x9A */
    memset(txt, 0, sizeof(txt));        /* ICD pad / unused slots = 0 */
    fin1 = icd_pos_from_feedback();     /* Channel 1 PID feedback -> 0x800000 home */
    icd_put_u24be(&txt[0], fin1);       /* Current Fin1 Posn (Channel 1) */
    icd_put_u24be(&txt[3], ICD_HOME_COUNT); /* Fin2 pad (no Channel 2) */
    icd_put_u24be(&txt[6], ICD_HOME_COUNT); /* Fin3 pad (no Channel 3) */
    icd_put_u24be(&txt[9], ICD_HOME_COUNT); /* Fin4 pad (no Channel 4) */
    icd_put_u24be(&txt[12], fin1);      /* Commanded Fin1 (Channel 1) */
    icd_put_u24be(&txt[15], ICD_NO_MOVE_COUNT); /* Fin2 pad: do not move */
    icd_put_u24be(&txt[18], ICD_NO_MOVE_COUNT); /* Fin3 pad: do not move */
    icd_put_u24be(&txt[21], ICD_NO_MOVE_COUNT); /* Fin4 pad: do not move */
    imot = (uint16_t)(0x800u + (((int32_t)AdcRawData[Current_Feedback] - 32768) / 16)); /* Channel 1 current */
    icd_put_u16le(&txt[24], imot);      /* Motor1 Current (Channel 1) */
    icd_put_u16le(&txt[26], 0x800u);    /* Motor2 pad (mid / unused) */
    icd_put_u16le(&txt[28], 0x800u);    /* Motor3 pad (mid / unused) */
    icd_put_u16le(&txt[30], 0x800u);    /* Motor4 pad (mid / unused) */
    if(output < 5.0)                    /* Channel 1 deadband = not moving */
    {
        icd_status |= (1u << 4);        /* Bit 4: Motor 1 is not moving */
        pwm_rep = 1000u;                /* ICD: 1000 = not moving */
    }
    else
    {
        icd_status &= (uint16_t)~(1u << 4); /* Motor 1 moving */
        pwm_rep = (uint16_t)(1000u - ((uint32_t)output * 1000u / PWM_PERIOD)); /* 0 = fastest */
    }
    icd_put_u16le(&txt[32], icd_status); /* Status bits 0-11 (Motor 1 used) */
    icd_put_u16le(&txt[34], pwm_rep);   /* Motor1 PWM (Channel 1) */
    icd_put_u16le(&txt[36], 1000u);     /* Motor2 pad idle */
    icd_put_u16le(&txt[38], 1000u);     /* Motor3 pad idle */
    icd_put_u16le(&txt[40], 1000u);     /* Motor4 pad idle */
    icd_put_u16le(&txt[42], (uint16_t)AdcRawData[Position_Feedback]); /* POT 1 (Channel 1) */
    icd_put_u16le(&txt[44], 0u);        /* POT 2 pad */
    icd_put_u16le(&txt[46], 0u);        /* POT 3 pad */
    icd_put_u16le(&txt[48], 0u);        /* POT 4 pad */
    icd_watch++;                        /* Watch Variable LSB counter */
    icd_put_u16le(&txt[50], (uint16_t)(icd_watch & 0x00FFu)); /* Watch; MSB 0 = Method1 */
    icd_put_u16le(&txt[52], (uint16_t)AdcRawData[Logic_5V]); /* ADC_5v */
    icd_put_u16le(&txt[54], (uint16_t)AdcRawData[LOGIC_PLUS_15]); /* ADC_28v slot = +15 V on this board */
    icd_put_u16le(&txt[56], (uint16_t)AdcRawData[LOGIC_PWM]); /* Supply_1v1 slot = PWM sense */
    for(i = 58u; i < 66u; i++)          /* Watch Variable 3+3+2 unused */
    {
        txt[i] = 0u;                    /* Pad */
    }
    icd_put_u16le(&txt[66], (uint16_t)AdcRawData[Logic_3V3]); /* Supply_3v3_VDD */
    icd_put_u16le(&txt[68], (uint16_t)AdcRawData[Logic_1V2]); /* Supply_2p5 slot = 1.2 V on this board */
    icd_put_u16le(&txt[70], (uint16_t)AdcRawData[LOGIC_3V]); /* Supply_3v3_VCC slot = +3 V */
    icd_put_u16le(&txt[72], (uint16_t)(icd_watch & 0x00FFu)); /* Watch Variable field 32 */
    IcdSendFrame(ICD_FIN_STATUS_RSP, txt, ICD_STATUS_LEN); /* 0x9A */
}

/*****************************************************************************
 * Function : HOME_ALL_Feedback
 * Purpose  : NEW ICD 0x91 -> 0x61. Home Channel 1 only (Fin1 = 0x800000).
 *            Fin2-4 position slots are ICD pad (body 8 kept).
 * Input    : Command unused
 * Output   : Analog_Command1 mid-scale; 0x61 frame
 *****************************************************************************/
void HOME_ALL_Feedback(uint8_t Command)
{
    uint8_t txt[ICD_FIN_POS_LEN];       /* 8-byte ICD positions (Ch1 + pad) */
    uint16_t p1;                        /* Channel 1 16-bit report */

    (void)Command;                      /* Reply MSGID is 0x61 */
    Analog_Command1 = 500u;             /* Home Channel 1 = mid PID scale */
    Mode = 1u;                          /* Digital home on Channel 1 */
    p1 = (uint16_t)AdcRawData[Position_Feedback]; /* Current Fin1 ADC */
    icd_put_u16le(&txt[0], p1);         /* Fin1 Position (Channel 1) */
    icd_put_u16le(&txt[2], 0u);         /* Fin2 pad */
    icd_put_u16le(&txt[4], 0u);         /* Fin3 pad */
    icd_put_u16le(&txt[6], 0u);         /* Fin4 pad */
    IcdSendFrame(ICD_HOME_ALL_RSP, txt, ICD_FIN_POS_LEN); /* 0x61 */
}

/*****************************************************************************
 * Function : FIN_POS_READ_Feedback
 * Purpose  : NEW ICD 0xB2 -> 0x82. Channel 1 position live; Fin2-4 pad.
 * Input    : none
 * Output   : 0x82 frame
 *****************************************************************************/
void FIN_POS_READ_Feedback(void)
{
    uint8_t txt[ICD_FIN_POS_LEN];       /* 8-byte ICD body (Ch1 + pad) */

    icd_put_u16le(&txt[0], (uint16_t)AdcRawData[Position_Feedback]); /* Fin1 (Channel 1) */
    icd_put_u16le(&txt[2], 0u);         /* Fin2 pad */
    icd_put_u16le(&txt[4], 0u);         /* Fin3 pad */
    icd_put_u16le(&txt[6], 0u);         /* Fin4 pad */
    IcdSendFrame(ICD_FIN_POS_RSP, txt, ICD_FIN_POS_LEN); /* 0x82 */
}

/*****************************************************************************
 * Function : ADC_ACQ_Feedback
 * Purpose  : NEW ICD 0xAA -> 0xBB 32-byte ADC snapshot.
 * Input    : AdcRawData[]
 * Output   : 0xBB frame
 *****************************************************************************/
void ADC_ACQ_Feedback(void)
{
    uint8_t txt[ICD_ADC_LEN];           /* 32-byte ADC body */

    memset(txt, 0, sizeof(txt));        /* Unused supplies = 0 */
    icd_put_u16le(&txt[0], (uint16_t)AdcRawData[Position_Feedback]); /* POT Output 1 (Channel 1) */
    icd_put_u16le(&txt[2], 0u);         /* POT 2 pad */
    icd_put_u16le(&txt[4], 0u);         /* POT 3 pad */
    icd_put_u16le(&txt[6], 0u);         /* POT 4 pad */
    icd_put_u16le(&txt[8], (uint16_t)AdcRawData[Logic_5V]); /* 5 V */
    icd_put_u16le(&txt[10], (uint16_t)AdcRawData[Current_Feedback]); /* Fin 1 Current (Channel 1) */
    icd_put_u16le(&txt[12], 0u);        /* Fin 2 Current pad */
    icd_put_u16le(&txt[14], 0u);        /* Fin 3 Current pad */
    icd_put_u16le(&txt[16], 0u);        /* Fin 4 Current pad */
    icd_put_u16le(&txt[18], (uint16_t)AdcRawData[Current_Feedback]); /* Total Current */
    icd_put_u16le(&txt[20], (uint16_t)AdcRawData[LOGIC_PWM]); /* 1.1 v Supply slot */
    icd_put_u16le(&txt[22], 0u);        /* 1.0 v Supply */
    icd_put_u16le(&txt[24], 0u);        /* 1.8 v Supply */
    icd_put_u16le(&txt[26], (uint16_t)AdcRawData[Logic_3V3]); /* 3.3 VDD */
    icd_put_u16le(&txt[28], (uint16_t)AdcRawData[Logic_1V2]); /* 2.5 V slot */
    icd_put_u16le(&txt[30], (uint16_t)AdcRawData[LOGIC_3V]); /* 3.3 Vcc slot */
    IcdSendFrame(ICD_ADC_ACQ_RSP, txt, ICD_ADC_LEN); /* 0xBB */
}

/*****************************************************************************
 * Function : STEP_SINE_TRAPEZOIDAL_START_Feedback
 * Purpose  : NEW ICD 0x63 functional-test telemetry (replaces old 0x8D stream).
 *            Channel 1 only. Fin2-4 / temp2-4 / POT2-4 are ICD pad.
 * Input    : Command unused
 * Output   : 0x63 46-byte record
 *****************************************************************************/
void STEP_SINE_TRAPEZOIDAL_START_Feedback(uint8_t Command)
{
    uint8_t txt[ICD_FUNC_TEST_RSP_LEN]; /* 46-byte 0x63 body (ICD length kept) */
    uint16_t p1;                        /* Channel 1 position */

    (void)Command;                      /* Reply MSGID is 0x63 */
    memset(txt, 0, sizeof(txt));        /* Unused ICD slots = 0 */
    p1 = (uint16_t)AdcRawData[Position_Feedback]; /* Achieved Fin1 */
    icd_put_u16le(&txt[0], p1);         /* Achieved FIN 1 (Channel 1) */
    icd_put_u16le(&txt[2], 0u);         /* Achieved FIN 2 pad */
    icd_put_u16le(&txt[4], 0u);         /* Achieved FIN 3 pad */
    icd_put_u16le(&txt[6], 0u);         /* Achieved FIN 4 pad */
    icd_put_u16le(&txt[8], (uint16_t)Analog_Command1); /* Commanded FIN 1 (Channel 1) */
    icd_put_u16le(&txt[10], 0u);        /* Commanded FIN 2 pad */
    icd_put_u16le(&txt[12], 0u);        /* Commanded FIN 3 pad */
    icd_put_u16le(&txt[14], 0u);        /* Commanded FIN 4 pad */
    icd_put_u16le(&txt[16], (uint16_t)AdcRawData[Current_Feedback]); /* Fin 1 Current (Channel 1) */
    icd_put_u16le(&txt[18], 0u);        /* Fin 2 Current pad */
    icd_put_u16le(&txt[20], 0u);        /* Fin 3 Current pad */
    icd_put_u16le(&txt[22], 0u);        /* Fin 4 Current pad */
    icd_put_u16le(&txt[24], Temp);      /* Temperature1 (this board) */
    icd_put_u16le(&txt[26], 0u);        /* Temperature2 pad */
    icd_put_u16le(&txt[28], 0u);        /* Temperature3 pad */
    icd_put_u16le(&txt[30], 0u);        /* Temperature4 pad */
    icd_put_u16le(&txt[32], p1);        /* POT_OUTPUT1 (Channel 1) */
    icd_put_u16le(&txt[34], 0u);        /* POT_OUTPUT2 pad */
    icd_put_u16le(&txt[36], 0u);        /* POT_OUTPUT3 pad */
    icd_put_u16le(&txt[38], 0u);        /* POT_OUTPUT4 pad */
    icd_put_u16le(&txt[40], (uint16_t)AdcRawData[Logic_5V]); /* 5V */
    icd_put_u16le(&txt[42], (uint16_t)AdcRawData[LOGIC_PLUS_15]); /* +28V slot */
    icd_put_u16le(&txt[44], icd_watch); /* DEBUG_Counter */
    IcdSendFrame(ICD_FUNC_TEST_RSP, txt, ICD_FUNC_TEST_RSP_LEN); /* 0x63 */
}

/*****************************************************************************
 * Function : STEP_SINE_TRAPEZOIDAL_STOP_Feedback
 * Purpose  : Coast/brake only. NEW ICD has no stop MSGID (old 0xE5/E7/E9/EB gone).
 * Input    : Command unused
 * Output   : Motor coast and brake engaged
 *****************************************************************************/
void STEP_SINE_TRAPEZOIDAL_STOP_Feedback(uint8_t Command)
{
    (void)Command;                      /* No NEW ICD stop opcode */
    GPIO_writePin(OVCOAST, SET);        /* Coast on */
    GPIO_writePin(BREAK, SET);          /* Brake on */
}

/*****************************************************************************
 * Function : RSS_422_New_Data
 * Purpose  : If UART ISR finished a NEW ICD frame, copy it for dispatch.
 * Input    : Uart_NewData, Rx_Char[]
 * Output   : OBC_DATA_ACT[]; Uart_NewData cleared
 *****************************************************************************/
void RSS_422_New_Data(void)
{
    if(Uart_NewData == 1u)              /* ISR accepted EB90 XOR frame */
    {
        Uart_NewData = 0u;              /* Do not parse this frame twice */
        memcpy(OBC_DATA_ACT, Rx_Char, sizeof(Rx_Char)); /* MSGID at [2] */
    }
}

/*****************************************************************************
 * Function : Digital_Data_Validation
 * Purpose  : Dispatch NEW ICD MSGID in OBC_DATA_ACT[2] (ACEU_ICD_V1_AMS).
 *            Channel 1 only: apply Fin1 / Motor1. Fin2-4 fields ignored.
 *            Live: 0x90/0x9A, 0xB0/0x80, 0xB1/0x81, 0xB2/0x82, 0x91/0x61,
 *            0x93/0x63, 0xAA/0xBB.
 *            Firmware/flash 0xA5 / 0xB5 / 0xC5 are not updated.
 * Input    : OBC_DATA_ACT[]
 * Output   : Analog_Command1, Mode, kp/kd/ki, UART replies
 *****************************************************************************/
void Digital_Data_Validation(void)
{
    uint32_t fin1;                      /* 0x90 Channel 1 (Fin1) 3-byte count */
    uint16_t step;                      /* 0x93 Channel 1 2-byte deflection */

    switch(OBC_DATA_ACT[2])             /* MSGID after STX */
    {
    case ICD_FIN_DEFLECT_CMD:           /* 0x90 Fin Deflection, MSGLEN 12 */
        /* Channel 1 only: Fin1 at MSGTXT[0..2]. Fin2/3/4 at [3..11] ignored. */
        fin1 = icd_get_u24be(&OBC_DATA_ACT[4]); /* Fin1 Deflection 3 bytes */
        icd_apply_fin1_count(fin1);     /* Home 0x800000; 0x000000 = no move Fin1 */
        if(Flag2 != 0u)                 /* ~2 ms status class */
        {
            HEALTH_PACKET_Feedback(ICD_FIN_STATUS_RSP); /* Stream 0x9A (Ch1 live) */
            Flag2 = 0u;                 /* Wait next Timer0 */
        }
        break;

    case ICD_CFG_READ_CMD:              /* 0xB0 Read Configuration */
        ACEU_CONFIG_READ(ICD_CFG_READ_CMD); /* Send 0x80 */
        OBC_DATA_ACT[2] = 0u;           /* One-shot */
        break;

    case ICD_CFG_WRITE_CMD:             /* 0xB1 Write Configuration (RAM only) */
        ACEU_CONFIG_WRITE();            /* Apply Kp/Kd/Ki; 0x81 status */
        OBC_DATA_ACT[2] = 0u;           /* One-shot */
        break;

    case ICD_FIN_POS_CMD:               /* 0xB2 Read Fin Positions */
        FIN_POS_READ_Feedback();        /* Send 0x82 */
        OBC_DATA_ACT[2] = 0u;           /* One-shot */
        break;

    case ICD_HOME_ALL_CMD:              /* 0x91 Home All */
        HOME_ALL_Feedback(ICD_HOME_ALL_CMD); /* Home Channel 1 only; send 0x61 */
        OBC_DATA_ACT[2] = 0u;           /* One-shot */
        break;

    case ICD_FUNC_TEST_CMD:             /* 0x93 Functional Test */
        /* Channel 1 only: bit 0 = Fin1. Bits 1-3 (Fin2-4) ignored. */
        if((OBC_DATA_ACT[4] & ICD_FIN1_MSK) != 0u)
        {
            step = (uint16_t)OBC_DATA_ACT[6] | ((uint16_t)OBC_DATA_ACT[7] << 8); /* Fin1 Deflection */
            Analog_Command1 = (step > 1000u) ? 1000u : (uint32_t)step; /* Channel 1 0..1000 */
            Mode = 1u;                  /* Digital test setpoint on Channel 1 */
        }
        if(Flag5 != 0u)                 /* Stream 0x63 */
        {
            STEP_SINE_TRAPEZOIDAL_START_Feedback(ICD_FUNC_TEST_CMD); /* 46-byte record */
            Flag5 = 0u;                 /* Wait next Flag5 */
        }
        break;

    case ICD_ADC_ACQ_CMD:               /* 0xAA ADC Data Acquisition */
        ADC_ACQ_Feedback();             /* Send 0xBB */
        OBC_DATA_ACT[2] = 0u;           /* One-shot */
        break;

    case 0xA5u:                         /* Firmware/flash - not updated */
    case 0xB5u:                         /* Firmware/flash - not updated */
    case 0xC5u:                         /* Firmware/flash - not updated */
        break;                          /* Leave flash operations unchanged */

    default:
        break;                          /* Unknown MSGID */
    }
}


/*****************************************************************************
 * Function : I2c_temp
 * Purpose  : Read TMP100 on I2C address 0x48. Register 0x00, 2 bytes.
 *            12-bit, 0.0625 C per LSB. NEW ICD 0x63 Temperature1.
 * Input    : none (I2C bus)
 * Output   : temperature in C as uint16_t (fraction truncated)
 *****************************************************************************/
uint16_t I2c_temp(void)
{
    uint16_t combinedBytes;             /* MSB||LSB before >>4 */
    int16_t rawTemp; /* Changed to signed type to naturally handle negative numbers */
    uint8_t msb, lsb;                   /* First and second I2C data bytes */
    float temperature_c;                /* Deg C after * 0.0625 */

    I2C_setSlaveAddress(I2CA_BASE, 0x48); /* TMP100 7-bit address */
    I2C_setDataCount(I2CA_BASE, 1);     /* Write one byte (register pointer) */
    I2C_putData(I2CA_BASE, 0x00);       /* Temperature register */
    I2C_setConfig(I2CA_BASE, I2C_MASTER_SEND_MODE); /* Master write */
    I2C_sendStartCondition(I2CA_BASE);  /* START */
    I2C_sendStopCondition(I2CA_BASE);   /* STOP after write */

    I2C_setDataCount(I2CA_BASE, 2);     /* Expect MSB then LSB */
    I2C_setConfig(I2CA_BASE, I2C_MASTER_RECEIVE_MODE); /* Master read */
    I2C_sendStartCondition(I2CA_BASE);  /* Repeated START */
    I2C_sendStopCondition(I2CA_BASE);   /* STOP after read */

    msb = I2C_getData(I2CA_BASE);       /* High byte */
    lsb = I2C_getData(I2CA_BASE);       /* Low byte */

    combinedBytes = ((uint16_t)msb << 8) | lsb; /* 16-bit left-justified temp */
    rawTemp = ((int16_t)combinedBytes) >> 4; /* 12-bit temperature */
    if (rawTemp & 0x0800)
    {
        rawTemp |= 0xF000;              /* Sign-extend bit 11 */
    }
    temperature_c = (float)rawTemp * 0.0625f; /* TMP100 LSB = 0.0625 C */
    return temperature_c; /* Returns actual degrees (e.g., 15.5f) */
}
