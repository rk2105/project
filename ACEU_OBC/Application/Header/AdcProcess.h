#ifndef HEADER_ADCPROCESS_H_
#define HEADER_ADCPROCESS_H_

/***********************************************************************************************************************
 * Enumerations
 **********************************************************************************************************************/
/**
 * Indices into AdcRawData[]. 16-bit samples: 0..65535.
 * NEW ICD 0x9A / 0xBB use Motor current, +5 V, +3.3 V, and related rails.
 */
typedef enum
{
    Position_command,   /* A0 analog setpoint (Mode 0). Mid-scale ~32767. */
    Position_Feedback,  /* A1 fin deflection / POT 1 (0x9A / 0x82 / 0xBB). */
    Current_Feedback,   /* A2 motor current 1 (0x9A Motor1 Current / 0xBB). */
    Logic_5V,           /* A3 +5 V supply (0x9A ADC_5v / 0xBB 5 V). */
    Logic_3V3,          /* A4 +3.3 V supply (0x9A Supply_3v3_VDD). */
    Logic_1V2,          /* A5 +1.2 V supply (0x9A Supply_2p5 slot). */
    Logic_M_5V,         /* B0 -5 V analog rail (board POST). */
    TP24,               /* Spare / test point (unused). */
    LOGIC_PLUS_15,      /* B1 +15 V. Filled in 0x9A ADC_28v slot on this board. */
    LOGIC_MINUS_15,     /* B2 -15 V. Board POST only. */
    LOGIC_3V,           /* B3 +3 V. Filled in 0x9A Supply_3v3_VCC slot. */
    LOGIC_PWM           /* B4 PWM sense. Filled in 0x9A Supply_1v1 slot. */
} adc_channels;

/***********************************************************************************************************************
 * Function Prototypes
 **********************************************************************************************************************/

/*****************************************************************************
 * Function : ExtAdcInit
 * Purpose  : External ADC bring-up if a part is on the board (not used here).
 * Input    : none
 * Output   : none
 *****************************************************************************/
void ExtAdcInit(void);

/*****************************************************************************
 * Function : IntAdcInit
 * Purpose  : Power ADCA/ADCB for 16-bit samples (volts approx raw/65535 * rail).
 * Input    : none
 * Output   : converters on
 *****************************************************************************/
void IntAdcInit(void);

/*****************************************************************************
 * Function : InitAdcSocs
 * Purpose  : Map SOC slots to pins used by 0x9A status and 0xBB ADC snapshot.
 * Input    : none
 * Output   : SOC hardware programmed
 *****************************************************************************/
void InitAdcSocs(void);

/*****************************************************************************
 * Function : ReadIntADC
 * Purpose  : Software-trigger all SOCs and copy RESULT into AdcRawData[].
 *            Used by Motor_Control, Voltage_checks, 0x9A / 0x63 / 0xBB.
 * Input    : none
 * Output   : AdcRawData[]
 *****************************************************************************/
void ReadIntADC(void);

#endif /* HEADER_ADCPROCESS_H_ */
