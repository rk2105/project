
/***************************************************** Include Files ****************************************************/
#include "SystemInit.h"                     /* ADC bases, SOC macros, channel enum names */
/***************************************************** End Declaration **************************************************/

uint32_t G_n_AdcRawData[13];            /* Unused leftover ADC array */
uint32_t AdcRawData[14];                /* ICD 16-bit samples: 0..65535 (see ReadIntADC) */

/*****************************************************************************
 * Function : IntAdcInit
 * Purpose  : Power ADCA and ADCB: clock /4, 16-bit single-ended (ICD ADC).
 *            INT pulse at end of conversion, then 10 �s wait after power-up.
 *            ICD volts ? (raw / 65535) * rail. Control uses raw/65.535 ? 0..1000.
 * Input    : none
 * Output   : Converters on (SOC map is InitAdcSocs)
 *****************************************************************************/
void IntAdcInit(void)
{
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);                               /* ADCA clock = SYSCLK / 4 */
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_4_0);                               /* ADCB clock = SYSCLK / 4 */

    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_SINGLE_ENDED);        /* 16-bit ICD sample, pin vs GND */
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_SINGLE_ENDED);        /* Same for ADCB */

    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);                /* INT flag after conversion finishes */
    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);                /* Same for ADCB */

    ADC_enableConverter(ADCA_BASE);                                             /* Analog power ON ADCA */
    ADC_enableConverter(ADCB_BASE);                                             /* Analog power ON ADCB */

    DEVICE_DELAY_US(10);                                                        /* Wait after power-up before first convert */
}

/*****************************************************************************
 * Function : InitAdcSocs
 * Purpose  : Map Start-Of-Conversion slots to pins. Software trigger only.
 *            ADCA: A0 cmd, A1 fin pos, A2 Imot, A3 +5 V, A4 +3.3 V, A5 +1.2 V
 *            (NEW ICD 0x9A / 0xBB rails). ADCB: B0 -5 V, B1 +15 V, B2 -15 V,
 *            B3 +3 V, B4 PWM sense. Sample window = 15 SYSCLK.
 *            Last ADCINT1 source written wins (ADCA = SOC5, ADCB = SOC4).
 * Input    : none
 * Output   : SOC hardware programmed
 *****************************************************************************/
void InitAdcSocs(void)
{
    const uint32_t sampleWindow = 15;                                           /* SYSCLK ticks in the sample-and-hold window */

    /* Configure SOCs of ADCA � software trigger, 15-cycle sample window */
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 15); /* A0 -> Position_command */
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN1, 15); /* A1 -> Position_Feedback (fin achieved) */
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, 15); /* A2 -> Current_Feedback (ICD Imot 1) */
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3, 15); /* A3 -> Logic_5V (ICD +5 V / POST bit 4) */
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN4, 15); /* A4 -> Logic_3V3 (ICD +3.3 V / POST bit 7) */
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN5, 15); /* A5 -> Logic_1V2 (ICD +1.2 V / POST bit 6) */

    /* Configure SOCs of ADCB */
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, sampleWindow); /* B0 -> Logic_M_5V (-5 V) */
    //    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3, 50); /* B3 -> TP24 */



    /*ME */
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN1, 15); /* B1 -> LOGIC_+15V (0x9A ADC_28v slot) */
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, 15); /* B2 -> LOGIC_-15V  */
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3, 15); /* B3 -> LOGIC_3V   */
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN4, 15); /* B4 -> LOGIC_PWM (0x9A Supply_1v1 slot) */


    /*
     *  Set SOC's to set the interrupt 1 flag. Enable the interrupt and
     *  make sure its flag is cleared. Each write re-points INT1; last one wins.
     */
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0); /* INT1 after SOC0 (overwritten below) */
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);                    /* Allow INT1 flag to set */
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);               /* Clear stale flag */

    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1); /* Re-point INT1 -> SOC1 */
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);                    /* Allow INT1 flag */
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);               /* Clear stale flag */

    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2); /* Re-point INT1 -> SOC2 */
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);                    /* Allow INT1 flag */
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);               /* Clear stale flag */

    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER3); /* Re-point INT1 -> SOC3 */
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);                    /* Allow INT1 flag */
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);               /* Clear stale flag */

    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER4); /* Re-point INT1 -> SOC4 */
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);                    /* Allow INT1 flag */
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);               /* Clear stale flag */

    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER5); /* Final: ADCA INT1 after SOC5 (A5 done) */
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);                    /* Allow INT1 flag */
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);               /* Clear stale flag */

    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0); /* ADCB INT1 after SOC0 (overwritten) */
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);                    /* Allow INT1 flag */
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);               /* Clear stale flag */

    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER3); /* ADCB INT1 after SOC3 (overwritten) */
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);                    /* Allow INT1 flag */
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);               /* Clear stale flag */

    /* ME */
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1); /* ADCB INT1 after SOC1 (overwritten) */
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);                    /* Allow INT1 flag */
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);               /* Clear stale flag */

    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2); /* ADCB INT1 after SOC2 (overwritten) */
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);                    /* Allow INT1 flag */
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);               /* Clear stale flag */

    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER4); /* Final: ADCB INT1 after SOC4 (B4 done) */
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);                    /* Allow INT1 flag */
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);               /* Clear stale flag */

}

/*****************************************************************************
 * Function : ReadIntADC
 * Purpose  : Software-force all SOCs, wait until ADCINT1 (last SOC), clear
 *            flags, copy RESULT registers into AdcRawData[].
 *            ICD: 16-bit; volts ? (raw / 65535) * rail_V. Control uses raw/65.535 ? 0..1000.
 *            Mid-scale ~32767 ? 0 V analog command (bipolar).
 * Input    : none (hardware ADC)
 * Output   : AdcRawData[] for Motor_Control, Voltage_checks (POST), ICD telemetry
 *****************************************************************************/
void ReadIntADC(void)
{
    HWREGH(ADCA_BASE + ADC_O_SOCFRC1) = (ADC_FORCE_SOC3); /* Dummy force SOC3 first (then all SOCs below) */
    HWREGH(ADCB_BASE + ADC_O_SOCFRC1) = (ADC_FORCE_SOC3); /* Dummy force ADCB SOC3 */


    HWREGH(ADCA_BASE + ADC_O_SOCFRC1) = (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2 | ADC_FORCE_SOC3 | ADC_FORCE_SOC4 | ADC_FORCE_SOC5); /* Start A0�A5 */
    HWREGH(ADCB_BASE + ADC_O_SOCFRC1) = (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2 | ADC_FORCE_SOC3 | ADC_FORCE_SOC4 ); /* Start B0�B4 */

    while((HWREGH(ADCA_BASE + ADC_O_INTFLG) & (1U << (uint16_t)ADC_INT_NUMBER1)) == 0U); /* Wait ADCA INT1 (SOC5) */
    while((HWREGH(ADCB_BASE + ADC_O_INTFLG) & (1U << (uint16_t)ADC_INT_NUMBER1)) == 0U); /* Wait ADCB INT1 (SOC4) */

    HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) = 1U << (uint16_t)ADC_INT_NUMBER1; /* Clear ADCA INT1 */
    HWREGH(ADCB_BASE + ADC_O_INTFLGCLR) = 1U << (uint16_t)ADC_INT_NUMBER1; /* Clear ADCB INT1 */

    AdcRawData[Position_command]    = HWREGH(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + ADC_SOC_NUMBER0); /* A0 analog cmd / ICD 0..65535 */
    AdcRawData[Position_Feedback]   = HWREGH(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + ADC_SOC_NUMBER1); /* A1 fin deflection achieved */
    AdcRawData[Current_Feedback]    = HWREGH(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + ADC_SOC_NUMBER2); /* A2 motor current 1 */
    AdcRawData[Logic_5V]            = HWREGH(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + ADC_SOC_NUMBER3); /* A3 +5 V */
    AdcRawData[Logic_3V3]           = HWREGH(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + ADC_SOC_NUMBER4); /* A4 +3.3 V */
    AdcRawData[Logic_1V2]           = HWREGH(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + ADC_SOC_NUMBER5); /* A5 +1.2 V */
    AdcRawData[Logic_M_5V]          = HWREGH(ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + ADC_SOC_NUMBER0); /* B0 -5 V */
    //    AdcRawData[TP24]                = HWREGH(ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + ADC_SOC_NUMBER3); /* B3 test point */

    /* ME */
    AdcRawData[LOGIC_PLUS_15]      = HWREGH(ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + ADC_SOC_NUMBER1); /* B1 +15 V */
    AdcRawData[LOGIC_MINUS_15]     = HWREGH(ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + ADC_SOC_NUMBER2); /* B2 -15 V */
    AdcRawData[LOGIC_3V]           = HWREGH(ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + ADC_SOC_NUMBER3); /* B3  3 V */
    AdcRawData[LOGIC_PWM]          = HWREGH(ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + ADC_SOC_NUMBER4); /* B4 PWM V */
}
