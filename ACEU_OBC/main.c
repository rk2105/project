/*****************************************************************************
 * File    : main.c  (ACEU_OBC - TMS320F2838x CPU1)
 * ICD     : ACEU_ICD_V1_AMS.pdf (Issue 01) - ACEU POV only
 *           Old ACUE_Single_Channel_ICD ($ 0xAA CRC16 #) is not used.
 * Purpose : After POST, run ACEU loop: RS-422 NEW ICD (STX EB90),
 *           ADC, command dispatch, motor PWM, I2C temperature.
 *           Channel 1 only (Fin1 / Motor1 / EPWM1). Fin2-4 ICD slots padded.
 *           Real-time pair is Fin Deflection 0x90 / status 0x9A.
 *           Firmware and flash MSGID 0xA5 / 0xB5 / 0xC5 are not updated.
 * NOTE    : Full change list is in ACEU_OBC_CHANGE_NOTE.txt (same folder).
 *           C2000 CPU1 22.6.1.LTS; C2000Ware 5.00; new ICD commands only.
 *****************************************************************************/

#include "SystemInit.h"             /* Types, GPIO names, NEW ICD prototypes */

/*****************************************************************************
 * Function : main
 * Purpose  : Start the ACEU application.
 *            1) SystemInit() - clocks, pins, ADC, PWM, interrupts
 *            2) If POST failed, stay in a trap and send 0x9A status
 *               (NEW ICD has no 0xE2 Health packet)
 *            3) LED ON = board is alive
 *            4) Forever: EB90 ICD command -> ADC -> Digital_Data_Validation
 *               -> Motor_Control -> I2C temperature
 * Input    : none
 * Output   : Motor GPIO/PWM; UART as in NEW ICD (0x9A / 0x80 / 0x81 / 0x63 / 0xBB)
 *****************************************************************************/
int main(void)
{
    uint8_t ACEUsysInitStatus = Failure;            /* Assume fail until SystemInit() returns Success */

    ACEUsysInitStatus = SystemInit();               /* Clocks, pins, ADC, PWM, IRQ (POST currently skipped in SystemInit) */

    while(ACEUsysInitStatus == Failure)             /* NEW ICD: no 0xE2; hold and send 0x9A rails/status */
    {
        HEALTH = 0x01;                              /* Local fail flag (not an ICD Health byte) */
        HEALTH_PACKET_Feedback(ICD_FIN_STATUS_RSP); /* Send 0x9A 74-byte status so GUI sees rails */
    }

    GPIO_writePin(LED, SET);                        /* Heartbeat LED ON after successful init */

    while(1)                                        /* Application loop forever */
    {
        RSS_422_New_Data();                         /* Copy a finished EB90 frame into OBC_DATA_ACT[] */

        ReadIntADC();                               /* 16-bit: cmd, fin pos, Imot, rails for 0x9A / 0xBB */

        Digital_Data_Validation();                  /* Dispatch NEW ICD MSGID in OBC_DATA_ACT[2] */

        Motor_Control();                            /* Channel 1: analog ADC (Mode 0) or ICD Fin1 (Mode 1) */

        Temp = I2c_temp();                          /* Board temperature for 0x63 Temperature1 */
    }
}
