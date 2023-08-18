/***************************************************************************//**
* \file CapSense_1_SensingCSD_LL.c
* \version 3.0
*
* \brief
*   This file defines the data structure global variables and provides
*   implementation for the low-level APIs of the CSD part of
*   the Sensing module. The file contains the APIs used for the CSD block
*   initialization, calibration, and scanning.
*
* \see CapSense_1 v3.0 Datasheet
*
*//*****************************************************************************
* Copyright (2016-2017), Cypress Semiconductor Corporation.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*******************************************************************************/
#include "cy_device_headers.h"
#include "gpio/cy_gpio.h"
#include "cyfitter_sysint_cfg.h"

#include "CapSense_1_ModClk.h"
#include "CapSense_1_Structure.h"
#include "CapSense_1_Configuration.h"
#include "CapSense_1_Sensing.h"
#include "CapSense_1_SensingCSD_LL.h"
#include "cyapicallbacks.h"

#if (CapSense_1_ENABLE == CapSense_1_CSD_EN)

/***************************************
* API Constants
***************************************/

#define CapSense_1_COARSE_INIT_WATCHDOG_TIME              (3u)
#define CapSense_1_COARSE_INIT_WATCHDOG_CYCLES_CALC       (CapSense_1_COARSE_INIT_WATCHDOG_TIME * CapSense_1_CPU_CLOCK_FREQ_MHZ)

#if(CapSense_1_COARSE_INIT_WATCHDOG_CYCLES_CALC >= CapSense_1_DELAY_EXTRACYCLES_NUM)
    #define CapSense_1_COARSE_INIT_WATCHDOG_CYCLES        (CapSense_1_COARSE_INIT_WATCHDOG_CYCLES_CALC -\
                                                                 CapSense_1_DELAY_EXTRACYCLES_NUM)
#else
    #define CapSense_1_COARSE_INIT_WATCHDOG_CYCLES        (CapSense_1_COARSE_INIT_WATCHDOG_CYCLES_CALC)
#endif /* (CapSense_1_COARSE_INIT_WATCHDOG_CYCLES_CALC >= CapSense_1_DELAY_EXTRACYCLES_NUM) */

#if (CapSense_1_ENABLE == CapSense_1_CSD_AUTO_ZERO_EN)
    #define CapSense_1_CSD_AZ_ENABLE_CFG                  (CapSense_1_CSD_HSCMP_AZ_EN_MSK)
#else
    #define CapSense_1_CSD_AZ_ENABLE_CFG                  (0uL)
#endif /* (CapSense_1_ENABLE == CapSense_1_CSD_AUTO_ZERO_EN) */

#if (CapSense_1_IDAC_SINKING == CapSense_1_CSD_IDAC_CONFIG)
    #define CapSense_1_HSCMP_SCAN_MASK                    (CapSense_1_CSD_HSCMP_HSCMP_EN_MSK | CapSense_1_CSD_AZ_ENABLE_CFG |\
                                                                 CapSense_1_CSD_HSCMP_HSCMP_INVERT_MSK)
#else
    #define CapSense_1_HSCMP_SCAN_MASK                    (CapSense_1_CSD_HSCMP_HSCMP_EN_MSK | CapSense_1_CSD_AZ_ENABLE_CFG)
#endif /* (CapSense_1_IDAC_SINKING == CapSense_1_CSD_IDAC_CONFIG) */

#define CapSense_1_HSCMP_INIT_MASK                        (CapSense_1_CSD_HSCMP_HSCMP_EN_MSK | CapSense_1_CSD_AZ_ENABLE_CFG)

#define CapSense_1_DEFAULT_SW_SHIELD_SEL                  (0x00000000uL)
#define CapSense_1_DEFAULT_CSD_SW_DSI_SEL                 (0x00000000uL)
#define CapSense_1_DEFAULT_CSD_INTR_SET                   (0x00000000uL)
#define CapSense_1_DEFAULT_SW_HS_P_SEL                    (0x00000000uL)
#define CapSense_1_DEFAULT_SW_HS_N_SEL                    (0x00000000uL)
#define CapSense_1_DEFAULT_CSD_SW_FW_TANK_SEL             (0x00000000uL)
#define CapSense_1_DEFAULT_CSD_ADC_CTL                    (0x00000000uL)
#define CapSense_1_DEFAULT_HSCMP_CFG                      (0x00000000uL)

/* CY_ID285392 */
#define CapSense_1_FILTER_DELAY_MAX                   (CapSense_1_CSD_CONFIG_FILTER_DELAY_4_CYCLES)
#define CapSense_1_EXTRA_COUNTS_MAX                   (CapSense_1_FILTER_DELAY_MAX + 5u)
#define CapSense_1_16_BIT_RESOLUTION                  (16uL)

#if (CapSense_1_IDAC_SINKING == CapSense_1_CSD_IDAC_CONFIG)
    #if(CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN)
        #define CapSense_1_DEFAULT_CSD_IO_SEL             (CapSense_1_CSD_TX_N_OUT_EN_PHI1 | CapSense_1_CSD_TX_N_AMUXA_EN_PHI2 |\
                                                                 CapSense_1_CSD_TX_OUT_EN_PHI1_DELAY | CapSense_1_CSD_TX_AMUXB_EN_PHI2_DELAY |\
                                                                 CapSense_1_CSD_TX_OUT_MSK | CapSense_1_CSD_TX_N_OUT_STATIC_CLOSE)
    #else
        #define CapSense_1_DEFAULT_CSD_IO_SEL             (CapSense_1_CSD_TX_N_OUT_EN_PHI1 | CapSense_1_CSD_TX_N_AMUXA_EN_PHI2 |\
                                                                                                         CapSense_1_CSD_TX_N_OUT_STATIC_CLOSE)
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) */
#else
    #if(CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN)
        #define CapSense_1_DEFAULT_CSD_IO_SEL             (CapSense_1_CSD_TX_N_OUT_EN_PHI1 | CapSense_1_CSD_TX_N_AMUXA_EN_PHI2 |\
                                                                 CapSense_1_CSD_TX_OUT_EN_PHI1_DELAY | CapSense_1_CSD_TX_AMUXB_EN_PHI2_DELAY)
    #else
        #define CapSense_1_DEFAULT_CSD_IO_SEL             (CapSense_1_CSD_TX_N_OUT_EN_PHI1 | CapSense_1_CSD_TX_N_AMUXA_EN_PHI2)
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) */
#endif /* (CapSense_1_IDAC_SINKING == CapSense_1_CSD_IDAC_CONFIG) */

/***************************************
* Variables
***************************************/
uint32 CapSense_1_configCsd = CapSense_1_DEFAULT_CSD_CONFIG;
/* Flag to indicate electrodes that were connected previously */
static uint8 CapSense_1_eleCsdDisconnectFlag = 0u;
#if (CapSense_1_ENABLE == CapSense_1_CSD_NOISE_METRIC_EN)
    /* Number of re-samplings when the bad conversion occurs */
    uint8 CapSense_1_badConversionsNum = CapSense_1_BAD_CONVERSIONS_NUM;
#endif /* (CapSense_1_ENABLE == CapSense_1_CSD_NOISE_METRIC_EN) */

#if (CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE)
    /* Stores IDAC and raw count that corresponds to a sensor with maximum Cp within a widget */
    uint8 CapSense_1_calibratedIdac = 1u;
    uint16 CapSense_1_calibratedRawcount = 1u;
    #if (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN)
        uint8 CapSense_1_calibratedIdacRow = 1u;
        uint16 CapSense_1_calibratedRawcountRow = 1u;
    #endif /*(CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN) */
#endif /* (CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE) */


/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/
/**
* \cond SECTION_CYSENSE_INTERNAL
* \addtogroup group_cysense_internal
* \{
*/

#if (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN)
    static void CapSense_1_SsSetShieldDelay(uint32 delay);

    #if (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_TANK_EN)
        static void CapSense_1_SsCSDEnableShieldTank(void);
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_TANK_EN) */

    #if (0u != CapSense_1_CSD_TOTAL_SHIELD_COUNT)
        static void CapSense_1_SsCSDEnableShieldElectrodes(void);
    #endif /* (0u != CapSense_1_CSD_TOTAL_SHIELD_COUNT) */

#endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) */

#if ((CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE) || \
     (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_AUTOCAL_EN))

    static cy_status CapSense_1_SsCSDCalibrateCheck(CapSense_1_RAM_WD_BASE_STRUCT const *ptrWdgt, uint32 rawLevel, uint32 rawData);

    #if (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN)
        static void CapSense_1_SsCSDSetSingleIdacMode(void);
        static void CapSense_1_SsCSDRestoreIdacMode(void);
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) */
    static cy_status CapSense_1_SsCSDCalibrateOneSensor(uint32 rawLevel,
                                                             uint8 *ptrIdac,
                                                             uint8 *iDAC8Max);
#endif /* ((CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE) || \
           (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_AUTOCAL_EN)) */

static void CapSense_1_SsCSDSetFilterDelay(void);
static void CapSense_1_SsCSDCmodPrecharge(void);
static void CapSense_1_SsCSDTriggerScan(void);
static void CapSense_1_SsCSDConfigIDACs(void);

static void CapSense_1_SsCSDSetModeSnsClockDivider(uint32 snsClkSource, uint32 snsClkDivider);

/** \}
* \endcond */


#if (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN)
    /*******************************************************************************
    * Function Name: CapSense_1_SsSetShieldDelay
    ****************************************************************************//**
    *
    * \brief
    *   This is an internal function that sets a shield delay.
    *
    * \details
    *   The function updates the CSD configuration register bits that define the shield
    *   signal delay relatively to the sense signal.
    *
    * \param delay
    *   Specifies the shield delay value:
    *                               0 - no delay
    *                               1 - 5ns delay
    *                               2 - 10ns delay
    *                               3 - 20ns delay
    *
    *******************************************************************************/
    static void CapSense_1_SsSetShieldDelay(uint32 delay)
    {
        /* Update CSD config register shield delay bits with shield delay value */
        CapSense_1_configCsd &= (uint32)(~CapSense_1_CSD_CONFIG_SHIELD_DELAY_MSK);
        CapSense_1_configCsd |= (delay << CapSense_1_CSD_CONFIG_SHIELD_DELAY_POS);
    }

    #if (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_TANK_EN)
        /*******************************************************************************
        * Function Name: CapSense_1_SsCSDEnableShieldTank
        ****************************************************************************//**
        *
        * \brief
        *   This function configures Csh pin
        *
        * \details
        *   The function updates the CSH_HSIOM register to connect Csh to AMUXBUS-B.
        *   For CSD v1 IP HW block it additionally sets Strong drive mode for Csh
        *   and applies High level to Csh pin when precharge is set to IO buffer.
        *
        *******************************************************************************/
        static void CapSense_1_SsCSDEnableShieldTank(void)
        {
            Cy_GPIO_SetDrivemode((GPIO_PRT_Type*)CapSense_1_CSD_CTANK_PORT_PTR, CapSense_1_CSD_CTANK_PIN, CY_GPIO_DM_ANALOG);
            Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)CapSense_1_CSD_CTANK_PORT_PTR, CapSense_1_CSD_CTANK_PIN,
                                                                       (en_hsiom_sel_t)CapSense_1_HSIOM_SEL_AMUXB);
        }
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_TANK_EN) */

    #if (0u != CapSense_1_CSD_TOTAL_SHIELD_COUNT)
        /*******************************************************************************
        * Function Name: CapSense_1_SsCSDEnableShieldElectrodes
        ****************************************************************************//**
        *
        * \brief
        *   This internal function initializes Shield Electrodes.
        *
        * \details
        *   The function sets the bit in the HSIOM register which enables the shield electrode
        *   functionality on the pin. The port and pin configurations are stored in
        *   the CapSense_1_shieldIoList structure.
        *
        *******************************************************************************/
        static void CapSense_1_SsCSDEnableShieldElectrodes(void)
        {
            uint32 shieldIndex;
            CapSense_1_FLASH_IO_STRUCT shieldIO;

            for (shieldIndex = 0u; shieldIndex < CapSense_1_CSD_TOTAL_SHIELD_COUNT; shieldIndex++)
            {
                shieldIO = CapSense_1_shieldIoList[shieldIndex];
                Cy_GPIO_SetDrivemode((GPIO_PRT_Type*)shieldIO.pcPtr, (uint32)shieldIO.pinNumber, CapSense_1_CSD_SHIELD_GPIO_DM);
                Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)shieldIO.pcPtr, (uint32)shieldIO.pinNumber, (en_hsiom_sel_t)CapSense_1_CSD_SHIELD_HSIOM_SEL);
            }
        }


        /*******************************************************************************
        * Function Name: CapSense_1_SsCSDDisableShieldElectrodes
        ****************************************************************************//**
        *
        * \brief
        *   This internal function disables Shield Electrodes.
        *
        * \details
        *   The function resets the bit in the HSIOM register which disables the shield
        *   electrode functionality on the pin. The port and pin configurations are
        *   stored in  the CapSense_1_shieldIoList structure.
        *
        *******************************************************************************/
        void CapSense_1_SsCSDDisableShieldElectrodes(void)
        {
            uint32 shieldIndex;
            CapSense_1_FLASH_IO_STRUCT shieldIO;

            for (shieldIndex = 0u; shieldIndex < CapSense_1_CSD_TOTAL_SHIELD_COUNT; shieldIndex++)
            {
                shieldIO = CapSense_1_shieldIoList[shieldIndex];

                Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)shieldIO.pcPtr, (uint32)shieldIO.pinNumber, (en_hsiom_sel_t)CapSense_1_HSIOM_SEL_GPIO);
                Cy_GPIO_SetDrivemode((GPIO_PRT_Type*)shieldIO.pcPtr, (uint32)shieldIO.pinNumber, CY_GPIO_DM_STRONG);
                Cy_GPIO_Clr((GPIO_PRT_Type*)shieldIO.pcPtr, (uint32)shieldIO.pinNumber);
            }
        }
    #endif /* (0u != CapSense_1_CSD_TOTAL_SHIELD_COUNT) */
#endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) */


/*******************************************************************************
* Function Name: CapSense_1_SsCSDSetModeSnsClockDivider
****************************************************************************//**
*
* \brief
*  Sets sense source and Sense Clock Divider
*
* \details
*  Updates CapSense_1_SENSE_PERIOD register with
*  sense source and Sense Clock Divider.
*
* \param snsClkSource
*  The sense source for the sense clock.
*
* \param snsClkDivider
*  The divider value for the sense clock.
*
*******************************************************************************/
static void CapSense_1_SsCSDSetModeSnsClockDivider(uint32 snsClkSource, uint32 snsClkDivider)
{
    /* Update reg value with divider and configuration */
    CY_SET_REG32(CapSense_1_CSD_SENSE_PERIOD_PTR, snsClkSource | (snsClkDivider - 1u));
}


/*******************************************************************************
* Function Name: CapSense_1_SsCSDSetFilterDelay
****************************************************************************//**
*
* \brief
*   Sets the filter delay for HW IP block.
*
* \details
*   This function updates HW configuration variable CapSense_1_configCsd
*   with the filter delay which depends on the sample clock frequency.
*   This variable is written into CONFIG register during enabling
*   to HW IP block.
*
*******************************************************************************/
static void CapSense_1_SsCSDSetFilterDelay(void)
{
    #if (CapSense_1_MOD_CSD_CLK_12MHZ < CYDEV_CLK_PERICLK__HZ)
        uint32 sampleClkFreqHz;
    #endif /* (CapSense_1_MOD_CSD_CLK_12MHZ < CYDEV_CLK_PERICLK__HZ) */

    CapSense_1_configCsd &= ~CapSense_1_CSD_CONFIG_FILTER_DELAY_MSK;

    #if (CapSense_1_MOD_CSD_CLK_12MHZ < CYDEV_CLK_PERICLK__HZ)
        sampleClkFreqHz = CYDEV_CLK_PERICLK__HZ / (uint32)CapSense_1_dsRam.modCsdClk;
        if(sampleClkFreqHz <= CapSense_1_MOD_CSD_CLK_12MHZ)
        {
            CapSense_1_configCsd |= CapSense_1_CSD_CONFIG_FILTER_DELAY_12MHZ;
        }
        else if(sampleClkFreqHz <= CapSense_1_MOD_CSD_CLK_24MHZ)
        {
            CapSense_1_configCsd |= CapSense_1_CSD_CONFIG_FILTER_DELAY_24MHZ;
        }
        else
        {
            CapSense_1_configCsd |= CapSense_1_CSD_CONFIG_FILTER_DELAY_48MHZ;
        }
    #else
        CapSense_1_configCsd |= CapSense_1_CSD_CONFIG_FILTER_DELAY_12MHZ;
    #endif /* (CapSense_1_MOD_CSD_CLK_12MHZ < CYDEV_CLK_PERICLK__HZ) */
}


/*******************************************************************************
* Function Name: CapSense_1_SsCSDConfigIDACs
****************************************************************************//**
*
* \brief
*   Configures the mode for IDAC registers
*
* \details
*   This function configures the IDAC modes depend on Compensation IDAC
*   enabled or disabled.
*
*******************************************************************************/
static void CapSense_1_SsCSDConfigIDACs(void)
{
    CY_SET_REG32(CapSense_1_CSD_IDACA_PTR, CapSense_1_IDAC_MOD_DEFAULT_CFG);
    #if ((CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) && \
         (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN))
        CY_SET_REG32(CapSense_1_CSD_IDACB_PTR, CapSense_1_IDAC_COMP_DEFAULT_CFG);
    #endif /* ((CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) && \
               (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN)) */
}


/*******************************************************************************
* Function Name: CapSense_1_SsCSDInitialize
****************************************************************************//**
*
* \brief
*   This API initializes the CSD module.
*
* \details
*   The function performs the following steps:
*   1) Sets GPIO output to "0" for all sensor pins;
*   2) Connects CMOD to AMUXBUS-A and to CSDBUS-A;
*   3) Connects CMOD to (sense path) to CSDCOMP;
*   4) Connects Csh_tank to AMUXBUS-B and to CSDBUS-B;
*   5) Connects VREF to REFGEN;
*   6) Configures REFGEN and sets the reference voltage;
*   7) Connects VREF to CSDCOMP and HSCOMP;
*   8) Configures IDAC and connect to CSDBUS-A (to drive CMOD);
*   9) Configures ModClk;
*   10) Configure SnsClk source;
*   11) Sets other CSD configurations (Csd Auto Zero time,
*       Sample Init period, interrupts,
*       CMOD and Csh_tank/shield initialization switch resistance).
*
*******************************************************************************/
void CapSense_1_SsCSDInitialize(void)
{
    uint32 newRegValue;

    /* Set all the sensors to inactive state */
    CapSense_1_SsClearCSDSensors();
    CapSense_1_BistDischargeExtCapacitors();

    #if (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN)
        /* Connect shields to AMUX-B bus (config HSIOM regs) */
        #if (0u != CapSense_1_CSD_TOTAL_SHIELD_COUNT)
            CapSense_1_SsCSDEnableShieldElectrodes();
        #endif /* (0u != CapSense_1_CSD_TOTAL_SHIELD_COUNT) */

        #if (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_TANK_EN)
            /* Configure Csh */
            CapSense_1_SsCSDEnableShieldTank();
        #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_TANK_EN) */
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) */

    /* Initialize the unused CSD IP registers to defaut state */
    CY_SET_REG32(CapSense_1_CSD_SENSE_DUTY_PTR,       CapSense_1_DEFAULT_SENSE_DUTY_SEL);

    /* Configure VREF */
    newRegValue = CY_GET_REG32(CapSense_1_CSD_SW_REFGEN_SEL_PTR);
    newRegValue |= CapSense_1_DEFAULT_SW_REFGEN_SEL;
    CY_SET_REG32(CapSense_1_CSD_SW_REFGEN_SEL_PTR, newRegValue);

    CY_SET_REG32(CapSense_1_CSD_INTR_SET_PTR,         CapSense_1_DEFAULT_CSD_INTR_SET);
    CY_SET_REG32(CapSense_1_CSD_SW_FW_TANK_SEL_PTR,   CapSense_1_DEFAULT_CSD_SW_FW_TANK_SEL);
    CY_SET_REG32(CapSense_1_CSD_SW_DSI_SEL_PTR,       CapSense_1_DEFAULT_CSD_SW_DSI_SEL);
    CY_SET_REG32(CapSense_1_CSD_ADC_CTL_PTR,          CapSense_1_DEFAULT_CSD_ADC_CTL);
    CY_SET_REG32(CapSense_1_CSD_AMBUF_PTR,            CapSense_1_CSD_AMBUF_PWR_MODE_OFF);
    CY_SET_REG32(CapSense_1_CSD_SW_SHIELD_SEL_PTR,    CapSense_1_DEFAULT_SW_SHIELD_SEL);
    CY_SET_REG32(CapSense_1_CSD_SW_HS_P_SEL_PTR,      CapSense_1_DEFAULT_SW_HS_P_SEL);
    CY_SET_REG32(CapSense_1_CSD_SW_HS_N_SEL_PTR,      CapSense_1_DEFAULT_SW_HS_N_SEL);
    CY_SET_REG32(CapSense_1_CSD_HSCMP_PTR,            CapSense_1_DEFAULT_HSCMP_CFG);
    CY_SET_REG32(CapSense_1_CSD_IO_SEL_PTR,           CapSense_1_DEFAULT_CSD_IO_SEL);

    /* Connect CMOD to AMUXBUS-A */
    Cy_GPIO_SetDrivemode((GPIO_PRT_Type*)CapSense_1_CSD_CMOD_PORT_PTR, CapSense_1_CSD_CMOD_PIN, CY_GPIO_DM_ANALOG);
    Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)CapSense_1_CSD_CMOD_PORT_PTR, CapSense_1_CSD_CMOD_PIN,
                                                                       (en_hsiom_sel_t)CapSense_1_CSD_CMOD_HSIOM_SEL);

    /* Connect AMUXBUS-A to CSDBUS-A */
    newRegValue = CY_GET_REG32(CapSense_1_CSD_SW_BYP_SEL_PTR);
    newRegValue |= CapSense_1_CSD_SW_BYP_SEL_SW_BYA_MSK;
    CY_SET_REG32(CapSense_1_CSD_SW_BYP_SEL_PTR, newRegValue);

    /* Connect CMOD to (sense path) to CSDCOMP */
    #if (CapSense_1_CSD__CMOD_PAD == CapSense_1_CMOD_CONNECTION)
        CY_SET_REG32(CapSense_1_CSD_SW_CMP_P_SEL_PTR, CapSense_1_CSD_SW_CMP_P_SEL_SW_SFPM_STATIC_CLOSE);
    #elif (CapSense_1_CSD__CSHIELD_PAD == CapSense_1_CMOD_CONNECTION)
        CY_SET_REG32(CapSense_1_CSD_SW_CMP_P_SEL_PTR, CapSense_1_CSD_SW_CMP_P_SEL_SW_SFPS_STATIC_CLOSE);
    #else
        CY_SET_REG32(CapSense_1_CSD_SW_CMP_P_SEL_PTR, CapSense_1_CSD_SW_CMP_P_SEL_SW_SFPT_STATIC_CLOSE);
    #endif /* (CapSense_1_CSD__CMOD_PAD == CapSense_1_CMOD_CONNECTION) */

    /* Configure shield driving path */
    #if (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN)
        /* Connect AMUXBUS-B to CSDBUS-B (and AMUXBUS-A to CSDBUS-A ) */
        CY_SET_REG32(CapSense_1_CSD_SW_BYP_SEL_PTR, CapSense_1_CSD_SW_BYP_SEL_SW_BYA_MSK | CapSense_1_CSD_SW_BYP_SEL_SW_BYB_MSK);

        /* Connect AMUXBUS-B to HSCMP positive input */
        CY_SET_REG32(CapSense_1_CSD_SW_HS_P_SEL_PTR, 0x00000000uL);
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) */

    /* Configure VREF */
    #if (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) && (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN)
        /* Connect VREF to REFGEN. Connect IDACB to CSDBUSA */
        newRegValue = CY_GET_REG32(CapSense_1_CSD_SW_REFGEN_SEL_PTR);
        newRegValue |= CapSense_1_CSD_SW_REFGEN_VREF_SRC | CapSense_1_CSD_SW_REFGEN_SEL_SW_IAIB_MSK;
        CY_SET_REG32(CapSense_1_CSD_SW_REFGEN_SEL_PTR, newRegValue);
    #else
        /* Connect VREF to REFGEN (IAIB switch is open) */
        newRegValue = CY_GET_REG32(CapSense_1_CSD_SW_REFGEN_SEL_PTR);
        newRegValue |= CapSense_1_CSD_SW_REFGEN_VREF_SRC;
        CY_SET_REG32(CapSense_1_CSD_SW_REFGEN_SEL_PTR, newRegValue);
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) && (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN */

    #if (CapSense_1_2000_MV > CapSense_1_CYDEV_VDDA_MV)

        /* Connect VREFLOW (from AMBUF and RefGen is in bypass mode) to CSDCOMP when Vdda < 2 V */
        CY_SET_REG32(CapSense_1_CSD_SW_CMP_N_SEL_PTR, CapSense_1_CSD_SW_CMP_N_SEL_SW_SCRH_STATIC_CLOSE);

        /* Connect Vrefhi to AMUBUF positive input when Vdaa < 2V
         *  Connect AMUBUF to SCDCMP negative input when Vdaa < 2V
         */
        #if (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN)
            #if (CapSense_1_IDAC_SINKING == CapSense_1_CSD_IDAC_CONFIG)
                CY_SET_REG32(CapSense_1_CSD_SW_AMUXBUF_SEL_PTR, CapSense_1_CSD_SW_AMUXBUF_SEL_SW_IRH_STATIC_CLOSE |
                                                                      CapSense_1_CSD_SW_AMUXBUF_SEL_SW_ICB_PHI2);
            #else
                CY_SET_REG32(CapSense_1_CSD_SW_AMUXBUF_SEL_PTR, CapSense_1_CSD_SW_AMUXBUF_SEL_SW_DEFAULT);
            #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) */

            CY_SET_REG32(CapSense_1_CSD_REFGEN_PTR, CapSense_1_CSD_REFGEN_WITH_SHIELD_CFG);
            CY_SET_REG32(CapSense_1_CSD_AMBUF_PTR, CapSense_1_CSD_AMBUF_PWR_MODE_NORM);
        #else
            CY_SET_REG32(CapSense_1_CSD_REFGEN_PTR, CapSense_1_CSD_REFGEN_NO_SHIELD_CFG);
            CY_SET_REG32(CapSense_1_CSD_AMBUF_PTR, CapSense_1_CSD_AMBUF_PWR_MODE_OFF);
        #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) */

        /* Connect VREFHI to HSCOMP */
        CY_SET_REG32(CapSense_1_CSD_SW_HS_N_SEL_PTR, CapSense_1_CSD_SW_HS_N_SEL_SW_HCRH_STATIC_CLOSE);
    #else

        /* Connect VREFHI (from RefGen) to CSDCOMP when Vdda >= 2 V */
        CY_SET_REG32(CapSense_1_CSD_SW_CMP_N_SEL_PTR, CapSense_1_CSD_SW_CMP_N_SEL_SW_SCRH_STATIC_CLOSE);

        #if (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN)

            /* Configure REFGEN. Set reference voltage when Vdda >= 2 V */
            CY_SET_REG32(CapSense_1_CSD_REFGEN_PTR, CapSense_1_CSD_REFGEN_WITH_SHIELD_CFG);

            /* Turn on CSD_AMBUF high power level when Vdaa >= 2V */
            CY_SET_REG32(CapSense_1_CSD_AMBUF_PTR, CapSense_1_CSD_AMBUF_PWR_MODE_HI);

            #if (CapSense_1_IDAC_SINKING != CapSense_1_CSD_IDAC_CONFIG)
                CY_SET_REG32(CapSense_1_CSD_SW_AMUXBUF_SEL_PTR, CapSense_1_CSD_SW_AMUXBUF_SEL_SW_IRH_STATIC_CLOSE |
                                                                      CapSense_1_CSD_SW_AMUXBUF_SEL_SW_ICB_PHI2);
            #else
                CY_SET_REG32(CapSense_1_CSD_SW_AMUXBUF_SEL_PTR, CapSense_1_CSD_SW_AMUXBUF_SEL_SW_DEFAULT);
            #endif /* (CapSense_1_IDAC_SINKING == CapSense_1_CSD_IDAC_CONFIG) */
        #else

            /* Configure REFGEN. Set reference voltage when Vdda >= 2 V */
            CY_SET_REG32(CapSense_1_CSD_REFGEN_PTR, CapSense_1_CSD_REFGEN_NO_SHIELD_CFG);
            CY_SET_REG32(CapSense_1_CSD_SW_AMUXBUF_SEL_PTR, CapSense_1_CSD_SW_AMUXBUF_SEL_SW_DEFAULT);
        #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) */

        /* Connect VREFHI to HSCOMP */
        CY_SET_REG32(CapSense_1_CSD_SW_HS_N_SEL_PTR, CapSense_1_CSD_SW_HS_N_SEL_SW_HCRH_STATIC_CLOSE);

    #endif /* (CapSense_1_2000_MV > CapSense_1_CYDEV_VDDA_MV) */

    /* Configure IDACs mode */
    CapSense_1_SsCSDConfigIDACs();

    /* Configure ModClk */
    CapSense_1_ModClk_SetDivider((uint32)CapSense_1_dsRam.modCsdClk - 1uL);

    /* Set other CSD configurations */
    #if (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN)
        CapSense_1_SsSetShieldDelay(CapSense_1_CSD_SHIELD_DELAY);
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) */

    /* Configure HW IP block filter delay */
    CapSense_1_SsCSDSetFilterDelay();

    #if (CapSense_1_DISABLE == CapSense_1_BLOCK_OFF_AFTER_SCAN_EN)
        /* Enable power to sub-blocks */
        CY_SET_REG32(CapSense_1_CSD_CONFIG_PTR, CapSense_1_configCsd |
                                                  CapSense_1_CSD_CONFIG_SENSE_EN_MSK |
                                                  CapSense_1_CSD_CONFIG_ENABLE_MSK);
    #endif /* (CapSense_1_ENABLE == CapSense_1_BLOCK_OFF_AFTER_SCAN_EN) */

    /* Set Csd Auto Zero time (set AZ_TIME bitmask) */
    CY_SET_REG32(CapSense_1_CSD_SEQ_TIME_PTR, CapSense_1_CSD_AUTO_ZERO_TIME);

    /* Select CMOD and Csh_tank/shield initialization switch resistance */
    CY_SET_REG32(CapSense_1_CSD_SW_RES_PTR, 0x00000000uL);

    /* Set the number of dummy fine initialization cycles */
    CY_SET_REG32(CapSense_1_CSD_SEQ_INIT_CNT_PTR, CapSense_1_CSD_FINE_INIT_TIME);

    #if ((CapSense_1_ENABLE == CapSense_1_CSD_COMMON_SNS_CLK_EN) &&\
         (CapSense_1_CLK_SOURCE_DIRECT == CapSense_1_CSD_SNS_CLK_SOURCE))
         /* Set clock dividers and clock source mode */
        CapSense_1_SsCSDConfigClock();
    #endif /* ((CapSense_1_ENABLE == CapSense_1_CSD_COMMON_SNS_CLK_EN) &&\
               (CapSense_1_CLK_SOURCE_DIRECT == CapSense_1_CSD_SNS_CLK_SOURCE)) */
}


/*******************************************************************************
* Function Name: CapSense_1_SsCSDElectrodeCheck
****************************************************************************//**
*
* \brief
*   Checks if electrodes were previously connected using
 * CapSense_1_CSDSetupWidgetExt() API and if yes disconnects them.
*
* \details
*   This function checks if CapSense_1_eleCsdDisconnectFlag is set.
*   If it set, the function disconnects the previously connected electrode.
*   The previous IO is contained in CapSense_1_curSnsIOPtr and
*   CapSense_1_curFlashSnsPtr contains the previous data for Ganged
*   sensors.
*
*******************************************************************************/
void CapSense_1_SsCSDElectrodeCheck(void)
{
    #if (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN)
        uint32 tempVal;
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN) */

    if (CapSense_1_ENABLE == CapSense_1_eleCsdDisconnectFlag)
    {
        /* Disconnect if electrodes were previous connected by CSDSetupWidgetExt() API */
        #if (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN)
            /* Check ganged sns flag */
            if (CapSense_1_GANGED_SNS_MASK == (CapSense_1_curFlashWdgtPtr->staticConfig & CapSense_1_GANGED_SNS_MASK))
            {
                /* Get number of ganged pins */
                tempVal = CapSense_1_curFlashSnsPtr->numPins;

                /* Get IO pointer */
                CapSense_1_curSnsIOPtr = &CapSense_1_ioList[CapSense_1_curFlashSnsPtr->firstPinId];

                /* Disconnect all ganged sensors */
                do
                {
                    CapSense_1_CSDDisconnectSns(CapSense_1_curSnsIOPtr);
                    CapSense_1_curSnsIOPtr++;
                    tempVal--;
                } while (0u != tempVal);
            }
            else
            {
                /* Disconnect ganged sensor */
                CapSense_1_CSDDisconnectSns(CapSense_1_curSnsIOPtr);
            }
        #else
            /* Disable sensor */
            CapSense_1_CSDDisconnectSns(CapSense_1_curSnsIOPtr);
        #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN) */

        CapSense_1_eleCsdDisconnectFlag = 0u;
    }
}


/*******************************************************************************
* Function Name: CapSense_1_SsCSDSetUpIdacs
****************************************************************************//**
*
* \brief
*  This internal function changes the IDACs values.
*
* \details
*  If Compensation IDAC is enabled, it updates two IDACs.
*  The Modulator IDAC is common for all the sensors of the widget.
*  The Compensation IDAC is updated for the sensor of the widget which
*  is available in the RAM_SNS_STRUCT structure.
*  If the Compensation IDAC is disabled, the function updates the Modulator IDAC
*  value only in the RAM_WD_BASE_STRUCT structure.
*
* \param
*  ptrWdgt The pointer to the RAM_WD_BASE_STRUCT structure.
*
*******************************************************************************/
void CapSense_1_SsCSDSetUpIdacs(CapSense_1_RAM_WD_BASE_STRUCT const *ptrWdgt)
{

    uint32 interruptState;

    uint32 idacARegValue;
    #if ((CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) && \
         (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN))
        uint32 idacBRegValue;
    #endif /* ((CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) && \
               (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN)) */

    interruptState = Cy_SysLib_EnterCriticalSection();

    /* Get IDACA Value */
    idacARegValue = CY_GET_REG32(CapSense_1_CSD_IDACA_PTR);

    /* Set IDACA Value mask */
    idacARegValue &= ~(CapSense_1_CSD_IDACA_VAL_MSK);

    /* Set IDACA value */
    #if (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN)
        if (CapSense_1_dsFlash.wdgtArray[(CapSense_1_widgetIndex)].numCols <= CapSense_1_sensorIndex)
        {
            idacARegValue |= (uint32)ptrWdgt->rowIdacMod[CapSense_1_scanFreqIndex];
        }
        else
        {
            idacARegValue |= (uint32)ptrWdgt->idacMod[CapSense_1_scanFreqIndex];
        }
    #else
        idacARegValue |= (uint32)ptrWdgt->idacMod[CapSense_1_scanFreqIndex];
    #endif /* (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN) */

    /* Update IDACA register with new value */
    CY_SET_REG32(CapSense_1_CSD_IDACA_PTR, idacARegValue);
    Cy_SysLib_ExitCriticalSection(interruptState);

    #if ((CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) && \
         (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN))
        interruptState = Cy_SysLib_EnterCriticalSection();
        /* Get IDACB Value */
        idacBRegValue = CY_GET_REG32(CapSense_1_CSD_IDACB_PTR);

        /* Set IDACB Value mask */
        idacBRegValue &= ~(CapSense_1_CSD_IDACB_VAL_MSK);

        /* Set IDACB value */
        idacBRegValue |= (uint32)CapSense_1_curRamSnsPtr->idacComp[CapSense_1_scanFreqIndex];

        /* Update IDACA register with new value */
        CY_SET_REG32(CapSense_1_CSD_IDACB_PTR, idacBRegValue);
        Cy_SysLib_ExitCriticalSection(interruptState);
    #endif /* ((CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) && \
               (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN)) */
}


/*******************************************************************************
* Function Name: CapSense_1_SsCSDGetNumberOfConversions
****************************************************************************//**
*
* \brief
*  This function gets Number of conversions.
*
* \details
*  This function gets Number of conversions using following equation:
*   conversionsNum = (2^resolution - 1) / snsClkDivider.
*
* \param
*  snsClkDivider The divider value for the sense clock.
*  resolution The widget resolution.
*  snsClkSrc The current Sense Clock Source.
*
* \return Returns the Number of conversions.
*
*******************************************************************************/
uint32 CapSense_1_SsCSDGetNumberOfConversions(uint32 snsClkDivider, uint32 resolution, uint32 snsClkSrc)
{
    uint32 divider;
    uint32 conversionsNum;

    (void)snsClkSrc;

    /* Calculate scanning resolution value in register */
    conversionsNum = (uint32)((1uL << resolution) - 1uL);

    #if(CapSense_1_SENSING_LOW_EMI == CapSense_1_CSD_SENSING_METHOD)
        divider = snsClkDivider - (snsClkDivider / 5uL) - 1uL;
    #else
        divider = snsClkDivider;
    #endif /* (CapSense_1_SENSING_LOW_EMI == CapSense_1_CSD_SENSING_METHOD) */

    /* CY_ID285392 */
    if (0u < divider)
    {
        if(resolution >= CapSense_1_16_BIT_RESOLUTION)
        {
            conversionsNum = (conversionsNum - CapSense_1_EXTRA_COUNTS_MAX) / divider;
        }
        else
        {
            conversionsNum /= divider;
        }
    }

    #if((CapSense_1_CLK_SOURCE_PRS8  == CapSense_1_CSD_SNS_CLK_SOURCE) ||\
        (CapSense_1_CLK_SOURCE_PRS12 == CapSense_1_CSD_SNS_CLK_SOURCE) ||\
        (CapSense_1_CLK_SOURCE_PRSAUTO == CapSense_1_CSD_SNS_CLK_SOURCE))
        switch (snsClkSrc)
        {
            case CapSense_1_CLK_SOURCE_PRS8:
            case CapSense_1_CLK_SOURCE_PRS12:
                /* Divide by 2 for PRS8/PRS12 mode */
                conversionsNum >>= 1u;
                break;

            default:
                break;
        }
    #endif /* ((CapSense_1_CLK_SOURCE_PRS8  == CapSense_1_CSD_SNS_CLK_SOURCE) ||\
               (CapSense_1_CLK_SOURCE_PRS12 == CapSense_1_CSD_SNS_CLK_SOURCE) ||\
               (CapSense_1_CLK_SOURCE_PRSAUTO == CapSense_1_CSD_SNS_CLK_SOURCE)) */

    return((conversionsNum > 0uL) ? (conversionsNum) : 1uL);
}


/*******************************************************************************
* Function Name: CapSense_1_SsCSDConfigClock
****************************************************************************//**
*
* \brief
*  This function configure sense clock for different modes.
*
* \details
*  Function sets the clock divider and configures the mode based on configuration.
*
*******************************************************************************/
void CapSense_1_SsCSDConfigClock(void)
{
    uint32 snsClkDivider;
    uint32 snsClkSrc;
    uint32 newRegValue;

    CapSense_1_RAM_WD_BASE_STRUCT const *ptrWdgt = (CapSense_1_RAM_WD_BASE_STRUCT *)
             CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].ptr2WdgtRam;

    /* Get sense divider based on configuration */
    #if (CapSense_1_ENABLE == CapSense_1_CSD_COMMON_SNS_CLK_EN)
        snsClkDivider = (uint32)CapSense_1_dsRam.snsCsdClk;
    #else
        #if (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN)
            /* Get SnsClck divider for rows or columns */
            if (CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].numCols <= CapSense_1_sensorIndex)
            {
                snsClkDivider = (uint32)(ptrWdgt->rowSnsClk);
            }
            else
            {
                snsClkDivider = (uint32)(ptrWdgt->snsClk);
            }
        #else
            snsClkDivider = (uint32)(ptrWdgt->snsClk);
        #endif /* (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN) */
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_COMMON_SNS_CLK_EN) */

    /* Get sense clk source calculated in CapSense_1_SsCSDInitialize() before */
    #if (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN)
        /* Get SnsClc Source for rows or columns */
        if (CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].numCols <= CapSense_1_sensorIndex)
        {
            snsClkSrc = (uint32)ptrWdgt->rowSnsClkSource;
        }
        else
        {
            snsClkSrc = (uint32)ptrWdgt->snsClkSource;
        }
    #else
        snsClkSrc = (uint32)ptrWdgt->snsClkSource;
    #endif /* (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN) */

    newRegValue = (snsClkSrc << CapSense_1_CSD_SENSE_PERIOD_LFSR_SIZE_POS);

    /* Configuring PRS SEL_BIT */
    if ((CapSense_1_CLK_SOURCE_PRS8 == snsClkSrc) ||
        (CapSense_1_CLK_SOURCE_PRS12 == snsClkSrc))
    {
        newRegValue |= CapSense_1_CSD_SENSE_PERIOD_SEL_LFSR_MSB_MSK;
    }

    #if((CapSense_1_CLK_SOURCE_PRS8  == CapSense_1_CSD_SNS_CLK_SOURCE) ||\
        (CapSense_1_CLK_SOURCE_PRS12 == CapSense_1_CSD_SNS_CLK_SOURCE) ||\
        (CapSense_1_CLK_SOURCE_PRSAUTO == CapSense_1_CSD_SNS_CLK_SOURCE))
        switch (snsClkSrc)
        {
        case CapSense_1_CLK_SOURCE_PRS8:
        case CapSense_1_CLK_SOURCE_PRS12:
            /* Divide by 2 for PRS8/PRS12 mode */
            snsClkDivider >>= 1;
            if (snsClkDivider == 0u)
            {
                snsClkDivider = 1u;
            }
            break;

        default:
            break;
        }
    #endif /* ((CapSense_1_CLK_SOURCE_PRS8  == CapSense_1_CSD_SNS_CLK_SOURCE) ||\
               (CapSense_1_CLK_SOURCE_PRS12 == CapSense_1_CSD_SNS_CLK_SOURCE) ||\
               (CapSense_1_CLK_SOURCE_PRSAUTO == CapSense_1_CSD_SNS_CLK_SOURCE)) */

    CapSense_1_SsCSDSetModeSnsClockDivider(newRegValue, snsClkDivider);

    #if(CapSense_1_SENSING_LOW_EMI == CapSense_1_CSD_SENSING_METHOD)
        /* Calculate duration of the  first phase of the sense clock to have 80:20 duty cycle. */
        newRegValue = snsClkDivider / 5uL;
        /* Ensure at least 1 clk_csd cycle duration of the first phase of the sense clock. */
        newRegValue = (newRegValue > 0uL) ? newRegValue : 1uL;
        CapSense_1_SsSetSnsFirstPhaseWidth(newRegValue);
    #endif /* (CapSense_1_SENSING_LOW_EMI == CapSense_1_CSD_SENSING_METHOD) */
}


/*******************************************************************************
* Function Name: CapSense_1_SsCSDCalculateScanDuration
****************************************************************************//**
*
* \brief
*   Calculates Scan Duration which is defined by scan resolution
*
* \details
*   The function calculates the number of conversions and updates
*   SEQ_NORM_CNT register. The number of conversions depends on resolution and
*   snsClk divider.
*
* \param ptrWdgt
*  The pointer to the RAM_WD_BASE_STRUCT structure.
*
*******************************************************************************/
void CapSense_1_SsCSDCalculateScanDuration(CapSense_1_RAM_WD_BASE_STRUCT const *ptrWdgt)
{
    uint32 conversionsNum;
    uint32 snsClkDivider;

    /* Get Number Of Conversions */
    #if (CapSense_1_ENABLE == CapSense_1_CSD_COMMON_SNS_CLK_EN)
        snsClkDivider = CapSense_1_dsRam.snsCsdClk;
    #else
        #if (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN)
            /* Get SnsClck divider for rows or columns */
            if (CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].numCols <= CapSense_1_sensorIndex)
            {
                snsClkDivider = (uint32)(ptrWdgt->rowSnsClk);
            }
            else
            {
                snsClkDivider = (uint32)(ptrWdgt->snsClk);
            }
        #else
            snsClkDivider = (uint32)(ptrWdgt->snsClk);
        #endif /* (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN) */
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_COMMON_SNS_CLK_EN) */

    conversionsNum = CapSense_1_SsCSDGetNumberOfConversions(snsClkDivider, (uint32)ptrWdgt->resolution, (uint32)ptrWdgt->snsClkSource);

    /* Set Number Of Conversions based on scanning resolution */
    CY_SET_REG32(CapSense_1_CSD_SEQ_NORM_CNT_PTR, (conversionsNum & CapSense_1_CSD_SEQ_NORM_CNT_CONV_CNT_MSK));
}


/*******************************************************************************
* Function Name: CapSense_1_CSDSetupWidget
****************************************************************************//**
*
* \brief
*  Performs hardware and firmware initialization required for scanning sensors
*  in a specific widget using the CSD sensing method. This function requires using
*  the CapSense_1_CSDScan() function to start scanning.
*
* \details
*  This function initializes the specific widget common parameters to perform
*  the CSD scanning. The initialization includes setting up a Modulator and
*  Sense clock frequency and scanning resolution.
*
*  This function does not connect any specific sensors to the scanning hardware,
*  neither does it start a scanning process. The CapSense_1_CSDScan()
*  API must be called after initializing the widget to start scanning.
*
*  This function is called when no scanning is in progress. I.e.
*  CapSense_1_IsBusy() returns a non-busy status.
*
*  This function is called by the CapSense_1_SetupWidget() API if the
*  given widget uses the CSD sensing method.
*
*  Calling this function directly from the application layer is not
*  recommended. This function is used to implement only the user's specific
*  use cases (for faster execution time or pipeline scanning for example).
*
* \param  widgetId
*  Specifies the ID number of the widget to perform hardware and firmware
*  initialization required for scanning sensors in the specific widget.
*  A macro for the widget ID can be found in the CapSense_1 Configuration header
*  file defined as CapSense_1_<WidgetName>_WDGT_ID.
*
*******************************************************************************/
void CapSense_1_CSDSetupWidget(uint32 widgetId)
{

    CapSense_1_RAM_WD_BASE_STRUCT const *ptrWdgt = (CapSense_1_RAM_WD_BASE_STRUCT *)
                                                    CapSense_1_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;

    /* Save widget Id to have assess to it after scanning  */
    CapSense_1_widgetIndex = (uint8)widgetId;

    #if ((CapSense_1_ENABLE == CapSense_1_CSD_CSX_EN) || \
        (CapSense_1_ENABLE == CapSense_1_SELF_TEST_EN) || \
        (CapSense_1_ENABLE == CapSense_1_ADC_EN))
         CapSense_1_SsSwitchSensingMode(CapSense_1_SENSE_METHOD_CSD_E);
    #endif /* ((CapSense_1_ENABLE == CapSense_1_CSD_CSX_EN) || \
               (CapSense_1_ENABLE == CapSense_1_SELF_TEST_EN) || \
               (CapSense_1_ENABLE == CapSense_1_ADC_EN)) */

    #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
        /* Reset the frequency scan channel if enabled */
        CapSense_1_scanFreqIndex = CapSense_1_FREQ_CHANNEL_0;
    #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */

    /* Disconnect previous electrode if it has been connected */
    CapSense_1_SsCSDElectrodeCheck();

    /* Initialize IO Sns electrode structure pointer to current widget */
    CapSense_1_curSnsIOPtr = (CapSense_1_FLASH_IO_STRUCT const *)
                                    CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].ptr2SnsFlash;

    /* Update status register in Data structure */
    CapSense_1_dsRam.status &= ~CapSense_1_STATUS_WDGT0_MASK;
    CapSense_1_dsRam.status |= CapSense_1_widgetIndex;

    /* Set up scanning resolution (Number of conversion) */
    CapSense_1_SsCSDCalculateScanDuration(ptrWdgt);

    #if (((CapSense_1_DISABLE == CapSense_1_CSD_COMMON_SNS_CLK_EN) || \
          (CapSense_1_CLK_SOURCE_DIRECT != CapSense_1_CSD_SNS_CLK_SOURCE)) && \
         ((!(CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN)) || \
          (CapSense_1_ENABLE == CapSense_1_CSD_COMMON_SNS_CLK_EN)))
        CapSense_1_SsCSDConfigClock();
    #endif /* ((CapSense_1_DISABLE == CapSense_1_CSD_COMMON_SNS_CLK_EN) ||\
               (CapSense_1_CLK_SOURCE_DIRECT != CapSense_1_CSD_SNS_CLK_SOURCE)) */
}


/*******************************************************************************
* Function Name: CapSense_1_CSDSetupWidgetExt
****************************************************************************//**
*
* \brief
*  Performs extended initialization for the CSD widget and also performs
*  initialization required for a specific sensor in the widget. This function
*  requires using the CapSense_1_CSDScanExt() function to initiate a scan.
*
* \details
*  This function does the same as CapSense_1_CSDSetupWidget() and also
*  does the following tasks:
*    1. Connects the first sensor of the widget.
*    2. Configures the IDAC value.
*    3. Initializes an interrupt callback function to initialize a scan of the
*  next sensors in a widget.
*
*  Once this function is called to initialize a widget and a sensor, the
*  CapSense_1_CSDScanExt() function is called to scan the sensor.
*
*  This function is called when no scanning is in progress. I.e.
*  CapSense_1_IsBusy() returns a non-busy status.
*
*  Calling this function directly from the application layer is not
*  recommended. This function is used to implement only the user's specific
*  use cases (for faster execution time or pipeline scanning for example).
*
* \param widgetId
*  Specifies the ID number of the widget to perform hardware and firmware
*  initialization required for scanning the specific sensor in the specific
*  widget.
*  A macro for the widget ID can be found in the CapSense_1 Configuration header
*  file defined as CapSense_1_<WidgetName>_WDGT_ID.
*
* \param sensorId
*  Specifies the ID number of the sensor within the widget to perform hardware
*  and firmware initialization required for scanning a specific sensor in a
*  specific widget.
*  A macro for the sensor ID within a specified widget can be found in the
*  CapSense_1 Configuration header file defined as
*  CapSense_1_<WidgetName>_SNS<SensorNumber>_ID
*
*******************************************************************************/
void CapSense_1_CSDSetupWidgetExt(uint32 widgetId, uint32 sensorId)
{
    CapSense_1_RAM_WD_BASE_STRUCT const *ptrWdgt = (CapSense_1_RAM_WD_BASE_STRUCT *)
                                                    CapSense_1_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;

    /* Save widget and sensor Ids to have access to it after scanning */
    CapSense_1_sensorIndex = (uint8)sensorId;

    /* Update global pointer to CapSense_1_RAM_SNS_STRUCT to current sensor */
    CapSense_1_curRamSnsPtr = (CapSense_1_RAM_SNS_STRUCT *)
                                              CapSense_1_dsFlash.wdgtArray[widgetId].ptr2SnsRam
                                              + CapSense_1_sensorIndex;

    CapSense_1_CSDSetupWidget(widgetId);

     /* Setup Idac Value */
    CapSense_1_SsCSDSetUpIdacs(ptrWdgt);

    #if (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN)
        /* initialize access pointers for current pointer to widget configuration in Flash */
        CapSense_1_curFlashWdgtPtr = &CapSense_1_dsFlash.wdgtArray[widgetId];
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN) */

    #if (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN)
        CapSense_1_SsCSDConnectSensorExt(widgetId, sensorId);
    #else
        /* Enable sensor */
        CapSense_1_curSnsIOPtr += CapSense_1_sensorIndex;
        CapSense_1_CSDConnectSns(CapSense_1_curSnsIOPtr);
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN) */

    #if (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN)
        /* Save sns pointer */
        CapSense_1_curFlashSnsPtr = (CapSense_1_FLASH_SNS_STRUCT const *)
                                           CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].ptr2SnsFlash +
                                           CapSense_1_sensorIndex;
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN) */

    CapSense_1_eleCsdDisconnectFlag = CapSense_1_DISCONNECT_IO_FLAG;

    /* Setup ISR handler to single-sensor scan function */
    CapSense_1_SsIsrInitialize(&CapSense_1_CSDPostSingleScan);
}


/*******************************************************************************
* Function Name: CapSense_1_SsCSDStartSample
****************************************************************************//**
*
* \brief
*   Starts the CSD conversion.
*
* \details
*   This function assumes that the CSD block is already set up using
*   the CapSense_1_CSDSetupWidget API and the sensor port-pin is connected to the CSD
*   block using CapSense_1_CSDConnectSns.
*
*******************************************************************************/
void CapSense_1_SsCSDStartSample(void)
{
    #ifdef CapSense_1_START_SAMPLE_CALLBACK
        CapSense_1_StartSampleCallback(CapSense_1_widgetIndex, CapSense_1_sensorIndex);
    #endif /* CapSense_1_START_SAMPLE_CALLBACK */

    /* Disable interrupt */
    #if defined(CapSense_1_ISR__INTC_ASSIGNED)
        NVIC_DisableIRQ(CapSense_1_ISR_cfg.intrSrc);
    #endif

    /* Enable power to sub-blocks */
    CY_SET_REG32(CapSense_1_CSD_HSCMP_PTR, CapSense_1_HSCMP_INIT_MASK);

    #if (CapSense_1_ENABLE == CapSense_1_BLOCK_OFF_AFTER_SCAN_EN)
        CY_SET_REG32(CapSense_1_CSD_CONFIG_PTR, CapSense_1_configCsd |
                                                  CapSense_1_CSD_CONFIG_SENSE_EN_MSK |
                                                  CapSense_1_CSD_CONFIG_ENABLE_MSK);
        #if(CapSense_1_CSD_ANALOG_STARTUP_DELAY_US > 0uL)
            Cy_SysLib_DelayUs(CapSense_1_CSD_ANALOG_STARTUP_DELAY_US);
        #endif /* (CapSense_1_CSD_ANALOG_STARTUP_DELAY_US > 0uL) */
    #endif /* (CapSense_1_ENABLE == CapSense_1_BLOCK_OFF_AFTER_SCAN_EN) */

    /* Precharging Cmod and Csh */
    CapSense_1_SsCSDCmodPrecharge();

    /* Trigger Scan */
    CapSense_1_SsCSDTriggerScan();
}


/*******************************************************************************
* Function Name: CapSense_1_CSDScanExt
****************************************************************************//**
*
* \brief
*  Starts the CSD conversion on the preconfigured sensor. This function requires
*  using the CapSense_1_CSDSetupWidgetExt() function to set up the a
*  widget.
*
* \details
*  This function performs single scanning of one sensor in the widget configured
*  by the CapSense_1_CSDSetupWidgetExt() function. It does the following
*  tasks:
*    1. Sets the busy flag in the CapSense_1_dsRam structure.
*    2. Performs the clock-phase alignment of the sense and modulator clocks.
*    3. Performs the Cmod pre-charging.
*    4. Starts single scanning.
*
*  Calling this function directly from the application layer is not
*  recommended. This function is used to implement only the user's specific
*  use cases (for faster execution time or pipeline scanning for example).
*  This function is called when no scanning is in progress. I.e.
*  CapSense_1_IsBusy() returns a non-busy status.
*
*  The sensor must be preconfigured by using the
*  CapSense_1_CSDSetupWidgetExt() API prior to calling this function.
*  The sensor remains ready for a next scan if a previous scan was triggered
*  by using the CapSense_1_CSDScanExt() function. In this case, calling
*  CapSense_1_CSDSetupWidgetExt() is not required every time before the
*  CapSense_1_CSDScanExt() function. If a previous scan was triggered in
*  any other way - CapSense_1_Scan(), CapSense_1_ScanAllWidgets() or
*  CapSense_1_RunTuner() - (see the CapSense_1_RunTuner() function
*  description for more details), the sensor must be preconfigured again by
*  using the CapSense_1_CSDSetupWidgetExt() API prior to calling the
*  CapSense_1_CSDScanExt() function.
*
*  If disconnection of the sensors is required after calling
*  CapSense_1_CSDScanExt(), the CapSense_1_CSDDisconnectSns()
*  function can be used.
*
*******************************************************************************/
void CapSense_1_CSDScanExt(void)
{
    #if ((CapSense_1_DISABLE == CapSense_1_CSD_COMMON_SNS_CLK_EN) && \
         (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN))
        CapSense_1_RAM_WD_BASE_STRUCT const *ptrWdgt = (CapSense_1_RAM_WD_BASE_STRUCT *)
                                                        CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].ptr2WdgtRam;

        CapSense_1_SsCSDCalculateScanDuration(ptrWdgt);
        CapSense_1_SsCSDConfigClock();
    #endif

    /* Set Start of sensor scan flag */
    CapSense_1_SetBusyFlags(CapSense_1_SW_STS_BUSY | CapSense_1_WDGT_SW_STS_BUSY);

    #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
        /* Reset the frequency scan channel if enabled */
        CapSense_1_scanFreqIndex = CapSense_1_FREQ_CHANNEL_0;
    #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */

    #if (CapSense_1_ENABLE != CapSense_1_BLOCK_OFF_AFTER_SCAN_EN)
        #if(CapSense_1_CSD_ANALOG_STARTUP_DELAY_US > 0uL)
            Cy_SysLib_DelayUs(CapSense_1_CSD_ANALOG_STARTUP_DELAY_US);
        #endif /* (CapSense_1_CSD_ANALOG_STARTUP_DELAY_US > 0uL) */
    #endif /* (CapSense_1_ENABLE != CapSense_1_BLOCK_OFF_AFTER_SCAN_EN) */

    CapSense_1_SsCSDStartSample();
}


/*******************************************************************************
* Function Name: CapSense_1_CSDScan
****************************************************************************//**
*
* \brief
*  This function initiates a scan for the sensors of the widget initialized by the
*  CapSense_1_CSDSetupWidget() function.
*
* \details
*  This function performs scanning of all the sensors in the widget configured by
*  the CapSense_1_CSDSetupWidget() function. It does the following tasks:
*    1. Connects the first sensor of the widget.
*    2. Configures the IDAC value.
*    3. Initializes the interrupt callback function to initialize a scan of
*       the next sensors in a widget.
*    4. Starts scanning for the first sensor in the widget.
*
*  This function is called by the CapSense_1_Scan() API if the given
*  widget uses the CSD sensing method.
*
*  Calling this function directly from the application layer is not
*  recommended. This function is used to implement only the user's specific
*  use cases (for faster execution time or pipeline scanning for example).
*
*  This function is called when no scanning is in progress. I.e.
*  CapSense_1_IsBusy() returns a non-busy status. The widget must be
*  preconfigured by the CapSense_1_CSDSetupWidget() function if any other
*  widget was previously scanned or any other type of the scan functions was used.
*
*******************************************************************************/
void CapSense_1_CSDScan(void)
{
    CapSense_1_RAM_WD_BASE_STRUCT const *ptrWdgt = (CapSense_1_RAM_WD_BASE_STRUCT *)
                                                    CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].ptr2WdgtRam;

     /*
      * Update CapSense_1_sensorIndex with the first sensor in
      * widget to use it in ISR handler to configure the next sensor
      */
    CapSense_1_sensorIndex = 0u;

    /* Update global pointer to CapSense_1_RAM_SNS_STRUCT to current sensor */
    CapSense_1_curRamSnsPtr = (CapSense_1_RAM_SNS_STRUCT *)
                                              CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].ptr2SnsRam
                                              + CapSense_1_sensorIndex;

    #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
        /* Reset the frequency scan channel if enabled */
        CapSense_1_scanFreqIndex = CapSense_1_FREQ_CHANNEL_0;
    #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */

    /* Setup Idac Value */
   CapSense_1_SsCSDSetUpIdacs(ptrWdgt);

    #if (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN)
        /* Check ganged sns flag */
        if (CapSense_1_GANGED_SNS_MASK == (CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].staticConfig &
            CapSense_1_GANGED_SNS_MASK))
        {
            /* Setup ISR handler to multiple-sensor scan function with ganged sensors */
            CapSense_1_SsIsrInitialize(&CapSense_1_CSDPostMultiScanGanged);
        }
        else
        {
            /* Set up ISR handler to multiple-sensor scan function without ganged sensors */
            CapSense_1_SsIsrInitialize(&CapSense_1_CSDPostMultiScan);
        }

        CapSense_1_SsCSDConnectSensorExt((uint32)CapSense_1_widgetIndex, (uint32)CapSense_1_sensorIndex);
    #else
        /* initialize ptr to sensor IO structure to appropriate address */
        CapSense_1_curSnsIOPtr = (CapSense_1_FLASH_IO_STRUCT const *)
                                        CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].ptr2SnsFlash
                                        + CapSense_1_sensorIndex;

        /* Enable sensor */
        CapSense_1_CSDConnectSns(CapSense_1_curSnsIOPtr);

        /* Set-up ISR handler to multiple-sensor scan function without ganged sensors */
        CapSense_1_SsIsrInitialize(&CapSense_1_CSDPostMultiScan);
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN) */

    /* Start scanning */
    CapSense_1_CSDScanExt();
}


/*******************************************************************************
* Function Name: CapSense_1_SsCSDConnectSensorExt
****************************************************************************//**
*
* \brief
*  Connects a ganged sensor port-pin to the sensing HW block via the AMUX bus.
*
* \details
*   The function gets the IO configuration registers addresses, their shifts and
*   masks from the FLASH_IO_STRUCT object. Basing on this data, it updates the HSIOM and
*   PC registers.
*
* \param widgetId
*  Specifies ID of the widget.
*
* \param sensorId
*  Specifies ID of the sensor in the widget.
*
*******************************************************************************/
void CapSense_1_SsCSDConnectSensorExt(uint32 widgetId, uint32 sensorId)
{
    #if (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN)
        uint32 tempVal;
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN) */

    /* initialize ptr to sensor IO structure to appropriate address */
    CapSense_1_curSnsIOPtr = (CapSense_1_FLASH_IO_STRUCT const *)
                                                      CapSense_1_dsFlash.wdgtArray[widgetId].ptr2SnsFlash
                                                      + sensorId;

    #if (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN)
        /* Check ganged sns flag */
        if (CapSense_1_GANGED_SNS_MASK ==
           (CapSense_1_dsFlash.wdgtArray[widgetId].staticConfig &
            CapSense_1_GANGED_SNS_MASK))
        {
            /* Get sns pointer */
            CapSense_1_curFlashSnsPtr = (CapSense_1_FLASH_SNS_STRUCT const *)
                                               CapSense_1_dsFlash.wdgtArray[widgetId].ptr2SnsFlash +
                                               sensorId;

            /* Get number of ganged pins */
            tempVal = CapSense_1_curFlashSnsPtr->numPins;

            /* Get IO pointer */
            CapSense_1_curSnsIOPtr = &CapSense_1_ioList[CapSense_1_curFlashSnsPtr->firstPinId];

            /* Connect all ganged sensors */
            do
            {
                CapSense_1_CSDConnectSns(CapSense_1_curSnsIOPtr);
                CapSense_1_curSnsIOPtr++;
                tempVal--;
            } while (0u != tempVal);
        }
        else
        {
            /* Connect sensor */
            CapSense_1_CSDConnectSns(CapSense_1_curSnsIOPtr);
        }
    #else
        /* Connect sensor */
        CapSense_1_CSDConnectSns(CapSense_1_curSnsIOPtr);
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN) */
}


/*******************************************************************************
* Function Name: CapSense_1_SsCSDDisconnectSnsExt
****************************************************************************//**
*
* \brief
*  Disconnects a ganged sensor port-pin from the sensing HW block and AMUX bus.
*  Sets the default state of the un-scanned sensor.
*
* \details
*   The function gets the IO configuration registers addresses, their shifts and
*   masks from the FLASH_IO_STRUCT object. Basing on this data and Inactive sensor
*   connection parameter, it updates the HSIOM, PC and DR registers.
*   The HSIOM register is updated only when the Inactive sensor connection parameter
*   is set to Shield.
*
* \param widgetId
*  Specifies ID of the widget.
*
* \param sensorId
*  Specifies ID of the sensor in the widget.
*
*******************************************************************************/
void CapSense_1_SsCSDDisconnectSnsExt(uint32 widgetId, uint32 sensorId)
{
    #if (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN)
        uint32 tempVal;
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN) */

    /* initialize ptr to sensor IO structure to appropriate address */
    CapSense_1_curSnsIOPtr = (CapSense_1_FLASH_IO_STRUCT const *)
                                                      CapSense_1_dsFlash.wdgtArray[widgetId].ptr2SnsFlash
                                                      + sensorId;

    #if (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN)
        /* Check ganged sns flag */
        if ((CapSense_1_dsFlash.wdgtArray[widgetId].staticConfig &
            CapSense_1_GANGED_SNS_MASK) == CapSense_1_GANGED_SNS_MASK)
        {
            /* Get sns pointer */
            CapSense_1_curFlashSnsPtr = (CapSense_1_FLASH_SNS_STRUCT const *)
                                               CapSense_1_dsFlash.wdgtArray[widgetId].ptr2SnsFlash +
                                               sensorId;

            /* Get number of ganged pins */
            tempVal = CapSense_1_curFlashSnsPtr->numPins;

            /* Get IO pointer */
            CapSense_1_curSnsIOPtr = &CapSense_1_ioList[CapSense_1_curFlashSnsPtr->firstPinId];

            /* Disconnect all ganged sensors */
            do
            {
                CapSense_1_CSDDisconnectSns(CapSense_1_curSnsIOPtr);
                CapSense_1_curSnsIOPtr++;
                tempVal--;
            } while (0u != tempVal);
        }
        else
        {
            /* Disconnect ganged sensor */
            CapSense_1_CSDDisconnectSns(CapSense_1_curSnsIOPtr);
        }
    #else
        /* Disconnect ganged sensor */
        CapSense_1_CSDDisconnectSns(CapSense_1_curSnsIOPtr);
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN) */
}


/*******************************************************************************
* Function Name: CapSense_1_CSDConnectSns
****************************************************************************//**
*
* \brief
*  Connects a port pin used by the sensor to the AMUX bus of the sensing HW block.
*
* \details
*  Connects a port pin used by the sensor to the AMUX bus of the sensing HW block
*  while a sensor is being scanned. The function ignores the fact if
*  the sensor is a ganged sensor and connects only a specified pin.
*
*  Scanning should be completed before calling this API.
*
*  Calling this function directly from the application layer is not
*  recommended. This function is used to implement only the user's specific
*  use cases. Functions that perform a setup and scan of a sensor/widget,
*  automatically set the required pin states and perform the sensor connection.
*  They do not take into account changes in the design made by the
*  CapSense_1_CSDConnectSns() function.
*
* \param  snsAddrPtr
*  Specifies the pointer to the FLASH_IO_STRUCT object belonging to a sensor
*  which to be connected to the sensing HW block.
*
*******************************************************************************/
void CapSense_1_CSDConnectSns(CapSense_1_FLASH_IO_STRUCT const *snsAddrPtr)
{
    uint32  interruptState;

    interruptState = Cy_SysLib_EnterCriticalSection();
    Cy_GPIO_SetDrivemode((GPIO_PRT_Type*)snsAddrPtr->pcPtr, (uint32)snsAddrPtr->pinNumber, CY_GPIO_DM_STRONG_IN_OFF);
    Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)snsAddrPtr->pcPtr, (uint32)snsAddrPtr->pinNumber, (en_hsiom_sel_t)CapSense_1_HSIOM_SEL_CSD_SENSE);
    Cy_SysLib_ExitCriticalSection(interruptState);
}


/*******************************************************************************
* Function Name: CapSense_1_CSDDisconnectSns
****************************************************************************//**
*
* \brief
*  Disconnects a sensor port pin from the sensing HW block and the AMUX bus. Sets
*  the default state of the un-scanned sensor.
*
* \details
*  This function works identically to CapSense_1_CSDConnectSns() except
*  it disconnects the specified port-pin used by the sensor.
*
*  Calling this function directly from the application layer is not
*  recommended. This function is used to implement only the user's specific
*  use cases. Functions that perform a setup and scan of sensor/widget
*  automatically set the required pin states and perform the sensor connection.
*  They ignore changes in the design made by the
*  CapSense_1_CSDDisconnectSns() function.
*
* \param  snsAddrPtr
*  Specifies the pointer to the FLASH_IO_STRUCT object belonging to a sensor
*  which should be disconnected from the sensing HW block.
*
*******************************************************************************/
void CapSense_1_CSDDisconnectSns(CapSense_1_FLASH_IO_STRUCT const *snsAddrPtr)
{
    uint32  interruptState;

    #if ((CapSense_1_DISABLE == CapSense_1_CSD_SHIELD_EN) ||  \
        (CapSense_1_SNS_CONNECTION_SHIELD != CapSense_1_CSD_INACTIVE_SNS_CONNECTION))
        interruptState = Cy_SysLib_EnterCriticalSection();
        Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)snsAddrPtr->pcPtr, (uint32)snsAddrPtr->pinNumber, (en_hsiom_sel_t)CapSense_1_HSIOM_SEL_GPIO);
        Cy_GPIO_SetDrivemode((GPIO_PRT_Type*)snsAddrPtr->pcPtr, (uint32)snsAddrPtr->pinNumber, CapSense_1_CSD_INACTIVE_SNS_GPIO_DM);
        Cy_GPIO_Clr((GPIO_PRT_Type*)snsAddrPtr->pcPtr, (uint32)snsAddrPtr->pinNumber);
        Cy_SysLib_ExitCriticalSection(interruptState);
    #else
        interruptState = Cy_SysLib_EnterCriticalSection();
        Cy_GPIO_SetDrivemode((GPIO_PRT_Type*)snsAddrPtr->pcPtr, (uint32)snsAddrPtr->pinNumber, CapSense_1_CSD_INACTIVE_SNS_GPIO_DM);
        Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)snsAddrPtr->pcPtr, (uint32)snsAddrPtr->pinNumber, CapSense_1_HSIOM_SEL_CSD_SHIELD);
        Cy_GPIO_Clr((GPIO_PRT_Type*)snsAddrPtr->pcPtr, (uint32)snsAddrPtr->pinNumber);
        Cy_SysLib_ExitCriticalSection(interruptState);
    #endif /* ((CapSense_1_DISABLE == CapSense_1_CSD_SHIELD_EN) && \
                (CapSense_1_SNS_CONNECTION_SHIELD != CapSense_1_CSD_INACTIVE_SNS_CONNECTION)) */
}

#if ((CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE) || \
     (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_AUTOCAL_EN))
    #if (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN)
        /*******************************************************************************
        * Function Name: CapSense_1_SsCSDSetSingleIdacMode
        ****************************************************************************//**
        *
        * \brief
        *  This function set single IDAC mode.
        *
        * \details
        *  The function enables IDACA LEG1 in CSD mode, IDACA LEG2 in GP mode.
        *
        *******************************************************************************/
        static void CapSense_1_SsCSDSetSingleIdacMode(void)
        {
            uint32 newRegValue;

            /* Enable IDACA LEG1 in CSD mode, IDACA LEG2 in GP mode */
            newRegValue = CY_GET_REG32(CapSense_1_CSD_IDACA_PTR) &
                          CapSense_1_CSD_IDACA_VAL_MSK;
            newRegValue |= CapSense_1_IDAC_MOD_CALIBRATION_CFG;
            CY_SET_REG32(CapSense_1_CSD_IDACA_PTR, newRegValue);
            #if (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) && \
                (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN)
                newRegValue = CY_GET_REG32(CapSense_1_CSD_IDACB_PTR) & CapSense_1_CSD_IDACB_VAL_MSK;
                newRegValue |= CapSense_1_IDAC_COMP_CALIBRATION_CFG;
                CY_SET_REG32(CapSense_1_CSD_IDACB_PTR, newRegValue);
            #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) && \
                      (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN */
        }
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) */
#endif /* ((CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE) || \
           (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_AUTOCAL_EN)) */

#if ((CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE) || \
     (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_AUTOCAL_EN))
    #if (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN)
        /*******************************************************************************
        * Function Name: CapSense_1_SsCSDRestoreIdacMode
        ****************************************************************************//**
        *
        * \brief
        *  This function restores IDAC configuration.
        *
        * \details
        *   The function restores Compensation IDAC if it is enabled.
        *
        *******************************************************************************/
        static void CapSense_1_SsCSDRestoreIdacMode(void)
        {
            /* Restore IDAC configuration */
            uint32 newRegValue;

            newRegValue = CY_GET_REG32(CapSense_1_CSD_SW_REFGEN_SEL_PTR);
            newRegValue |= CapSense_1_DEFAULT_SW_REFGEN_SEL;
            CY_SET_REG32(CapSense_1_CSD_SW_REFGEN_SEL_PTR, newRegValue);
            CY_SET_REG32(CapSense_1_CSD_IDACA_PTR, CapSense_1_IDAC_MOD_DEFAULT_CFG);
            #if (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) && \
                (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN)
                CY_SET_REG32(CapSense_1_CSD_IDACB_PTR, CapSense_1_IDAC_COMP_DEFAULT_CFG);
            #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) && \
                      (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN */
        }
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) */

    /*******************************************************************************
    * Function Name: CapSense_1_SsCSDCalibrateCheck
    ****************************************************************************//**
    *
    * \brief
    *  This internal function checks if the calibration is performed successfully.
    *
    * \details
    *   The function verifies calibration error based on maximum raw count level.
    *
    * \param rawLevel The raw data level which should be reached during the
    *   calibration procedure.
    * \param rawData  Calibrated raw counts.
    *
    * \return Returns the status of the operation:
    *   - Zero     - All the sensors in the widget are calibrated successfully.
    *   - Non-Zero - Calibration failed for any sensor in the widget.
    *
    *******************************************************************************/
    static cy_status CapSense_1_SsCSDCalibrateCheck(CapSense_1_RAM_WD_BASE_STRUCT const *ptrWdgt, uint32 rawLevel, uint32 rawData)
    {
        cy_status calibrateStatus;
        uint32 rawCountsError;
        uint32 minRawLevel = 0u;
        uint32 maxRawLevel = (0x00000001Lu << ptrWdgt->resolution) - 1u;

        /* Calculate the error in rawcounts for given resolution (round to floor) */
        rawCountsError = ((maxRawLevel * CapSense_1_IDAC_MOD_MAX_CALIB_ERROR) /
                          CapSense_1_PERCENTAGE_100) + 1u;

        /* Get minimum and maximum range values */
        if ((rawLevel + rawCountsError) <= maxRawLevel)
        {
            maxRawLevel = rawLevel + rawCountsError;
        }
        if (rawLevel >= rawCountsError)
        {
            minRawLevel = rawLevel - rawCountsError;
        }

        /* Check if calibration is in valid range */
        if ((rawData < minRawLevel) || (rawData > maxRawLevel))
        {
            calibrateStatus = CY_RET_BAD_DATA;
        }
        else
        {
            calibrateStatus = CY_RET_SUCCESS;
        }

        return (calibrateStatus);
    }


    /*******************************************************************************
    * Function Name: CapSense_1_SsCSDCalibrateOneSensor
    ****************************************************************************//**
    *
    * \brief
    *  Computes the Modulator IDAC value for one sensor that provides raw signal at
    *  the specified level.
    *
    * \details
    *   The function performs the following tasks:
    *   1. initializes The Compensation IDAC to zero
    *   2. Enables the calibrated sensor
    *   3. Performs eight iteration steps to get the target = rawLevel
    *      setting and resetting the IDAC value bits beginning from the
    *      MSB bit to an MSB bit.
    *   4. Disables the calibrated sensor
    *   5. Checks if the IDAC is in the correct range
    *   6. Updates the maximum IDAC value in the widget
    *   7. Stores the temporary ModIDAC8 value (in the single-IDAC mode).
    *
    *  The CSD block should be initialized with the CapSense_1_SsCSDInitialize()
    *  API before using this function.
    *
    * \param  rawLevel
    *   The raw data level which should be reached during the calibration procedure.
    * \param  ptrIdac  The pointer to IDAC.
    * \param  iDAC8Max The pointer to the maximum IDAC value in the widget.
    *
    * \return Returns the status of the operation:
    *  - Zero     - All the sensors in the widget are calibrated successfully.
    *  - Non-Zero - Calibration failed for any sensor in the widget.
    *
    *******************************************************************************/
    static cy_status CapSense_1_SsCSDCalibrateOneSensor(uint32 rawLevel,
                                                           uint8 *ptrIdac,
                                                           uint8 *iDAC8Max)
    {
        cy_status calibrateStatus;
        uint32 calMask = ((uint32)CapSense_1_CAL_MIDDLE_BIT << 1uL);
        uint32 watchdogCounter;

        uint16 rawData0 = 0u;
        #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
            uint16 rawData1 = 0u;
            uint16 rawData2 = 0u;
        #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */

        /* Declare and initialize ptr to widget and sensor structures to appropriate address */
        CapSense_1_RAM_WD_BASE_STRUCT *ptrWdgt = (CapSense_1_RAM_WD_BASE_STRUCT *)
                                                        CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].ptr2WdgtRam;

        ptrIdac[0u] = 0u;
        #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
            ptrIdac[CapSense_1_FREQ_CHANNEL_1] = 0u;
            ptrIdac[CapSense_1_FREQ_CHANNEL_2] = 0u;
        #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */

        /* Setup the calibrated sensor of the widget (Set IDAC = calMask and connect the sensor) */
        CapSense_1_CSDSetupWidgetExt((uint32)CapSense_1_widgetIndex, (uint32)CapSense_1_sensorIndex);

        #if (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN)
            /* Set single IDAC mode */
            CapSense_1_SsCSDSetSingleIdacMode();
        #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) */

        /*
         * Perform the iteration steps to get target = rawLevel.
         * The number of iteration steps is defined by calMask and depend on IDAC resolution.
         */
        do
        {
            /* Decrease IDAC until rawData reaches rawLevel */
            if (rawData0 < rawLevel)
            {
                /* Reset bit for null channel (decrease IDAC -> will increase rawData) */
                ptrIdac[0u] &= (uint8)(~calMask);
            }
            #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
                if (rawData1 < rawLevel)
                {
                    ptrIdac[CapSense_1_FREQ_CHANNEL_1] &= (uint8)(~calMask);
                }
                if (rawData2 < rawLevel)
                {
                    ptrIdac[CapSense_1_FREQ_CHANNEL_2] &= (uint8)(~calMask);
                }
            #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */

            /*
            * Shift calMask to set or reset next bit.
            * Perform scan even if callibration mask is equal to zero.
            * It gives raw counts for the last changed IDAC.
            */
            calMask >>= 1u;

            /* Set bit for null channel */
            ptrIdac[0u] |= (uint8)calMask;
            #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
                ptrIdac[CapSense_1_FREQ_CHANNEL_1] |= (uint8)calMask;
                ptrIdac[CapSense_1_FREQ_CHANNEL_2] |= (uint8)calMask;
            #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */

            /* Update IDAC */
            CapSense_1_SsCSDSetUpIdacs(ptrWdgt);

            /* Scan sensor */
            CapSense_1_CSDScanExt();

            /* Initialize Watchdog Counter with time interval which is enough to charge 100 nF capacitor */
            watchdogCounter = CapSense_1_CALIBR_WATCHDOG_CYCLES_NUM;
            while (((CapSense_1_dsRam.status & CapSense_1_SW_STS_BUSY) != 0u) &&
                   (0u != watchdogCounter))
            {
                /* Wait until scan complete and decrement Watchdog Counter to prevent unending loop */
                watchdogCounter--;
            }

            /* Read rawData */
            rawData0 = CapSense_1_curRamSnsPtr->raw[0u];
            #if(CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
                rawData1 = CapSense_1_curRamSnsPtr->raw[CapSense_1_FREQ_CHANNEL_1];
                rawData2 = CapSense_1_curRamSnsPtr->raw[CapSense_1_FREQ_CHANNEL_2];
            #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */
        }
        while (calMask > 0u);

        #if (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN)
            CapSense_1_SsCSDDisconnectSnsExt((uint32)CapSense_1_widgetIndex, (uint32)CapSense_1_sensorIndex);
        #else
            /* initialize pointer to sensor IO structure */
            CapSense_1_curSnsIOPtr = (CapSense_1_FLASH_IO_STRUCT const *)
                                            CapSense_1_dsFlash.wdgtArray[CapSense_1_widgetIndex].ptr2SnsFlash
                                            + CapSense_1_sensorIndex;

            /* Disable sensor */
            CapSense_1_CSDDisconnectSns(CapSense_1_curSnsIOPtr);
        #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_GANGED_SNS_EN) */

        /* Set flag about disconnecting the electrode */
        CapSense_1_eleCsdDisconnectFlag = 0u;

        #if (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN)
            /* Restore IDAC configuration to default state */
            CapSense_1_SsCSDRestoreIdacMode();
        #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) */

        /* Check if IDAC is in correct range */
        calibrateStatus = CapSense_1_SsCSDCalibrateCheck(ptrWdgt, rawLevel, (uint32)rawData0);
        #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
            calibrateStatus |= CapSense_1_SsCSDCalibrateCheck(ptrWdgt, rawLevel, (uint32)rawData1);
            calibrateStatus |= CapSense_1_SsCSDCalibrateCheck(ptrWdgt, rawLevel, (uint32)rawData2);
        #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */

        /*
         * Update maximum IDAC value in widget. iDAC8Max variable will contain
         * maximum IDAC value after all the sensors in a widget have been scanned.
         */
        if (*iDAC8Max < ptrIdac[0u])
        {
            *iDAC8Max = ptrIdac[0u];
            #if (CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE)
                /* Stores IDAC and raw count that corresponds to a sensor with maximum Cp within a widget */
                CapSense_1_calibratedIdac = *iDAC8Max;
                CapSense_1_calibratedRawcount = rawData0;
            #endif /* (CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE) */
        }
        #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
            if (iDAC8Max[CapSense_1_FREQ_CHANNEL_1] < ptrIdac[CapSense_1_FREQ_CHANNEL_1])
            {
                iDAC8Max[CapSense_1_FREQ_CHANNEL_1] = ptrIdac[CapSense_1_FREQ_CHANNEL_1];
            }
            if (iDAC8Max[CapSense_1_FREQ_CHANNEL_2] < ptrIdac[CapSense_1_FREQ_CHANNEL_2])
            {
                iDAC8Max[CapSense_1_FREQ_CHANNEL_2] = ptrIdac[CapSense_1_FREQ_CHANNEL_2];
            }
        #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */

        #if (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN)
            /*
             * Store temporary ModIDAC8 value (in single-IDAC mode) in idacComp variable.
             * We need this value to calculate idacMod value the in the dual-IDAC mode.
             */
            CapSense_1_curRamSnsPtr->idacComp[0u] = ptrIdac[0u];
            #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
                CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_1] = ptrIdac[CapSense_1_FREQ_CHANNEL_1];
                CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_2] = ptrIdac[CapSense_1_FREQ_CHANNEL_2];
            #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */
        #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) */

        return (calibrateStatus);
    }

    /*******************************************************************************
    * Function Name: CapSense_1_CSDCalibrateWidget
    ****************************************************************************//**
    *
    * \brief
    *  Executes the IDAC calibration for all the sensors in the widget specified in
    *  the input.
    *
    * \details
    *  Performs a successive approximation search algorithm to find appropriate
    *  IDAC values for sensors in the specified widget that provides the raw
    *  count to the level specified by the target parameter.
    *
    *  Calibration is always performed in the single IDAC mode and if the dual
    *  IDAC mode (Enable compensation IDAC is enabled) is configured, the IDAC
    *  values are re-calculated to match the raw count target. If a widget
    *  consists of two or more elements (buttons, slider segments, etc.), then
    *  calibration is performed by the element with the highest sensor
    *  capacitance.
    *
    *  Calibration fails if the achieved raw count is outside of the +/-10%
    *  range of the target.
    *
    *  This function is available when the CSD Enable IDAC auto-calibration
    *  parameter is enabled or the  SmartSense auto-tuning mode is configured.
    *
    * \param widgetId
    *  Specifies the ID number of the CSD widget to calibrate its raw count.
    *  A macro for the widget ID can be found in the
    *  CapSense_1 Configuration header file defined as
    *  CapSense_1_<WidgetName>_WDGT_ID.
    *
    * \param target
    *  Specifies the calibration target in percentages of the maximum raw count.
    *
    * \return
    *  Returns the status of the specified widget calibration:
    *    - CY_RET_SUCCESS - The operation is successfully completed.
    *    - CY_RET_BAD_PARAM - The input parameter is invalid.
    *    - CY_RET_BAD_DATA - The calibration failed and the Component may not operate as expected.
    *
    *******************************************************************************/
    cy_status CapSense_1_CSDCalibrateWidget(uint32 widgetId, uint32 target)
    {
        uint32 rawLevel;
        cy_status calibrateStatus = CY_RET_SUCCESS;

        #if (CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE)
            #if (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN)
                uint8 maxIdac = 1u;
                uint16 maxRaw = 1u;
            #endif /*(CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN) */
        #endif /* (CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE) */

        /* Declare and initialize ptr to widget and sensor structures to appropriate address */
        CapSense_1_RAM_WD_BASE_STRUCT *ptrWdgt = (CapSense_1_RAM_WD_BASE_STRUCT *)
                                              CapSense_1_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;

        #if (1u == CapSense_1_NUM_SCAN_FREQS)
            uint8 iDAC8Max[CapSense_1_NUM_SCAN_FREQS] = {1u};
        #else
            uint8 iDAC8Max[CapSense_1_NUM_SCAN_FREQS] = {1u, 1u, 1u};
        #endif /* (1u == CapSense_1_NUM_SCAN_FREQS) */

        uint8 *ptrIdac;

        /* Calibrate maximum level for given resolution */
        rawLevel = (0x00000001Lu << ptrWdgt->resolution) - 1u;

        /* Calculate percentage of maximum value */
        rawLevel = (rawLevel * target) / CapSense_1_PERCENTAGE_100;

        /* Update CapSense_1_widgetIndex */
        CapSense_1_widgetIndex = (uint8)widgetId;

        /* Setup ISR handler to single-sensor scan function */
        CapSense_1_SsIsrInitialize(&CapSense_1_CSDPostSingleScan);

        /* Init pointer to Modulator IDAC */
        ptrIdac = &ptrWdgt->idacMod[0u];

        #if (CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE)
            CapSense_1_calibratedIdac = 1u;
            CapSense_1_calibratedRawcount = 1u;
        #endif /* (CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE) */

        /* Go through all the sensors (or columns of touchpad) of widget to calibrate them */
        for (CapSense_1_sensorIndex = 0u;
             CapSense_1_sensorIndex < CapSense_1_dsFlash.wdgtArray[widgetId].numCols;
             CapSense_1_sensorIndex++)
        {
            /* Calibrate current sensor in widget */
            calibrateStatus |= CapSense_1_SsCSDCalibrateOneSensor(rawLevel, ptrIdac, iDAC8Max);
        }

        /* Normalize idacMod and idacComp values */
        #if (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN)
            /* Calculate Modulator value: idacMod = (iDAC8Max + 1)/2 */
            ptrIdac[0u] = (iDAC8Max[0u] + 1u) >> 1u;
            #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
                ptrIdac[CapSense_1_FREQ_CHANNEL_1] =  (iDAC8Max[CapSense_1_FREQ_CHANNEL_1] + 1u) >> 1u;
                ptrIdac[CapSense_1_FREQ_CHANNEL_2] = (iDAC8Max[CapSense_1_FREQ_CHANNEL_2] + 1u) >> 1u;
            #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */

            /* Go through all columns of touchpad (or all the sensors in button) to normalize them */
            for (CapSense_1_sensorIndex = 0u;
                 CapSense_1_sensorIndex < CapSense_1_dsFlash.wdgtArray[widgetId].numCols;
                 CapSense_1_sensorIndex++)
            {
                CapSense_1_curRamSnsPtr = (CapSense_1_RAM_SNS_STRUCT *)
                         CapSense_1_dsFlash.wdgtArray[widgetId].ptr2SnsRam + CapSense_1_sensorIndex;
                /* Calculate Compensation IDAC value: idacComp(i) = (iDAC8(i) - idacMod) * Target */
                if (CapSense_1_curRamSnsPtr->idacComp[0u] >= ptrIdac[0u])
                {
                    CapSense_1_curRamSnsPtr->idacComp[0u] -= ptrIdac[0u];
                    #if (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN)
                        CapSense_1_curRamSnsPtr->idacComp[0u] =
                            (uint8)((((target * CapSense_1_curRamSnsPtr->idacComp[0u]) + CapSense_1_PERCENTAGE_100) - 1u) /
                            CapSense_1_PERCENTAGE_100);
                    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN) */
                }
                else
                {
                    CapSense_1_curRamSnsPtr->idacComp[0u] = 0u;
                }
                #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
                    /* Do same for first frequency channel */
                    if (CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_1] >= ptrIdac[CapSense_1_FREQ_CHANNEL_1])
                    {
                        CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_1] -= ptrIdac[CapSense_1_FREQ_CHANNEL_1];
                        #if (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN)
                            CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_1] =
                                (uint8)((((target * CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_1]) +
                                CapSense_1_PERCENTAGE_100) - 1u) / CapSense_1_PERCENTAGE_100);
                        #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN) */
                    }
                    else
                    {
                       CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_1] = 0u;
                    }
                    /* Do same for second frequency channel */
                    if (CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_2] >= ptrIdac[CapSense_1_FREQ_CHANNEL_2])
                    {
                        CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_2] -= ptrIdac[CapSense_1_FREQ_CHANNEL_2];
                        #if (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN)
                            CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_2] =
                                (uint8)((((target * CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_2]) +
                                CapSense_1_PERCENTAGE_100) - 1u) / CapSense_1_PERCENTAGE_100);
                        #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN) */
                    }
                    else
                    {
                        CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_2] = 0u;
                    }
                #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */
            }
        #else
            /* Normalize widget for single-IDAC mode*/
            ptrIdac[0u] = iDAC8Max[0u];
            #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
                ptrIdac[CapSense_1_FREQ_CHANNEL_1] = iDAC8Max[CapSense_1_FREQ_CHANNEL_1];
                ptrIdac[CapSense_1_FREQ_CHANNEL_2] = iDAC8Max[CapSense_1_FREQ_CHANNEL_2];
            #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */
        #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) */

        #if (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN)
            if ((CapSense_1_WD_TOUCHPAD_E == (CapSense_1_WD_TYPE_ENUM)CapSense_1_dsFlash.wdgtArray[widgetId].wdgtType) ||
                (CapSense_1_WD_MATRIX_BUTTON_E == (CapSense_1_WD_TYPE_ENUM)CapSense_1_dsFlash.wdgtArray[widgetId].wdgtType))
            {
                /* Init pointer to Modulator IDAC for columns */
                ptrIdac = &ptrWdgt->rowIdacMod[0u];

                /* Reset the maximum value for columns */
                iDAC8Max[0u] = 1u;
                #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
                    iDAC8Max[CapSense_1_FREQ_CHANNEL_1] = 1u;
                    iDAC8Max[CapSense_1_FREQ_CHANNEL_2] = 1u;
                #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */

                #if (CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE)
                    maxIdac = CapSense_1_calibratedIdac;
                    maxRaw = CapSense_1_calibratedRawcount;
                    CapSense_1_calibratedIdac = 1u;
                    CapSense_1_calibratedRawcount = 1u;
                #endif /* (CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE) */

                /* Go through all columns of touchpad to calibrate them */
                for (CapSense_1_sensorIndex = CapSense_1_dsFlash.wdgtArray[widgetId].numCols;
                     CapSense_1_sensorIndex < (uint8)CapSense_1_dsFlash.wdgtArray[widgetId].totalNumSns;
                     CapSense_1_sensorIndex++)
                {
                    /* Calibrate current sensor in widget */
                    calibrateStatus |= CapSense_1_SsCSDCalibrateOneSensor(rawLevel, ptrIdac, iDAC8Max);
                }

                #if (CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE)
                    CapSense_1_calibratedIdacRow = CapSense_1_calibratedIdac;
                    CapSense_1_calibratedRawcountRow = CapSense_1_calibratedRawcount;
                    CapSense_1_calibratedIdac = maxIdac;
                    CapSense_1_calibratedRawcount = maxRaw;
                #endif /* (CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE) */

                /* Normalize idacMod and idacComp values */
                #if (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN)
                    /* Calculate Modulator value: idacMod = (iDAC8Max + 1)/2 */
                    ptrIdac[0u] = (iDAC8Max[0u] + 1u) >> 1u;
                    #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
                        ptrIdac[CapSense_1_FREQ_CHANNEL_1] =  (iDAC8Max[CapSense_1_FREQ_CHANNEL_1] + 1u) >> 1u;
                        ptrIdac[CapSense_1_FREQ_CHANNEL_2] = (iDAC8Max[CapSense_1_FREQ_CHANNEL_2] + 1u) >> 1u;
                    #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */

                    /* Go through all columns of touchpad to normalize them */
                    for (CapSense_1_sensorIndex = CapSense_1_dsFlash.wdgtArray[widgetId].numCols;
                         CapSense_1_sensorIndex < (uint8)CapSense_1_dsFlash.wdgtArray[widgetId].totalNumSns;
                         CapSense_1_sensorIndex++)
                    {
                        CapSense_1_curRamSnsPtr = (CapSense_1_RAM_SNS_STRUCT *)
                                 CapSense_1_dsFlash.wdgtArray[widgetId].ptr2SnsRam + CapSense_1_sensorIndex;

                        /* Calculate Compensation IDAC value: idacComp(i) = (iDAC8(i) - idacMod) * Target */
                        if (CapSense_1_curRamSnsPtr->idacComp[0u] >= ptrIdac[0u])
                        {
                            CapSense_1_curRamSnsPtr->idacComp[0u] -= ptrIdac[0u];
                            #if (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN)
                                CapSense_1_curRamSnsPtr->idacComp[0u] =
                                    (uint8)((((target * CapSense_1_curRamSnsPtr->idacComp[0u]) +
                                        CapSense_1_PERCENTAGE_100) - 1u) / CapSense_1_PERCENTAGE_100);
                            #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN) */
                        }
                        #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
                            /* Do same for first frequency channel */
                            if (CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_1] >= ptrIdac[CapSense_1_FREQ_CHANNEL_1])
                            {
                                CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_1] -= ptrIdac[CapSense_1_FREQ_CHANNEL_1];
                                #if (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN)
                                    CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_1] =
                                        (uint8)((((target * CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_1]) +
                                        CapSense_1_PERCENTAGE_100) - 1u) / CapSense_1_PERCENTAGE_100);
                                #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN) */
                            }
                            /* Do same for second frequency channel */
                            if (CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_2] >= ptrIdac[CapSense_1_FREQ_CHANNEL_2])
                            {
                                CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_2] -= ptrIdac[CapSense_1_FREQ_CHANNEL_2];
                                #if (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN)
                                    CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_2] =
                                        (uint8)((((target * CapSense_1_curRamSnsPtr->idacComp[CapSense_1_FREQ_CHANNEL_2]) +
                                        CapSense_1_PERCENTAGE_100) - 1u) / CapSense_1_PERCENTAGE_100);
                                #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_DEDICATED_IDAC_COMP_EN) */
                            }
                        #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */
                    }
                #else
                    /* Normalize widget for single-IDAC mode*/
                    ptrIdac[0u] = iDAC8Max[0u];
                    #if (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN)
                        ptrIdac[CapSense_1_FREQ_CHANNEL_1] = iDAC8Max[CapSense_1_FREQ_CHANNEL_1];
                        ptrIdac[CapSense_1_FREQ_CHANNEL_2] = iDAC8Max[CapSense_1_FREQ_CHANNEL_2];
                    #endif /* (CapSense_1_ENABLE == CapSense_1_MULTI_FREQ_SCAN_EN) */
                #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_COMP_EN) */
            }
        #endif /* (CapSense_1_CSD_MATRIX_WIDGET_EN || CapSense_1_CSD_TOUCHPAD_WIDGET_EN) */

        CapSense_1_sensorIndex = 0u;

        return calibrateStatus;
    }
#endif /* ((CapSense_1_CSD_SS_DIS != CapSense_1_CSD_AUTOTUNE)) || \
            (CapSense_1_ENABLE == CapSense_1_CSD_IDAC_AUTOCAL_EN)) */


/*******************************************************************************
* Function Name: CapSense_1_SsCSDCmodPrecharge
****************************************************************************//**
*
* \brief
*  This function initializes the Cmod charging to Vref.
*
* \details
*  Coarse initialization for CMOD and Cch.
*  The coarse initialization is performed by HSCOMP.
*  The HSCOMP monitors the Cmod voltage via Cmod sense path
*  and charges the Cmod using HCAV switch via CSDBUS-A, AMUXBUS-A
*  and static connection of Cmod to AMUXBUS-A.
*
*******************************************************************************/
static void CapSense_1_SsCSDCmodPrecharge(void)
{
    uint32 watchdogCounter;

    CY_SET_REG32(CapSense_1_CSD_CSDCMP_PTR, CapSense_1_CSD_CSDCMP_INIT);

    CY_SET_REG32(CapSense_1_CSD_SW_FW_MOD_SEL_PTR,  CapSense_1_CSD_SW_FW_MOD_SEL_INIT);
    CY_SET_REG32(CapSense_1_CSD_SW_FW_TANK_SEL_PTR, CapSense_1_CSD_SW_FW_TANK_SEL_INIT);
    CY_SET_REG32(CapSense_1_CSD_SW_SHIELD_SEL_PTR,  CapSense_1_CSD_SW_SHIELD_SEL_INIT);
    #if (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN)
        /*
         * Connect CMOD to (sense path) to HSCOMP: HMPM or HMPS or HMPT switches depend on Cmod connects to certain pad
         * Connect AMUXBUS-B to HSCMP positive input
         */
        CY_SET_REG32(CapSense_1_CSD_SW_HS_P_SEL_PTR, CapSense_1_CSD_SW_HS_P_SEL_COARSE);

        #if(0u != CapSense_1_CSD_SHIELD_TANK_EN)
            /* Connect CTANK to AMUXBUS-A */
            Cy_GPIO_SetHSIOM((GPIO_PRT_Type*)CapSense_1_CSD_CTANK_PORT_PTR, CapSense_1_CSD_CTANK_PIN,
                                                                       (en_hsiom_sel_t)CapSense_1_HSIOM_SEL_AMUXA);
        #endif /* (0u != CapSense_1_CSD_SHIELD_TANK_EN) */
    #else
        /* Connect CMOD to (sense path) to HSCOMP: HMPM or HMPS or HMPT switches depend on Cmod connects to certain pad */
        CY_SET_REG32(CapSense_1_CSD_SW_HS_P_SEL_PTR, CapSense_1_CSD_SW_HS_P_SEL_COARSE);
    #endif /* (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) */

    CY_SET_REG32(CapSense_1_CSD_SW_RES_PTR, CapSense_1_CSD_SW_RES_INIT);

    /* Clear all interrupt pending requests */
    CY_SET_REG32(CapSense_1_CSD_INTR_PTR, CapSense_1_CSD_INTR_ALL_MSK);
    (void)CY_GET_REG32(CapSense_1_CSD_INTR_PTR);

    /* Unmask INIT interrupt (enable interrupt) */
    CY_SET_REG32(CapSense_1_CSD_INTR_MASK_PTR, CapSense_1_CSD_INTR_MASK_CLEAR_MSK);

    /* Start SEQUENCER for coarse initialization for Cmod */
    CY_SET_REG32(CapSense_1_CSD_SEQ_START_PTR, CapSense_1_CSD_SEQ_START_SEQ_MODE_MSK |
                                                 CapSense_1_CSD_SEQ_START_START_MSK);

    /* Init Watchdog Counter to prevent a hang */
    watchdogCounter = CapSense_1_CSD_PRECHARGE_WATCHDOG_CYCLES_NUM;

    /* Wait for INTR.INIT goes high */
    while((0u == (CY_GET_REG32(CapSense_1_CSD_INTR_PTR) & CapSense_1_CSD_INTR_INIT_MSK)) && (0u != watchdogCounter))
    {
        watchdogCounter--;
    }

    if (0u == watchdogCounter)
    {
        /* Set sequencer to idle state if coarse initialization fails */
        CY_SET_REG32(CapSense_1_CSD_SEQ_START_PTR, CapSense_1_CSD_SEQ_START_ABORT_MSK);
    }
}


/*******************************************************************************
* Function Name: CapSense_1_SsCSDTriggerScan
****************************************************************************//**
*
* \brief
*  This function triggers the scanning.
*
* \details
*  Trigger the fine initialization (scan some dummy cycles) and start sampling.
*  Fine initialization for CMOD and Start scan.
*  For the fine initialization and sampling, Cmod is static connected to AMUXBUS-A
*  and in every conversion (one cycle of SnsClk), the sensor capacitance is charged
*  from Cmod and discharged to ground using the switches in GPIO cell.
*  The CSDCOMP monitors voltage on Cmod using the sense path and charges Cmod back
*  to Vref using IDACs by connecting IDAC to CSDBUS-A and then the AMUXBUS-A.
*
*******************************************************************************/
static void CapSense_1_SsCSDTriggerScan(void)
{
    uint32 watchdogCounter;

    /* Clear previous interrupts */
    CY_SET_REG32(CapSense_1_CSD_INTR_PTR, CapSense_1_CSD_INTR_ALL_MSK);
    (void)CY_GET_REG32(CapSense_1_CSD_INTR_PTR);

    /* Enable interrupt */
    #if defined(CapSense_1_ISR__INTC_ASSIGNED)
        NVIC_EnableIRQ(CapSense_1_ISR_cfg.intrSrc);
    #endif

    CY_SET_REG32(CapSense_1_CSD_SW_HS_P_SEL_PTR, CapSense_1_CSD_SW_HS_P_SEL_SCAN);

    /* Set scanning configuration for switches */
    CY_SET_REG32(CapSense_1_CSD_SW_FW_MOD_SEL_PTR, CapSense_1_CSD_SW_FW_MOD_SEL_SCAN);
    CY_SET_REG32(CapSense_1_CSD_SW_FW_TANK_SEL_PTR,CapSense_1_CSD_SW_FW_TANK_SEL_SCAN);

    CY_SET_REG32(CapSense_1_CSD_SW_SHIELD_SEL_PTR,  CapSense_1_CSD_SW_SHIELD_SEL_SCAN);

    #if((CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) && (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_TANK_EN))
        /* Connect CTANK to AMUXBUS-B */
        CapSense_1_SsCSDEnableShieldTank();
    #endif /* ((CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_EN) && (CapSense_1_ENABLE == CapSense_1_CSD_SHIELD_TANK_EN)) */

    CY_SET_REG32(CapSense_1_CSD_SW_RES_PTR, CapSense_1_CSD_SW_RES_SCAN);
    CY_SET_REG32(CapSense_1_CSD_CSDCMP_PTR, CapSense_1_CSD_CSDCMP_SCAN);

    #if (CapSense_1_DISABLE == CapSense_1_CSD_SHIELD_EN)
        /* Disable HSCOMP during the sampling phase when shield is disabled */
        CY_SET_REG32(CapSense_1_CSD_HSCMP_PTR, 0u);
    #else
        CY_SET_REG32(CapSense_1_CSD_HSCMP_PTR, CapSense_1_HSCMP_SCAN_MASK);
    #endif /* (CapSense_1_DISABLE == CapSense_1_CSD_SHIELD_EN) */

    #if (CapSense_1_CLK_SOURCE_DIRECT != CapSense_1_CSD_SNS_CLK_SOURCE)
        /* Force the LFSR to it's initial state (all ones) */
        CY_SET_REG32(CapSense_1_CSD_SENSE_PERIOD_PTR, CY_GET_REG32(CapSense_1_CSD_SENSE_PERIOD_PTR) |
                                                       CapSense_1_CSD_SENSE_PERIOD_LFSR_CLEAR_MSK);
    #endif /* (CapSense_1_CLK_SOURCE_DIRECT != CapSense_1_CSD_SNS_CLK_SOURCE) */

    /* Enable SAMPLE interrupt */
    CY_SET_REG32(CapSense_1_CSD_INTR_MASK_PTR, CapSense_1_CSD_INTR_MASK_SAMPLE_MSK);

    /* Init Watchdog Counter to prevent a hang */
    watchdogCounter = CapSense_1_CSD_PRECHARGE_WATCHDOG_CYCLES_NUM;

    /* Wait for IDLE state of the HW sequencer */
    while((0u != (CapSense_1_CSD_STAT_SEQ_REG & CapSense_1_CSD_STAT_SEQ_SEQ_STATE_MSK)) && (0u != watchdogCounter))
    {
        watchdogCounter--;
    }

    /* Start SEQUENCER for fine initialization scan for Cmod and then for normal scan */
    CY_SET_REG32(CapSense_1_CSD_SEQ_START_PTR, CapSense_1_CSD_SEQ_START_AZ0_SKIP_MSK |
                                                 CapSense_1_CSD_SEQ_START_AZ1_SKIP_MSK |
                                                 CapSense_1_CSD_SEQ_START_START_MSK);
}

#endif  /* (CapSense_1_ENABLE == CapSense_1_CSD_EN) */


/* [] END OF FILE */
