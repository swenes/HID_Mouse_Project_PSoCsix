/***************************************************************************//**
* \file CapSense_1_Filter.h
* \version 3.0
*
* \brief
*   This file contains the definitions for all firmware filters implementation.
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

#if !defined(CY_SENSE_CapSense_1_FILTER_H)
#define CY_SENSE_CapSense_1_FILTER_H

#include "syslib/cy_syslib.h"
#include "cyfitter.h"

#include "CapSense_1_Structure.h"
#include "CapSense_1_Configuration.h"

/***************************************
* Function Prototypes
***************************************/

/*******************************************************************************
* LOW LEVEL API
*******************************************************************************/
/**
* \cond SECTION_CYSENSE_LOW_LEVEL
* \addtogroup group_cysense_low_level
* \{
*/

cy_status CapSense_1_UpdateAllBaselines(void);
cy_status CapSense_1_UpdateWidgetBaseline(uint32 widgetId);
cy_status CapSense_1_UpdateSensorBaseline(uint32 widgetId, uint32 sensorId);

void CapSense_1_InitializeAllBaselines(void);
void CapSense_1_InitializeWidgetBaseline(uint32 widgetId);
void CapSense_1_InitializeSensorBaseline(uint32 widgetId, uint32 sensorId);

#if ((CapSense_1_ENABLE == CapSense_1_RC_FILTER_EN) || \
     (0u != (CapSense_1_CSD_AUTOTUNE & CapSense_1_CSD_SS_TH_EN)))
    void CapSense_1_InitializeAllFilters(void);
    void CapSense_1_InitializeWidgetFilter(uint32 widgetId);
#endif /* ((CapSense_1_ENABLE == CapSense_1_RC_FILTER_EN) || \
           (0u != (CapSense_1_CSD_AUTOTUNE & CapSense_1_CSD_SS_TH_EN))) */

/** \}
* \endcond */


/*******************************************************************************
* Function Prototypes - internal functions
*******************************************************************************/
/**
* \cond SECTION_CYSENSE_INTERNAL
* \addtogroup group_cysense_internal
* \{
*/

void CapSense_1_FtInitialize(void);

#if (0u != (CapSense_1_CSD_AUTOTUNE & CapSense_1_CSD_SS_TH_EN))
    void CapSense_1_RunNoiseEnvelope(uint32 widgetId, uint32 sensorId);
    void CapSense_1_InitializeNoiseEnvelope(uint32 widgetId, uint32 sensorId);
#endif /* (0u != (CapSense_1_CSD_AUTOTUNE & CapSense_1_CSD_SS_TH_EN)) */

#if (CapSense_1_REGULAR_RC_IIR_FILTER_EN || CapSense_1_PROX_RC_IIR_FILTER_EN)
    void CapSense_1_InitializeIIR(uint32 widgetId, uint32 sensorId);
    void CapSense_1_RunIIR(uint32 widgetId, uint32 sensorId);
#endif /* (CapSense_1_REGULAR_RC_IIR_FILTER_EN || CapSense_1_PROX_RC_IIR_FILTER_EN) */

#if (CapSense_1_REGULAR_RC_MEDIAN_FILTER_EN || CapSense_1_PROX_RC_MEDIAN_FILTER_EN)
    void CapSense_1_InitializeMedian(uint32 widgetId, uint32 sensorId);
    void CapSense_1_RunMedian(uint32 widgetId, uint32 sensorId);
#endif /* (CapSense_1_REGULAR_RC_MEDIAN_FILTER_EN || CapSense_1_PROX_RC_MEDIAN_FILTER_EN) */

#if (CapSense_1_REGULAR_RC_AVERAGE_FILTER_EN || CapSense_1_PROX_RC_AVERAGE_FILTER_EN)
    void CapSense_1_InitializeAverage(uint32 widgetId, uint32 sensorId);
    void CapSense_1_RunAverage(uint32 widgetId, uint32 sensorId);
#endif /* (CapSense_1_REGULAR_RC_AVERAGE_FILTER_EN || CapSense_1_PROX_RC_AVERAGE_FILTER_EN) */

void CapSense_1_FtInitializeBaseline(CapSense_1_RAM_SNS_STRUCT *ptrSensor, uint32 wdType);
uint32 CapSense_1_FtUpdateBaseline(
                            CapSense_1_RAM_WD_BASE_STRUCT *ptrWidgetRam,
                            CapSense_1_RAM_SNS_STRUCT *ptrSensor,
                            uint32 wdType);

#if (CapSense_1_POS_MEDIAN_FILTER_EN || CapSense_1_REGULAR_RC_MEDIAN_FILTER_EN || CapSense_1_PROX_RC_MEDIAN_FILTER_EN)
uint32 CapSense_1_FtMedian(uint32 x1, uint32 x2, uint32 x3);
#endif /*CapSense_1_POS_MEDIAN_FILTER_EN || CapSense_1_REGULAR_RC_MEDIAN_FILTER_EN || CapSense_1_PROX_RC_MEDIAN_FILTER_EN*/

uint32 CapSense_1_FtIIR1stOrder(uint32 input, uint32 prevOutput, uint32 n, uint32 shift);

#if (CapSense_1_POS_JITTER_FILTER_EN)
    uint32 CapSense_1_FtJitter(uint32 input, uint32 prevOutput);
#endif /* CapSense_1_POS_JITTER_FILTER_EN */

void CapSense_1_FtInitializeBaselineChannel(CapSense_1_RAM_SNS_STRUCT *ptrSensor, uint32 wdType, uint32 channel);

#if (CapSense_1_ENABLE == CapSense_1_RC_FILTER_EN)
    void CapSense_1_FtRunEnabledFilters(uint32 widgetId, uint32 sensorId);
    void CapSense_1_FtRunEnabledFiltersInternal(CapSense_1_PTR_FILTER_VARIANT ptrFilterHistObj,
                                                      CapSense_1_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
#endif /* (CapSense_1_ENABLE == CapSense_1_RC_FILTER_EN) */


#if (CapSense_1_REGULAR_RC_IIR_FILTER_EN || CapSense_1_PROX_RC_IIR_FILTER_EN)
    void CapSense_1_InitializeIIRInternal(CapSense_1_PTR_FILTER_VARIANT ptrFilterHistObj,
                                                  CapSense_1_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
    void CapSense_1_RunIIRInternal(CapSense_1_PTR_FILTER_VARIANT ptrFilterHistObj,
                                           CapSense_1_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
#endif /* (CapSense_1_REGULAR_RC_IIR_FILTER_EN || CapSense_1_PROX_RC_IIR_FILTER_EN) */

#if (CapSense_1_REGULAR_RC_MEDIAN_FILTER_EN || CapSense_1_PROX_RC_MEDIAN_FILTER_EN)
    void CapSense_1_InitializeMedianInternal(CapSense_1_PTR_FILTER_VARIANT ptrFilterHistObj,
                                                     CapSense_1_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
    void CapSense_1_RunMedianInternal(CapSense_1_PTR_FILTER_VARIANT ptrFilterHistObj,
                                              CapSense_1_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
#endif /* (CapSense_1_REGULAR_RC_MEDIAN_FILTER_EN || CapSense_1_PROX_RC_MEDIAN_FILTER_EN) */

#if (CapSense_1_REGULAR_RC_AVERAGE_FILTER_EN || CapSense_1_PROX_RC_AVERAGE_FILTER_EN)
    void CapSense_1_InitializeAverageInternal(CapSense_1_PTR_FILTER_VARIANT ptrFilterHistObj,
                                                      CapSense_1_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
    void CapSense_1_RunAverageInternal(CapSense_1_PTR_FILTER_VARIANT ptrFilterHistObj,
                                               CapSense_1_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType);
#endif /* (CapSense_1_REGULAR_RC_AVERAGE_FILTER_EN || CapSense_1_PROX_RC_AVERAGE_FILTER_EN) */

#if (CapSense_1_ENABLE == CapSense_1_ALP_FILTER_EN)
    void CapSense_1_InitializeALP(uint32 widgetId, uint32 sensorId);
    void CapSense_1_RunALP(uint32 widgetId, uint32 sensorId);
    void CapSense_1_InitializeALPInternal(CapSense_1_PTR_FILTER_VARIANT ptrFilterHistObj,
                                                CapSense_1_RAM_SNS_STRUCT *ptrSensorObj,
                                                uint32 wdType);
    void CapSense_1_ConfigRunALPInternal(CapSense_1_PTR_FILTER_VARIANT ptrFilterHistObj,
                                        CapSense_1_RAM_WD_BASE_STRUCT *ptrRamWdgt,
                                        CapSense_1_RAM_SNS_STRUCT *ptrSensorObj,
                                        uint32 wdType);
    void CapSense_1_RunALPInternal(CapSense_1_PTR_FILTER_VARIANT ptrFilterHistObj,
                                        ALP_FLTR_CONFIG_STRUCT *ptrAlpFilterConfig,
                                        CapSense_1_RAM_SNS_STRUCT *ptrSensorObj,
                                        uint32 wdType);
#endif /* (CapSense_1_ENABLE == CapSense_1_ALP_FILTER_EN) */

/** \}
* \endcond */

/***************************************
* Initial Parameter Constants
***************************************/
#define NOISE_ENVELOPE_SHIFT                        (0x02u)
#define NOISE_ENVELOPE_RUN_DIFF_SHIFT               (0x03u)
#define NOISE_ENVELOPE_SIGN_REG                     (0x0Fu)
#define NOISE_ENVELOPE_SIGN_REG_SHIFT               (0x01u)
#define NOISE_ENVELOPE_RESET_COUNTER                (0x0Au)
#define NOISE_ENVELOPE_4_TIMES                      (0x02u)

#endif /* End CY_SENSE_CapSense_1_FILTER_H */


/* [] END OF FILE */
