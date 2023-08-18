/***************************************************************************//**
* \file USBFS.h
* \version 1.0
*
* This file provides constants and parameter values for the USBFS Device 
* Component.
*
********************************************************************************
* \copyright
* Copyright 2018, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(USBFS_CY_USBFS_DEV_PDL_H)
#define USBFS_CY_USBFS_DEV_PDL_H

#include "cyfitter.h"
#include "cy_usbfs_dev_drv.h"
#include "cy_usb_dev.h"
#include "cy_usb_dev_audio.h"
#include "cy_usb_dev_cdc.h"
#include "cy_usb_dev_hid.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
*                          Initial Parameter Constants
*******************************************************************************/

/* Defines the nubmer of enabled endpoints */
#define USBFS_ACTIVE_ENDPOINTS_MASK  (1U)

/* Operation mode */
#define USBFS_EP_MANAGEMENT_DMA_AUTO (0UL)

/* Enabled Dma channels for endpoints 1-8 */
#define USBFS_EP1_DMA_CH_ENABLED     (0UL)
#define USBFS_EP2_DMA_CH_ENABLED     (0UL)
#define USBFS_EP3_DMA_CH_ENABLED     (0UL)
#define USBFS_EP4_DMA_CH_ENABLED     (0UL)
#define USBFS_EP5_DMA_CH_ENABLED     (0UL)
#define USBFS_EP6_DMA_CH_ENABLED     (0UL)
#define USBFS_EP7_DMA_CH_ENABLED     (0UL)
#define USBFS_EP8_DMA_CH_ENABLED     (0UL)

/* Interrupt Level Select register */
#define USBFS_INTR_LEVEL_SEL \
                        (CY_USBFS_DEV_DRV_SET_SOF_LVL(2UL) | \
                         CY_USBFS_DEV_DRV_SET_BUS_RESET_LVL(2UL) | \
                         CY_USBFS_DEV_DRV_SET_EP0_LVL(2UL) | \
                         CY_USBFS_DEV_DRV_SET_LPM_LVL(0UL) | \
                         CY_USBFS_DEV_DRV_SET_ARB_EP_LVL(0UL) | \
                         CY_USBFS_DEV_DRV_SET_EP1_LVL(1UL) | \
                         CY_USBFS_DEV_DRV_SET_EP2_LVL(1UL) | \
                         CY_USBFS_DEV_DRV_SET_EP3_LVL(1UL) | \
                         CY_USBFS_DEV_DRV_SET_EP4_LVL(1UL) | \
                         CY_USBFS_DEV_DRV_SET_EP5_LVL(1UL) | \
                         CY_USBFS_DEV_DRV_SET_EP6_LVL(1UL) | \
                         CY_USBFS_DEV_DRV_SET_EP7_LVL(1UL) | \
                         CY_USBFS_DEV_DRV_SET_EP8_LVL(1UL))


/*******************************************************************************
*                              Conditional includes
*******************************************************************************/

#if (USBFS_EP1_DMA_CH_ENABLED)
    #include "USBFS_ep1_dma.h"
#endif /* (USBFS_EP1_DMA_CH_ENABLED) */

#if (USBFS_EP2_DMA_CH_ENABLED)
    #include "USBFS_ep2_dma.h"
#endif /* (USBFS_EP2_DMA_CH_ENABLED) */

#if (USBFS_EP3_DMA_CH_ENABLED)
    #include "USBFS_ep3_dma.h"
#endif /* (USBFS_EP3_DMA_CH_ENABLED) */

#if (USBFS_EP4_DMA_CH_ENABLED)
    #include "USBFS_ep4_dma.h"
#endif /* (USBFS_EP4_DMA_CH_ENABLED) */

#if (USBFS_EP5_DMA_CH_ENABLED)
    #include "USBFS_ep5_dma.h"
#endif /* (USBFS_EP5_DMA_CH_ENABLED) */

#if (USBFS_EP6_DMA_CH_ENABLED)
    #include "USBFS_ep6_dma.h"
#endif /* (USBFS_EP6_DMA_CH_ENABLED) */

#if (USBFS_EP7_DMA_CH_ENABLED)
    #include "USBFS_ep7_dma.h"
#endif /* (USBFS_EP7_DMA_CH_ENABLED) */

#if (USBFS_EP8_DMA_CH_ENABLED)
    #include "USBFS_ep8_dma.h"
#endif /* (USBFS_EP8_DMA_CH_ENABLED) */


/*******************************************************************************
*                             Function Prototypes
*******************************************************************************/
/**
* \addtogroup group_general
* @{
*/
/* Component specific functions. */
void USBFS_Start(uint32_t device, bool blocking);
/** @} group_general */


/*******************************************************************************
*                          Variables with External Linkage
*******************************************************************************/
/**
* \addtogroup group_globals
* @{
*/
extern uint8_t USBFS_initVar;
extern cy_stc_usbfs_dev_drv_config_t const USBFS_drvConfig;
extern cy_stc_usbfs_dev_drv_context_t USBFS_drvContext;

extern cy_stc_usb_dev_context_t       USBFS_devContext;
extern cy_stc_usb_dev_audio_context_t USBFS_audioContext;
extern cy_stc_usb_dev_cdc_context_t   USBFS_cdcContext;
extern cy_stc_usb_dev_hid_context_t   USBFS_hidContext;
/** @} group_globals */


/*******************************************************************************
*                         Preprocessor Macros
*******************************************************************************/
/**
* \addtogroup group_macros
* @{
*/
/** The pointer to the base address of the hardware */
#define USBFS_HW     ((USBFS_Type *) USBFS_USBFS__HW)
/** @} group_macros */


/*******************************************************************************
*                         In-line Function Implementation
*******************************************************************************/


/*******************************************************************************
* Function Name: USBFS_InterruptHigh
****************************************************************************//**
*
* Interrupt Service Routine for the high group of the interrupt sources.
*
*******************************************************************************/
__STATIC_INLINE void USBFS_InterruptHigh(void)
{
    /* Get interrupt cause */
    uint32_t intrCause = Cy_USBFS_Dev_Drv_GetInterruptCauseHi(USBFS_HW);   
    
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USBFS_HW, intrCause, &USBFS_drvContext);
}


/*******************************************************************************
* Function Name: USBFS_InterruptMedium
****************************************************************************//**
*
* Interrupt Service Routine for the medium group of the interrupt sources.
*
*******************************************************************************/
__STATIC_INLINE void USBFS_InterruptMedium(void)
{
    /* Get interrupt cause */
    uint32_t intrCause = Cy_USBFS_Dev_Drv_GetInterruptCauseMed(USBFS_HW);
    
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USBFS_HW, intrCause, &USBFS_drvContext);
}


/*******************************************************************************
* Function Name: USBFS_InterruptLow
****************************************************************************//**
*
* Interrupt Service Routine for the low group of the interrupt sources.
*
*******************************************************************************/
__STATIC_INLINE void USBFS_InterruptLow(void)
{
    /* Get interrupt cause */
    uint32_t intrCause = Cy_USBFS_Dev_Drv_GetInterruptCauseLo(USBFS_HW);
    
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USBFS_HW, intrCause, &USBFS_drvContext);
}

#if defined(__cplusplus)
}
#endif

#endif /* USBFS_CY_USBFS_DEV_PDL_H */


/* [] END OF FILE */
