/***************************************************************************//**
* \file USBFS.c
* \version 1.0
*
* This file provides the source code to the API for the USBFS Device Component.
*
********************************************************************************
* \copyright
* Copyright 2018, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "USBFS_cfg.h"

#include "cy_sysint.h"
#include "cyfitter_sysint.h"
#include "cyfitter_sysint_cfg.h"

#if defined(__cplusplus)
extern "C" {
#endif


/*******************************************************************************
* Global data allocation
*******************************************************************************/

#if (USBFS_EP_MANAGEMENT_DMA_AUTO)
    /* Buffer for endpoints data */
    static uint8_t USBFS_endpointsBuffer[USBFS_ENDPOINTS_BUFFER_SIZE] CY_ALIGN(2);
#endif /* (USBFS_EP_MANAGEMENT_DMA_AUTO) */

#if (USBFS_EP1_DMA_CH_ENABLED)
    /* DMA Descriptors */
    static cy_stc_dma_descriptor_t USBFS_ep1DmaDescr0;
    
    /* DMA channel configuration */
    static cy_stc_usbfs_dev_drv_dma_config_t const USBFS_ep1dmaConfig =
    {
        .base        = USBFS_ep1_dma_HW,
        .chNum       = USBFS_ep1_dma_DW_CHANNEL,
        .priority    = USBFS_ep1_dma_PRIORITY,
        .preemptable = USBFS_ep1_dma_PREEMPTABLE,
        .descr0      = &USBFS_ep1DmaDescr0,
        .descr1      = NULL,
        .outTrigMux  = (uint32_t) USBFS_USBFS__DMA_BURSTEND_0,
    };
#endif /* (USBFS_EP1_DMA_CH_ENABLED) */

#if (USBFS_EP2_DMA_CH_ENABLED)
    /* DMA Descriptors */
    static cy_stc_dma_descriptor_t USBFS_ep2DmaDescr0;
    
    /* DMA channel configuration */
    static cy_stc_usbfs_dev_drv_dma_config_t const USBFS_ep2dmaConfig =
    {
        .base        = USBFS_ep2_dma_HW,
        .chNum       = USBFS_ep2_dma_DW_CHANNEL,
        .priority    = USBFS_ep2_dma_PRIORITY,
        .preemptable = USBFS_ep2_dma_PREEMPTABLE,
        .descr0      = &USBFS_ep2DmaDescr0,
        .descr1      = NULL,
        .outTrigMux  = (uint32_t) USBFS_USBFS__DMA_BURSTEND_1,
    };
#endif /* (USBFS_EP2_DMA_CH_ENABLED) */

#if (USBFS_EP3_DMA_CH_ENABLED)
    /* DMA Descriptors */
    static cy_stc_dma_descriptor_t USBFS_ep3DmaDescr0;
    
    /* DMA channel configuration */
    static cy_stc_usbfs_dev_drv_dma_config_t const USBFS_ep3dmaConfig =
    {
        .base        = USBFS_ep3_dma_HW,
        .chNum       = USBFS_ep3_dma_DW_CHANNEL,
        .priority    = USBFS_ep3_dma_PRIORITY,
        .preemptable = USBFS_ep3_dma_PREEMPTABLE,
        .descr0      = &USBFS_ep3DmaDescr0,
        .descr1      = NULL,
        .outTrigMux  = (uint32_t) USBFS_USBFS__DMA_BURSTEND_2,      
    };
#endif /* (USBFS_EP3_DMA_CH_ENABLED) */

#if (USBFS_EP4_DMA_CH_ENABLED)
    /* DMA Descriptors */
    static cy_stc_dma_descriptor_t USBFS_ep4DmaDescr0;
    
    /* DMA channel configuration */
    static cy_stc_usbfs_dev_drv_dma_config_t const USBFS_ep4dmaConfig =
    {
        .base        = USBFS_ep4_dma_HW,
        .chNum       = USBFS_ep4_dma_DW_CHANNEL,
        .priority    = USBFS_ep4_dma_PRIORITY,
        .preemptable = USBFS_ep4_dma_PREEMPTABLE,
        .descr0      = &USBFS_ep4DmaDescr0,
        .descr1      = NULL,
        .outTrigMux  = (uint32_t) USBFS_USBFS__DMA_BURSTEND_3,    
    };
#endif /* (USBFS_EP4_DMA_CH_ENABLED) */

#if (USBFS_EP5_DMA_CH_ENABLED)
    /* DMA Descriptors */
    static cy_stc_dma_descriptor_t USBFS_ep5DmaDescr0;
    
    /* DMA channel configuration */
    static cy_stc_usbfs_dev_drv_dma_config_t const USBFS_ep5dmaConfig =
    {
        .base        = USBFS_ep5_dma_HW,
        .chNum       = USBFS_ep5_dma_DW_CHANNEL,
        .priority    = USBFS_ep5_dma_PRIORITY,
        .preemptable = USBFS_ep5_dma_PREEMPTABLE,
        .descr0      = &USBFS_ep5DmaDescr0,
        .descr1      = NULL,
        .outTrigMux  = (uint32_t) USBFS_USBFS__DMA_BURSTEND_4,       
    };
#endif /* (USBFS_EP5_DMA_CH_ENABLED) */

#if (USBFS_EP6_DMA_CH_ENABLED)
    /* DMA Descriptors */
    static cy_stc_dma_descriptor_t USBFS_ep6DmaDescr0;
    
    /* DMA channel configuration */
    static cy_stc_usbfs_dev_drv_dma_config_t const USBFS_ep6dmaConfig =
    {
        .base        = USBFS_ep6_dma_HW,
        .chNum       = USBFS_ep6_dma_DW_CHANNEL,
        .priority    = USBFS_ep6_dma_PRIORITY,
        .preemptable = USBFS_ep6_dma_PREEMPTABLE,
        .descr0      = &USBFS_ep6DmaDescr0,
        .descr1      = NULL,
        .outTrigMux  = (uint32_t) USBFS_USBFS__DMA_BURSTEND_5,
    };
#endif /* (USBFS_EP6_DMA_CH_ENABLED) */

#if (USBFS_EP7_DMA_CH_ENABLED)
    /* DMA Descriptors */
    static cy_stc_dma_descriptor_t USBFS_ep7DmaDescr0;
    
    
    /* DMA channel configuration */
    static cy_stc_usbfs_dev_drv_dma_config_t const USBFS_ep7dmaConfig =
    {
        .base        = USBFS_ep7_dma_HW,
        .chNum       = USBFS_ep7_dma_DW_CHANNEL,
        .priority    = USBFS_ep7_dma_PRIORITY,
        .preemptable = USBFS_ep7_dma_PREEMPTABLE,
        .descr0      = &USBFS_ep7DmaDescr0,
        .descr1      = NULL,
        .outTrigMux  = (uint32_t) USBFS_USBFS__DMA_BURSTEND_6,      
    };
#endif /* (USBFS_EP7_DMA_CH_ENABLED) */

#if (USBFS_EP8_DMA_CH_ENABLED)
    /* DMA Descriptors */
    static cy_stc_dma_descriptor_t USBFS_ep8DmaDescr0;
    
    /* DMA channel configuration */
    static cy_stc_usbfs_dev_drv_dma_config_t const USBFS_ep8dmaConfig =
    {
        .base        = USBFS_ep8_dma_HW,
        .chNum       = USBFS_ep8_dma_DW_CHANNEL,
        .priority    = USBFS_ep8_dma_PRIORITY,
        .preemptable = USBFS_ep8_dma_PREEMPTABLE,
        .descr0      = &USBFS_ep8DmaDescr0,
        .descr1      = NULL,
        .outTrigMux  = (uint32_t) USBFS_USBFS__DMA_BURSTEND_7,
    };
#endif /* (USBFS_EP8_DMA_CH_ENABLED) */

/** 
* USBFS_initVar indicates whether the USBFS
* component has been initialized. The variable is initialized to 0
* and set to 1 the first time USBFS_Start() is called.
* This allows  the component to restart without reinitialization
* after the first call to the USBFS_Start() routine.
*/
uint8_t USBFS_initVar = 0U;

/** 
* The instance-specific configuration structure.
* The pointer to this structure should be passed to Cy_SCB_UART_Init function
* to initialize component with GUI selected settings.
*/

/** USBFS Device Driver configuration structure */
cy_stc_usbfs_dev_drv_config_t const USBFS_drvConfig =
{
    .mode = CY_USBFS_DEV_DRV_EP_MANAGEMENT_CPU,
    
    .epBuffer     = NULL,
    .epBufferSize = 0U,

    .dmaConfig[0] = NULL,
    .dmaConfig[1] = NULL,
    .dmaConfig[2] = NULL,
    .dmaConfig[3] = NULL,
    .dmaConfig[4] = NULL,
    .dmaConfig[5] = NULL,
    .dmaConfig[6] = NULL,
    .dmaConfig[7] = NULL,
    
    .epAccess     = CY_USBFS_DEV_DRV_USE_8_BITS_DR,
    .intrLevelSel = USBFS_INTR_LEVEL_SEL,    
    .enableLpm    = false,
};

/* USBFS Device Driver context */
cy_stc_usbfs_dev_drv_context_t USBFS_drvContext;

/* USB Device context */
cy_stc_usb_dev_context_t USBFS_devContext;

#if (USBFS_AUDIO_CLASS_SUPPORTED)
    /* Audio Class configuration and context structures */
    cy_stc_usb_dev_audio_context_t USBFS_audioContext;
#endif  

#if (USBFS_CDC_CLASS_SUPPORTED)
    /* CDC Class context structures */
    cy_stc_usb_dev_cdc_context_t USBFS_cdcContext;
#endif

#if (USBFS_HID_CLASS_SUPPORTED)
    /* HID Class context structures */
    cy_stc_usb_dev_hid_context_t USBFS_hidContext;
#endif


/*******************************************************************************
* Function Name: USBFS_Start
****************************************************************************//**
*
* This function performs all required initialization for the USBFS component.
* After this function call, the USB device initiates communication with the
* host by pull-up D+ line. This is the preferred method to begin component
* operation.
*
* \param device
* Contains the device number of the desired device descriptor. 
*
* param blocking
* Define if the function is blocking or non-blocking.
*
* \note
* Global interrupts have to be enabled because interrupts are required for 
* USBFS component operation.
*
*******************************************************************************/
void USBFS_Start(uint32_t device, bool blocking)
{
    /* Check that device exits */
    CY_ASSERT_L1(device < (uint32_t) USBFS_NUM_DEVICES);
    
    if (0U == USBFS_initVar)
    {
        USBFS_initVar = 1U;
        
        /* Initialize USB Device */
        (void) Cy_USB_Dev_Init( USBFS_HW, 
                               &USBFS_drvConfig, 
                               &USBFS_drvContext,
                               &USBFS_devices[device], 
                               &USBFS_devConfig,
                               &USBFS_devContext);
        
    #if (USBFS_AUDIO_CLASS_SUPPORTED)
        /* Add Audio Class support */
        (void) Cy_USB_Dev_Audio_Init(NULL,
                                     &USBFS_audioContext,
                                     &USBFS_devContext);
    #endif
        
    #if (USBFS_CDC_CLASS_SUPPORTED)
        /* Add CDC Class support */
        (void) Cy_USB_Dev_CDC_Init(&USBFS_cdcConfig,
                                   &USBFS_cdcContext,
                                   &USBFS_devContext);
    #endif
        
    #if (USBFS_HID_CLASS_SUPPORTED)
        /* Add HID Class support */
        (void) Cy_USB_Dev_HID_Init(&USBFS_hidConfig,
                                   &USBFS_hidContext,
                                   &USBFS_devContext);
    #endif
        
        /* Initialize interrupts */
    #if defined(USBFS_LowPriorityInterrupt__INTC_ASSIGNED)
        (void) Cy_SysInt_Init(&USBFS_LowPriorityInterrupt_cfg, 
                              &USBFS_InterruptLow);
    #endif /* (USBFS_SCB_IRQ__INTC_ASSIGNED) */

    #if defined(USBFS_MediumPriorityInterrupt__INTC_ASSIGNED)
        (void) Cy_SysInt_Init(&USBFS_MediumPriorityInterrupt_cfg, 
                              &USBFS_InterruptMedium);
    #endif /* (USBFS_SCB_IRQ__INTC_ASSIGNED) */

    #if defined(USBFS_HighPriorityInterrupt__INTC_ASSIGNED)
        (void) Cy_SysInt_Init(&USBFS_HighPriorityInterrupt_cfg, 
                              &USBFS_InterruptHigh);
    #endif /* (USBFS_SCB_IRQ__INTC_ASSIGNED) */
    }
    
    /* Enable interrupts */
#if defined(USBFS_LowPriorityInterrupt__INTC_ASSIGNED)
    NVIC_EnableIRQ((IRQn_Type) USBFS_LowPriorityInterrupt_cfg.intrSrc);
#endif /* (USBFS_SCB_IRQ__INTC_ASSIGNED) */

#if defined(USBFS_MediumPriorityInterrupt__INTC_ASSIGNED)
    NVIC_EnableIRQ((IRQn_Type) USBFS_MediumPriorityInterrupt_cfg.intrSrc);
#endif /* (USBFS_SCB_IRQ__INTC_ASSIGNED) */

#if defined(USBFS_HighPriorityInterrupt__INTC_ASSIGNED)
    NVIC_EnableIRQ((IRQn_Type) USBFS_HighPriorityInterrupt_cfg.intrSrc);
#endif /* (USBFS_SCB_IRQ__INTC_ASSIGNED) */

    /* Connect USB Device on the bus */
    (void) Cy_USB_Dev_Connect(blocking, CY_USB_DEV_WAIT_FOREVER, &USBFS_devContext);
}


#if defined(__cplusplus)
}
#endif


/* [] END OF FILE */
