/***************************************************************************//**
* \file main_cm0p.c
* \version 1.0
*
* \brief
* Objective:
*    This example demonstrates how to configure the USB block in a PSoC® 6 MCU 
*    as a Human Interface Device (HID). The device enumerates as a 3-button 
*    mouse.
*
* Compatible Kits:
*    CY8CKIT-062-WIFI-BT
*
* \note
* To connect USB Device (PSoC6) to the USB Host use USB receptacle J28 for 
* CY8CKIT-062-WIFI-BT
*
********************************************************************************
* \copyright
* Copyright 2017-2019, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/
#include "project.h"

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    for(;;)
    {
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }
}

/* [] END OF FILE */
