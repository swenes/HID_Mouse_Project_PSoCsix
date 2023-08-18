/***************************************************************************//**
* \file USBFS_cfg.h
* \version 1.0
*
* This file provides interface to access device descriptors and middleware 
* configuration structures for the USBFS Device Component.
*
********************************************************************************
* \copyright
* Copyright 2018, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(USBFS_CY_USBFS_DEV_PDL_CFG_H)
#define USBFS_CY_USBFS_DEV_PDL_CFG_H
    


#include <stddef.h>
#include "cy_device_headers.h"

#if defined(CY_PSOC_CREATOR_USED)
    #include "USBFS.h"

    /* Enable code for both cores */
    #define USBFS_USB_CORE __CORTEX_M

    /* Endpoint Buffer Size */
    #define USBFS_ENDPOINTS_BUFFER_SIZE 8U

    #if (USBFS_ACTIVE_ENDPOINTS_MASK != 0x1)
    #error The USB Device peripheral parameter Endpoint Mask expected value is 0x1. Change Endpoint Mask to this value or run the USB Configurator using PSoC Creator and adjust the number of active endpoints.
    #endif

#else
    #include "cycfg_peripherals.h"

    #if !defined(USBFS_HW)
        #error The USB Device peripheral Alias expected name is 'USBFS'. Change Alias to 'USBFS' or run the USB Configurator using ModusToolbox and save configuration.
    #else
        #if !defined(USBFS_USB_CORE)
            #error The target core is not selected for USB Device Middleware operation in the cycfg_peripherals.h: missing #define USBFS_USB_CORE.
        #endif

        #if defined(USBFS_USB_CORE) && (USBFS_USB_CORE == __CORTEX_M)

            #if !defined(USBFS_ACTIVE_ENDPOINTS_MASK)
            #error The active endpoint mask is not set in the cycfg_peripherals.h: missing #define USBFS_ACTIVE_ENDPOINTS_MASK.
            #endif

            #if !defined(USBFS_ENDPOINTS_BUFFER_SIZE)
            #error The endpoints buffer size is not set in the cycfg_peripherals.h: missing #define USBFS_ENDPOINTS_BUFFER_SIZE.
            #endif

            #if defined(USBFS_ACTIVE_ENDPOINTS_MASK) && defined(USBFS_ENDPOINTS_BUFFER_SIZE)

                #if (USBFS_ACTIVE_ENDPOINTS_MASK != 0x1)
                #error The USB Device peripheral configuration and USB Device Descriptors are out of synchronization. Run the USB Configurator using ModusToolbox and save the configuration.
                #endif

                #if (USBFS_ENDPOINTS_BUFFER_SIZE != 8)
                #error The USB Device peripheral configuration and USB Device Descriptors are out of synchronization. Run the USB Configurator using ModusToolbox and save the configuration.
                #endif

            #endif

        #endif /* defined(USBFS_USB_CORE) && (USBFS_USB_CORE == __CORTEX_M) */

    #endif /* !defined(USBFS_HW) */

#endif /* defined(CY_PSOC_CREATOR_USED) */

#if defined(USBFS_USB_CORE) && (USBFS_USB_CORE == __CORTEX_M)
    
#if defined(__cplusplus)
extern "C" {
#endif

#include "cy_usb_dev.h"
#include "cy_usb_dev_descr.h"
#include "cy_usb_dev_hid.h"
#include "cy_usb_dev_cdc.h"

/* Number of USB Device */
#define USBFS_NUM_DEVICES           1

/* Class specific defines */
#define USBFS_AUDIO_CLASS_SUPPORTED 0U
#define USBFS_CDC_CLASS_SUPPORTED   0U
#define USBFS_HID_CLASS_SUPPORTED   1U


/* Array of USB Devices */
extern const cy_stc_usb_dev_device_t USBFS_devices[USBFS_NUM_DEVICES];

/* Device configuration */
extern const cy_stc_usb_dev_config_t USBFS_devConfig;
/* HID configuration */
extern const cy_stc_usb_dev_hid_config_t USBFS_hidConfig;


/*
USB_CONFIG_START
<Configuration major="1" minor="0">
    <Node type="root">
        <Node type="device">
            <Field name="bcdUSB" value="0x200"/>
            <Field name="bDeviceClass" value="0x00"/>
            <Field name="bDeviceSubClass" value="0x00"/>
            <Field name="bDeviceProtocol" value="0"/>
            <Field name="idVendor" value="0x4B4"/>
            <Field name="idProduct" value="0xE17B"/>
            <Field name="bcdDevice" value="0"/>
            <Field name="iManufacturer" value="Cypress Semiconductor"/>
            <Field name="iProduct" value="Mouse"/>
            <Field name="iSerialNumber" value="Empty"/>
            <Node type="configuration">
                <Field name="iConfiguration" value=""/>
                <Field name="Self Powered" value="Disable"/>
                <Field name="Remote Wakeup" value="Disable"/>
                <Field name="bMaxPower" value="50"/>
                <Node type="interface">
                    <Node type="alternate.hid">
                        <Field name="bInterfaceSubClass" value="0"/>
                        <Field name="bInterfaceProtocol" value="0"/>
                        <Field name="iInterface" value=""/>
                        <Node type="hid">
                            <Field name="bCountryCode" value="0x00"/>
                            <Field name="Report" value="0x05;0x01;0x09;0x02;0xA1;0x01;0x09;0x01;0xA1;0x00;0x05;0x09;0x19;0x01;0x29;0x03;0x15;0x00;0x25;0x01;0x95;0x03;0x75;0x01;0x81;0x02;0x95;0x01;0x75;0x05;0x81;0x01;0x05;0x01;0x09;0x30;0x09;0x31;0x15;0x81;0x25;0x7F;0x75;0x08;0x95;0x02;0x81;0x06;0xC0;0xC0;"/>
                        </Node>
                        <Node type="endpoint">
                            <Field name="endpointNum" value="EP1"/>
                            <Field name="direction" value="IN"/>
                            <Field name="Transfer Type" value="Interrupt"/>
                            <Field name="Synchronization Type" value="No Synchronization"/>
                            <Field name="Usage Type" value="Data endpoint"/>
                            <Field name="wMaxPacketSize" value="8"/>
                            <Field name="bInterval" value="10"/>
                        </Node>
                    </Node>
                </Node>
            </Node>
        </Node>
    </Node>
</Configuration>
USB_CONFIG_END
*/

#if defined(__cplusplus)
}
#endif

#endif /* defined(USBFS_USB_CORE) && (USBFS_USB_CORE == __CORTEX_M) */


    
#endif /* USBFS_CY_USBFS_DEV_PDL_CFG_H */


/* [] END OF FILE */
