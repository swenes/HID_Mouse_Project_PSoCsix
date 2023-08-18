/***************************************************************************//**
* \file USBFS_cfg.h
* \version 1.0
*
* This file provides device descriptors and middleware configuration structures 
* for the USBFS Device Component.
*
********************************************************************************
* \copyright
* Copyright 2018, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "USBFS_cfg.h"



#if defined(USBFS_USB_CORE) && (USBFS_USB_CORE == __CORTEX_M)

/*******************************************************************************
*                       Device Descriptors Initialization
*******************************************************************************/

static uint8_t const USBFS_deviceDescriptors[] = 
{
/******************************************************************************/
/* Device Descriptor                                                          */
/******************************************************************************/
/* bLength */                                                             0x12U, 
/* bDescriptorType */                                                     0x01U, 
/* bcdUSB */                                                       0x00U, 0x02U, 
/* bDeviceClass */                                                        0x00U, 
/* bDeviceSubClass */                                                     0x00U, 
/* bDeviceProtocol */                                                     0x00U, 
/* bMaxPacketSize */                                                      0x08U, 
/* idVendor */                                                     0xB4U, 0x04U, 
/* idProduct */                                                    0x7BU, 0xE1U, 
/* bcdDevice */                                                    0x00U, 0x00U, 
/* iManufacturer */                                                       0x01U, 
/* iProduct */                                                            0x02U, 
/* iSerialNumber */                                                       0x00U, 
/* bNumConfigurations */                                                  0x01U, 
};


static uint8_t const USBFS_configurationDescriptors[] = 
{
/******************************************************************************/
/* Configuration Descriptor                                                   */
/******************************************************************************/
/* bLength */                                                             0x09U, 
/* bDescriptorType */                                                     0x02U, 
/* wTotalLength */                                                 0x22U, 0x00U, 
/* bNumInterfaces */                                                      0x01U, 
/* bConfigurationValue */                                                 0x01U, 
/* iConfiguration */                                                      0x00U, 
/* bmAttributes */                                                        0x80U, 
/* bMaxPower */                                                           0x19U, 

/******************************************************************************/
/* HID Alternate Settings                                                     */
/******************************************************************************/
/* bLength */                                                             0x09U, 
/* bDescriptorType */                                                     0x04U, 
/* bInterfaceNumber */                                                    0x00U, 
/* bAlternateSetting */                                                   0x00U, 
/* bNumEndpoints */                                                       0x01U, 
/* bInterfaceClass */                                                     0x03U, 
/* bInterfaceSubClass */                                                  0x00U, 
/* bInterfaceProtocol */                                                  0x00U, 
/* iInterface */                                                          0x00U, 

/******************************************************************************/
/* HID Descriptor                                                             */
/******************************************************************************/
/* bLength */                                                             0x09U, 
/* bDescriptorType */                                                     0x21U, 
/* bcdHID */                                                       0x11U, 0x01U, 
/* bCountryCode */                                                        0x00U, 
/* bNumDescriptors */                                                     0x01U, 
/* bDescriptorType(Report) */                                             0x22U, 
/* wDescriptorLength */                                            0x32U, 0x00U, 

/******************************************************************************/
/* Endpoint Descriptor                                                        */
/******************************************************************************/
/* bLength */                                                             0x07U, 
/* bDescriptorType */                                                     0x05U, 
/* bEndpointAddress */                                                    0x81U, 
/* bmAttributes */                                                        0x03U, 
/* wMaxPacketSize */                                               0x08U, 0x00U, 
/* bInterval */                                                           0x0AU, 
};


static uint8_t const USBFS_stringDescriptors[] = 
{
/******************************************************************************/
/* Language ID Descriptor                                                     */
/******************************************************************************/
/* bLength */                                                             0x04U, 
/* bDescriptorType */                                                     0x03U, 
/* wLANGID */                                                      0x09U, 0x04U, 

/******************************************************************************/
/* String Descriptor                                                          */
/******************************************************************************/
/* bLength */                                                             0x2CU, 
/* bDescriptorType */                                                     0x03U, 
/* bString */ 
    (uint8_t)'C', 0U, (uint8_t)'y', 0U, (uint8_t)'p', 0U, (uint8_t)'r', 0U,
    (uint8_t)'e', 0U, (uint8_t)'s', 0U, (uint8_t)'s', 0U, (uint8_t)' ', 0U,
    (uint8_t)'S', 0U, (uint8_t)'e', 0U, (uint8_t)'m', 0U, (uint8_t)'i', 0U,
    (uint8_t)'c', 0U, (uint8_t)'o', 0U, (uint8_t)'n', 0U, (uint8_t)'d', 0U,
    (uint8_t)'u', 0U, (uint8_t)'c', 0U, (uint8_t)'t', 0U, (uint8_t)'o', 0U,
    (uint8_t)'r', 0U,

/******************************************************************************/
/* String Descriptor                                                          */
/******************************************************************************/
/* bLength */                                                             0x0CU, 
/* bDescriptorType */                                                     0x03U, 
/* bString */ 
    (uint8_t)'M', 0U, (uint8_t)'o', 0U, (uint8_t)'u', 0U, (uint8_t)'s', 0U,
    (uint8_t)'e', 0U,
};


static uint8_t const USBFS_hidReportDescriptors[] = 
{
/* USAGE_PAGE */                                                   0x05U, 0x01U, 
/* USAGE */                                                        0x09U, 0x02U, 
/* COLLECTION */                                                   0xA1U, 0x01U, 
/* USAGE */                                                        0x09U, 0x01U, 
/* COLLECTION */                                                   0xA1U, 0x00U, 
/* USAGE_PAGE */                                                   0x05U, 0x09U, 
/* USAGE_MINIMUM */                                                0x19U, 0x01U, 
/* USAGE_MAXIMUM */                                                0x29U, 0x03U, 
/* LOGICAL_MINIMUM */                                              0x15U, 0x00U, 
/* LOGICAL_MAXIMUM */                                              0x25U, 0x01U, 
/* REPORT_COUNT */                                                 0x95U, 0x03U, 
/* REPORT_SIZE */                                                  0x75U, 0x01U, 
/* INPUT */                                                        0x81U, 0x02U, 
/* REPORT_COUNT */                                                 0x95U, 0x01U, 
/* REPORT_SIZE */                                                  0x75U, 0x05U, 
/* INPUT */                                                        0x81U, 0x01U, 
/* USAGE_PAGE */                                                   0x05U, 0x01U, 
/* USAGE */                                                        0x09U, 0x30U, 
/* USAGE */                                                        0x09U, 0x31U, 
/* LOGICAL_MINIMUM */                                              0x15U, 0x81U, 
/* LOGICAL_MAXIMUM */                                              0x25U, 0x7FU, 
/* REPORT_SIZE */                                                  0x75U, 0x08U, 
/* REPORT_COUNT */                                                 0x95U, 0x02U, 
/* INPUT */                                                        0x81U, 0x06U, 
/* END_COLLECTION */                                                      0xC0U, 
/* END_COLLECTION */                                                      0xC0U, 
};


/*******************************************************************************
*                       Device Structures Initialization
*******************************************************************************/

/* Endpoints array initialization */
static const cy_stc_usb_dev_endpoint_t USBFS_endpoints[] = 
{
    {
        .endpointDescriptor = &USBFS_configurationDescriptors[27], 
    },
};

/* Pointers to endpoints array initialization */
static const cy_stc_usb_dev_endpoint_t* USBFS_endpointsPtr[] = 
{
    &USBFS_endpoints[0],
};

static const uint8_t USBFS_hidReportIdx[] =
{
    1,
};

/* HID array initialization */
static const cy_stc_usb_dev_hid_t  USBFS_hid[] = 
{
    {
        .hidDescriptor        = &USBFS_configurationDescriptors[18], 
        .reportDescriptor     = &USBFS_hidReportDescriptors[0],
        .reportDescriptorSize = 50,
        .inputReportPos       = 0U,
        .numInputReports      = 1U,
        .inputReportIdx       = &USBFS_hidReportIdx[0],
        .inputReportIdxSize   = 1U,
    },
};

/* Alternate settings array initialization */
static const cy_stc_usb_dev_alternate_t USBFS_alternates[] = 
{
    {
        .interfaceDescriptor = &USBFS_configurationDescriptors[9],
        .numEndpoints        = 1,
        .endpoints           = &USBFS_endpointsPtr[0],
        .hid                 = &USBFS_hid[0],
    },
};

/* Pointers to alternates array initialization */
static const cy_stc_usb_dev_alternate_t* USBFS_alternatesPtr[] = 
{
    &USBFS_alternates[0],
};

/* Interfaces array initialization */
static const cy_stc_usb_dev_interface_t USBFS_interfaces[] =
{
    {
        .numAlternates = 1U,
        .alternates    = &USBFS_alternatesPtr[0],
        .endpointsMask = 0x0001U,
    },
};

/* Pointers to interfaces array initialization */
static const cy_stc_usb_dev_interface_t* USBFS_interfacesPtr[] = 
{
    &USBFS_interfaces[0],
};

/* Configurations array initialization */
static const cy_stc_usb_dev_configuration_t  USBFS_configurations[] =
{
    {
        .configDescriptor = &USBFS_configurationDescriptors[0],
        .numInterfaces    = 1U,
        .interfaces       = &USBFS_interfacesPtr[0],
    },
};

/* Pointers to configurations array initialization */
static const cy_stc_usb_dev_configuration_t* USBFS_configurationsPtr[] = 
{
    &USBFS_configurations[0],
};

/* Pointers to Strings array initialization */
static const uint8_t* USBFS_stringPtr[] = 
{
    &USBFS_stringDescriptors[0],
    &USBFS_stringDescriptors[4],
    &USBFS_stringDescriptors[48],

};


/* String array initialization */
static const cy_stc_usb_dev_string_t  USBFS_strings[] =
{
    {
        .numStrings                = 3U,
        .stringDescriptors         = &USBFS_stringPtr[0],
        .osStringDescriptors       = NULL,
    },
};

/* Device array initialization */
const cy_stc_usb_dev_device_t USBFS_devices[] =
{
    {
        .deviceDescriptor  = &USBFS_deviceDescriptors[0],
        .bosDescriptor     = NULL, 
        .strings           = &USBFS_strings[0], 
        .numConfigurations = 1U, 
        .configurations    = &USBFS_configurationsPtr[0], 
    },
};

/* HID configuration */
static uint8_t idleTimers[2U * 1U];
const cy_stc_usb_dev_hid_config_t USBFS_hidConfig = 
{
    .timers    = idleTimers,
    .timersNum = 1U,
};


/* Device configuration */
static uint8_t endpoint0Buffer[64U];
const cy_stc_usb_dev_config_t USBFS_devConfig = 
{
    .ep0Buffer = endpoint0Buffer,
    .ep0BufferSize = 64U,
};

#endif /* defined(USBFS_USB_CORE) && (USBFS_USB_CORE == __CORTEX_M) */

/* [] END OF FILE */

