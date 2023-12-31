/*******************************************************************************
* File Name: cyfitter_gpio.h
* 
* PSoC Creator  4.4
*
* Description:
* 
* This file is automatically generated by PSoC Creator.
*
********************************************************************************
* Copyright (c) 2007-2020 Cypress Semiconductor.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#ifndef INCLUDED_CYFITTER_GPIO_H
#define INCLUDED_CYFITTER_GPIO_H
#include "cy_device_headers.h"

/* RED */
#define RED_0_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define RED_0_INBUF_ENABLED 0u
#define RED_0_INIT_DRIVESTATE 1u
#define RED_0_INIT_MUXSEL 3u
#define RED_0_INPUT_SYNC 2u
#define RED_0_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define RED_0_NUM 3u
#define RED_0_PORT GPIO_PRT0
#define RED_0_SLEWRATE CY_GPIO_SLEW_FAST
#define RED_0_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define RED_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define RED_INBUF_ENABLED 0u
#define RED_INIT_DRIVESTATE 1u
#define RED_INIT_MUXSEL 3u
#define RED_INPUT_SYNC 2u
#define RED_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define RED_NUM 3u
#define RED_PORT GPIO_PRT0
#define RED_SLEWRATE CY_GPIO_SLEW_FAST
#define RED_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS

/* BLUE */
#define BLUE_0_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define BLUE_0_INBUF_ENABLED 0u
#define BLUE_0_INIT_DRIVESTATE 1u
#define BLUE_0_INIT_MUXSEL 8u
#define BLUE_0_INPUT_SYNC 2u
#define BLUE_0_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define BLUE_0_NUM 1u
#define BLUE_0_PORT GPIO_PRT11
#define BLUE_0_SLEWRATE CY_GPIO_SLEW_FAST
#define BLUE_0_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define BLUE_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define BLUE_INBUF_ENABLED 0u
#define BLUE_INIT_DRIVESTATE 1u
#define BLUE_INIT_MUXSEL 8u
#define BLUE_INPUT_SYNC 2u
#define BLUE_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define BLUE_NUM 1u
#define BLUE_PORT GPIO_PRT11
#define BLUE_SLEWRATE CY_GPIO_SLEW_FAST
#define BLUE_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS

/* GREEN */
#define GREEN_0_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define GREEN_0_INBUF_ENABLED 0u
#define GREEN_0_INIT_DRIVESTATE 1u
#define GREEN_0_INIT_MUXSEL 3u
#define GREEN_0_INPUT_SYNC 2u
#define GREEN_0_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define GREEN_0_NUM 1u
#define GREEN_0_PORT GPIO_PRT1
#define GREEN_0_SLEWRATE CY_GPIO_SLEW_FAST
#define GREEN_0_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define GREEN_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define GREEN_INBUF_ENABLED 0u
#define GREEN_INIT_DRIVESTATE 1u
#define GREEN_INIT_MUXSEL 3u
#define GREEN_INPUT_SYNC 2u
#define GREEN_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define GREEN_NUM 1u
#define GREEN_PORT GPIO_PRT1
#define GREEN_SLEWRATE CY_GPIO_SLEW_FAST
#define GREEN_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS

/* USBFS_Dm */
#define USBFS_Dm_0_DRIVEMODE CY_GPIO_DM_ANALOG
#define USBFS_Dm_0_INBUF_ENABLED 0u
#define USBFS_Dm_0_INIT_DRIVESTATE 1u
#define USBFS_Dm_0_INIT_MUXSEL 0u
#define USBFS_Dm_0_INPUT_SYNC 2u
#define USBFS_Dm_0_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define USBFS_Dm_0_NUM 1u
#define USBFS_Dm_0_PORT GPIO_PRT14
#define USBFS_Dm_0_SLEWRATE CY_GPIO_SLEW_FAST
#define USBFS_Dm_0_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define USBFS_Dm_DRIVEMODE CY_GPIO_DM_ANALOG
#define USBFS_Dm_INBUF_ENABLED 0u
#define USBFS_Dm_INIT_DRIVESTATE 1u
#define USBFS_Dm_INIT_MUXSEL 0u
#define USBFS_Dm_INPUT_SYNC 2u
#define USBFS_Dm_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define USBFS_Dm_NUM 1u
#define USBFS_Dm_PORT GPIO_PRT14
#define USBFS_Dm_SLEWRATE CY_GPIO_SLEW_FAST
#define USBFS_Dm_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS

/* USBFS_Dp */
#define USBFS_Dp_0_DRIVEMODE CY_GPIO_DM_ANALOG
#define USBFS_Dp_0_INBUF_ENABLED 0u
#define USBFS_Dp_0_INIT_DRIVESTATE 1u
#define USBFS_Dp_0_INIT_MUXSEL 0u
#define USBFS_Dp_0_INPUT_SYNC 2u
#define USBFS_Dp_0_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define USBFS_Dp_0_NUM 0u
#define USBFS_Dp_0_PORT GPIO_PRT14
#define USBFS_Dp_0_SLEWRATE CY_GPIO_SLEW_FAST
#define USBFS_Dp_0_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define USBFS_Dp_DRIVEMODE CY_GPIO_DM_ANALOG
#define USBFS_Dp_INBUF_ENABLED 0u
#define USBFS_Dp_INIT_DRIVESTATE 1u
#define USBFS_Dp_INIT_MUXSEL 0u
#define USBFS_Dp_INPUT_SYNC 2u
#define USBFS_Dp_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define USBFS_Dp_NUM 0u
#define USBFS_Dp_PORT GPIO_PRT14
#define USBFS_Dp_SLEWRATE CY_GPIO_SLEW_FAST
#define USBFS_Dp_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS

/* ModeChange */
#define ModeChange_0_DRIVEMODE CY_GPIO_DM_HIGHZ
#define ModeChange_0_INBUF_ENABLED 1u
#define ModeChange_0_INIT_DRIVESTATE 0u
#define ModeChange_0_INIT_MUXSEL 0u
#define ModeChange_0_INPUT_SYNC 2u
#define ModeChange_0_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define ModeChange_0_NUM 4u
#define ModeChange_0_PORT GPIO_PRT0
#define ModeChange_0_SLEWRATE CY_GPIO_SLEW_FAST
#define ModeChange_0_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define ModeChange_DRIVEMODE CY_GPIO_DM_HIGHZ
#define ModeChange_INBUF_ENABLED 1u
#define ModeChange_INIT_DRIVESTATE 0u
#define ModeChange_INIT_MUXSEL 0u
#define ModeChange_INPUT_SYNC 2u
#define ModeChange_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define ModeChange_NUM 4u
#define ModeChange_PORT GPIO_PRT0
#define ModeChange_SLEWRATE CY_GPIO_SLEW_FAST
#define ModeChange_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS

/* CapSense_Rx */
#define CapSense_Rx_0_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_Rx_0_INBUF_ENABLED 0u
#define CapSense_Rx_0_INIT_DRIVESTATE 1u
#define CapSense_Rx_0_INIT_MUXSEL 4u
#define CapSense_Rx_0_INPUT_SYNC 2u
#define CapSense_Rx_0_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_Rx_0_NUM 1u
#define CapSense_Rx_0_PORT GPIO_PRT8
#define CapSense_Rx_0_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_Rx_0_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define CapSense_Rx_1_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_Rx_1_INBUF_ENABLED 0u
#define CapSense_Rx_1_INIT_DRIVESTATE 1u
#define CapSense_Rx_1_INIT_MUXSEL 4u
#define CapSense_Rx_1_INPUT_SYNC 2u
#define CapSense_Rx_1_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_Rx_1_NUM 2u
#define CapSense_Rx_1_PORT GPIO_PRT8
#define CapSense_Rx_1_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_Rx_1_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS

/* CapSense_Tx */
#define CapSense_Tx_0_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_Tx_0_INBUF_ENABLED 0u
#define CapSense_Tx_0_INIT_DRIVESTATE 1u
#define CapSense_Tx_0_INIT_MUXSEL 0u
#define CapSense_Tx_0_INPUT_SYNC 2u
#define CapSense_Tx_0_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_Tx_0_NUM 0u
#define CapSense_Tx_0_PORT GPIO_PRT1
#define CapSense_Tx_0_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_Tx_0_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define CapSense_Tx_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_Tx_INBUF_ENABLED 0u
#define CapSense_Tx_INIT_DRIVESTATE 1u
#define CapSense_Tx_INIT_MUXSEL 0u
#define CapSense_Tx_INPUT_SYNC 2u
#define CapSense_Tx_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_Tx_NUM 0u
#define CapSense_Tx_PORT GPIO_PRT1
#define CapSense_Tx_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_Tx_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS

/* CapSense_Sns */
#define CapSense_Sns_0_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_Sns_0_INBUF_ENABLED 0u
#define CapSense_Sns_0_INIT_DRIVESTATE 1u
#define CapSense_Sns_0_INIT_MUXSEL 0u
#define CapSense_Sns_0_INPUT_SYNC 2u
#define CapSense_Sns_0_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_Sns_0_NUM 3u
#define CapSense_Sns_0_PORT GPIO_PRT8
#define CapSense_Sns_0_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_Sns_0_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define CapSense_Sns_1_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_Sns_1_INBUF_ENABLED 0u
#define CapSense_Sns_1_INIT_DRIVESTATE 1u
#define CapSense_Sns_1_INIT_MUXSEL 0u
#define CapSense_Sns_1_INPUT_SYNC 2u
#define CapSense_Sns_1_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_Sns_1_NUM 4u
#define CapSense_Sns_1_PORT GPIO_PRT8
#define CapSense_Sns_1_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_Sns_1_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define CapSense_Sns_2_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_Sns_2_INBUF_ENABLED 0u
#define CapSense_Sns_2_INIT_DRIVESTATE 1u
#define CapSense_Sns_2_INIT_MUXSEL 0u
#define CapSense_Sns_2_INPUT_SYNC 2u
#define CapSense_Sns_2_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_Sns_2_NUM 5u
#define CapSense_Sns_2_PORT GPIO_PRT8
#define CapSense_Sns_2_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_Sns_2_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define CapSense_Sns_3_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_Sns_3_INBUF_ENABLED 0u
#define CapSense_Sns_3_INIT_DRIVESTATE 1u
#define CapSense_Sns_3_INIT_MUXSEL 0u
#define CapSense_Sns_3_INPUT_SYNC 2u
#define CapSense_Sns_3_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_Sns_3_NUM 6u
#define CapSense_Sns_3_PORT GPIO_PRT8
#define CapSense_Sns_3_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_Sns_3_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define CapSense_Sns_4_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_Sns_4_INBUF_ENABLED 0u
#define CapSense_Sns_4_INIT_DRIVESTATE 1u
#define CapSense_Sns_4_INIT_MUXSEL 0u
#define CapSense_Sns_4_INPUT_SYNC 2u
#define CapSense_Sns_4_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_Sns_4_NUM 7u
#define CapSense_Sns_4_PORT GPIO_PRT8
#define CapSense_Sns_4_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_Sns_4_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS

/* CapSense_Cmod */
#define CapSense_Cmod_0_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_Cmod_0_INBUF_ENABLED 0u
#define CapSense_Cmod_0_INIT_DRIVESTATE 1u
#define CapSense_Cmod_0_INIT_MUXSEL 0u
#define CapSense_Cmod_0_INPUT_SYNC 2u
#define CapSense_Cmod_0_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_Cmod_0_NUM 7u
#define CapSense_Cmod_0_PORT GPIO_PRT7
#define CapSense_Cmod_0_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_Cmod_0_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define CapSense_Cmod_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_Cmod_INBUF_ENABLED 0u
#define CapSense_Cmod_INIT_DRIVESTATE 1u
#define CapSense_Cmod_INIT_MUXSEL 0u
#define CapSense_Cmod_INPUT_SYNC 2u
#define CapSense_Cmod_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_Cmod_NUM 7u
#define CapSense_Cmod_PORT GPIO_PRT7
#define CapSense_Cmod_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_Cmod_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS

/* CapSense_CintA */
#define CapSense_CintA_0_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_CintA_0_INBUF_ENABLED 0u
#define CapSense_CintA_0_INIT_DRIVESTATE 1u
#define CapSense_CintA_0_INIT_MUXSEL 0u
#define CapSense_CintA_0_INPUT_SYNC 2u
#define CapSense_CintA_0_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_CintA_0_NUM 1u
#define CapSense_CintA_0_PORT GPIO_PRT7
#define CapSense_CintA_0_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_CintA_0_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define CapSense_CintA_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_CintA_INBUF_ENABLED 0u
#define CapSense_CintA_INIT_DRIVESTATE 1u
#define CapSense_CintA_INIT_MUXSEL 0u
#define CapSense_CintA_INPUT_SYNC 2u
#define CapSense_CintA_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_CintA_NUM 1u
#define CapSense_CintA_PORT GPIO_PRT7
#define CapSense_CintA_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_CintA_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS

/* CapSense_CintB */
#define CapSense_CintB_0_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_CintB_0_INBUF_ENABLED 0u
#define CapSense_CintB_0_INIT_DRIVESTATE 1u
#define CapSense_CintB_0_INIT_MUXSEL 0u
#define CapSense_CintB_0_INPUT_SYNC 2u
#define CapSense_CintB_0_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_CintB_0_NUM 2u
#define CapSense_CintB_0_PORT GPIO_PRT7
#define CapSense_CintB_0_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_CintB_0_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS
#define CapSense_CintB_DRIVEMODE CY_GPIO_DM_ANALOG
#define CapSense_CintB_INBUF_ENABLED 0u
#define CapSense_CintB_INIT_DRIVESTATE 1u
#define CapSense_CintB_INIT_MUXSEL 0u
#define CapSense_CintB_INPUT_SYNC 2u
#define CapSense_CintB_INTERRUPT_MODE CY_GPIO_INTR_DISABLE
#define CapSense_CintB_NUM 2u
#define CapSense_CintB_PORT GPIO_PRT7
#define CapSense_CintB_SLEWRATE CY_GPIO_SLEW_FAST
#define CapSense_CintB_THRESHOLD_LEVEL CY_GPIO_VTRIP_CMOS

#endif /* INCLUDED_CYFITTER_GPIO_H */
