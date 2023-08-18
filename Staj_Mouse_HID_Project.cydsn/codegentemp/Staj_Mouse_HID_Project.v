// ======================================================================
// Staj_Mouse_HID_Project.v generated from TopDesign.cysch
// 08/18/2023 at 16:26
// This file is auto generated. ANY EDITS YOU MAKE MAY BE LOST WHEN THIS FILE IS REGENERATED!!!
// ======================================================================

`define CYDEV_CHIP_FAMILY_PSOC3 1
`define CYDEV_CHIP_FAMILY_PSOC4 2
`define CYDEV_CHIP_FAMILY_PSOC5 3
`define CYDEV_CHIP_FAMILY_PSOC6 4
`define CYDEV_CHIP_FAMILY_FM0P 5
`define CYDEV_CHIP_FAMILY_FM3 6
`define CYDEV_CHIP_FAMILY_FM4 7
`define CYDEV_CHIP_FAMILY_UNKNOWN 0
`define CYDEV_CHIP_MEMBER_UNKNOWN 0
`define CYDEV_CHIP_MEMBER_3A 1
`define CYDEV_CHIP_REVISION_3A_PRODUCTION 3
`define CYDEV_CHIP_REVISION_3A_ES3 3
`define CYDEV_CHIP_REVISION_3A_ES2 1
`define CYDEV_CHIP_REVISION_3A_ES1 0
`define CYDEV_CHIP_MEMBER_5B 2
`define CYDEV_CHIP_REVISION_5B_PRODUCTION 0
`define CYDEV_CHIP_REVISION_5B_ES0 0
`define CYDEV_CHIP_MEMBER_5A 3
`define CYDEV_CHIP_REVISION_5A_PRODUCTION 1
`define CYDEV_CHIP_REVISION_5A_ES1 1
`define CYDEV_CHIP_REVISION_5A_ES0 0
`define CYDEV_CHIP_MEMBER_4G 4
`define CYDEV_CHIP_REVISION_4G_PRODUCTION 17
`define CYDEV_CHIP_REVISION_4G_ES 17
`define CYDEV_CHIP_REVISION_4G_ES2 33
`define CYDEV_CHIP_MEMBER_4U 5
`define CYDEV_CHIP_REVISION_4U_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4E 6
`define CYDEV_CHIP_REVISION_4E_PRODUCTION 0
`define CYDEV_CHIP_REVISION_4E_CCG2_NO_USBPD 0
`define CYDEV_CHIP_MEMBER_4X 7
`define CYDEV_CHIP_REVISION_4X_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4O 8
`define CYDEV_CHIP_REVISION_4O_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4R 9
`define CYDEV_CHIP_REVISION_4R_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4T 10
`define CYDEV_CHIP_REVISION_4T_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4N 11
`define CYDEV_CHIP_REVISION_4N_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4S 12
`define CYDEV_CHIP_REVISION_4S_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4W 13
`define CYDEV_CHIP_REVISION_4W_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4AC 14
`define CYDEV_CHIP_REVISION_4AC_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4AD 15
`define CYDEV_CHIP_REVISION_4AD_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4AE 16
`define CYDEV_CHIP_REVISION_4AE_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4Q 17
`define CYDEV_CHIP_REVISION_4Q_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4Y 18
`define CYDEV_CHIP_REVISION_4Y_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4Z 19
`define CYDEV_CHIP_REVISION_4Z_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4D 20
`define CYDEV_CHIP_REVISION_4D_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4J 21
`define CYDEV_CHIP_REVISION_4J_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4K 22
`define CYDEV_CHIP_REVISION_4K_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4V 23
`define CYDEV_CHIP_REVISION_4V_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4H 24
`define CYDEV_CHIP_REVISION_4H_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4AA 25
`define CYDEV_CHIP_REVISION_4AA_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4A 26
`define CYDEV_CHIP_REVISION_4A_PRODUCTION 17
`define CYDEV_CHIP_REVISION_4A_ES0 17
`define CYDEV_CHIP_MEMBER_4F 27
`define CYDEV_CHIP_REVISION_4F_PRODUCTION 0
`define CYDEV_CHIP_REVISION_4F_PRODUCTION_256K 0
`define CYDEV_CHIP_REVISION_4F_PRODUCTION_256DMA 0
`define CYDEV_CHIP_MEMBER_4P 28
`define CYDEV_CHIP_REVISION_4P_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4M 29
`define CYDEV_CHIP_REVISION_4M_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4AB 30
`define CYDEV_CHIP_REVISION_4AB_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4L 31
`define CYDEV_CHIP_REVISION_4L_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4I 32
`define CYDEV_CHIP_REVISION_4I_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_6A 33
`define CYDEV_CHIP_REVISION_6A_ES 17
`define CYDEV_CHIP_REVISION_6A_PRODUCTION 33
`define CYDEV_CHIP_REVISION_6A_NO_UDB 33
`define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE1 34
`define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE1_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE2 35
`define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE2_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE3 36
`define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE3_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_FM3 37
`define CYDEV_CHIP_REVISION_FM3_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_FM4 38
`define CYDEV_CHIP_REVISION_FM4_PRODUCTION 0
`define CYDEV_CHIP_FAMILY_USED 4
`define CYDEV_CHIP_MEMBER_USED 33
`define CYDEV_CHIP_REVISION_USED 33
// Component: cy_virtualmux_v1_0
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\cy_virtualmux_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\cy_virtualmux_v1_0\cy_virtualmux_v1_0.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\cy_virtualmux_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\cy_virtualmux_v1_0\cy_virtualmux_v1_0.v"
`endif

// Component: ZeroTerminal
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\ZeroTerminal"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\ZeroTerminal\ZeroTerminal.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\ZeroTerminal"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\ZeroTerminal\ZeroTerminal.v"
`endif

// bIncludeDriverToComponent_v1_0(PDL_DRIVER_NAME=dma, PDL_DRIVER_REQ_VERSION=2.0.0, PDL_DRIVER_SUBGROUP=, PDL_DRIVER_VARIANT=, CY_API_CALLBACK_HEADER_INCLUDE=#include "cyapicallbacks.h", CY_COMMENT=, CY_COMPONENT_NAME=bIncludeDriverToComponent_v1_0, CY_CONFIG_TITLE=dma, CY_CONST_CONFIG=true, CY_CONTROL_FILE=<:default:>, CY_DATASHEET_FILE=<:default:>, CY_FITTER_NAME=USBFS:dma, CY_INSTANCE_SHORT_NAME=dma, CY_MAJOR_VERSION=1, CY_MINOR_VERSION=0, CY_PDL_DRIVER_NAME=dma, CY_PDL_DRIVER_REQ_VERSION=2.0.0, CY_PDL_DRIVER_SUBGROUP=, CY_PDL_DRIVER_VARIANT=, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=PSoC Creator  4.4, INSTANCE_NAME=USBFS_dma, )
module bIncludeDriverToComponent_v1_0_0 ;





endmodule

// bIncludeDriverToComponent_v1_0(PDL_DRIVER_NAME=usb_dev, PDL_DRIVER_REQ_VERSION=1.0.0, PDL_DRIVER_SUBGROUP=, PDL_DRIVER_VARIANT=, CY_API_CALLBACK_HEADER_INCLUDE=#include "cyapicallbacks.h", CY_COMMENT=, CY_COMPONENT_NAME=bIncludeDriverToComponent_v1_0, CY_CONFIG_TITLE=usb_dev, CY_CONST_CONFIG=true, CY_CONTROL_FILE=<:default:>, CY_DATASHEET_FILE=<:default:>, CY_FITTER_NAME=USBFS:usb_dev, CY_INSTANCE_SHORT_NAME=usb_dev, CY_MAJOR_VERSION=1, CY_MINOR_VERSION=0, CY_PDL_DRIVER_NAME=usb_dev, CY_PDL_DRIVER_REQ_VERSION=1.0.0, CY_PDL_DRIVER_SUBGROUP=, CY_PDL_DRIVER_VARIANT=, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=PSoC Creator  4.4, INSTANCE_NAME=USBFS_usb_dev, )
module bIncludeDriverToComponent_v1_0_1 ;





endmodule

// USBFS_DEV_PDL_v1_0(ArbiterGroup=0, BusResetGroup=2, DmaPriority=3, EnableLpm=false, EnableSofTerminal=false, Ep0Group=2, Ep1Group=1, Ep2Group=1, Ep3Group=1, Ep4Group=1, Ep5Group=1, Ep6Group=1, Ep7Group=1, Ep8Group=1, EpAccessWidth=0, EpEnableMask=1, EpManagementDma=false, EpManagementMode=0, LpmGroup=0, RemoveEp1Dma=true, RemoveEp2Dma=true, RemoveEp3Dma=true, RemoveEp4Dma=true, RemoveEp5Dma=true, RemoveEp6Dma=true, RemoveEp7Dma=true, RemoveEp8Dma=true, SofGroup=2, CY_API_CALLBACK_HEADER_INCLUDE=#include "cyapicallbacks.h", CY_COMMENT=, CY_COMPONENT_NAME=USBFS_DEV_PDL_v1_0, CY_CONFIG_TITLE=USBFS, CY_CONST_CONFIG=true, CY_CONTROL_FILE=<:default:>, CY_DATASHEET_FILE=<:default:>, CY_FITTER_NAME=USBFS, CY_INSTANCE_SHORT_NAME=USBFS, CY_MAJOR_VERSION=1, CY_MINOR_VERSION=0, CY_PDL_DRIVER_NAME=usbfs, CY_PDL_DRIVER_REQ_VERSION=1.0.0, CY_PDL_DRIVER_SUBGROUP=, CY_PDL_DRIVER_VARIANT=, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=PSoC Creator  4.4, INSTANCE_NAME=USBFS, )
module USBFS_DEV_PDL_v1_0_2 (
    sof);
    output      sof;


          wire [7:0] dma_req;
          wire [7:0] dma_nrq;
          wire  Net_15;
          wire  Net_13;
          wire  Net_12;
          wire  Net_11;
          wire  Net_10;
          wire  Net_8;
          wire  Net_7;
          wire  Net_6;
          wire  Net_5;
          wire  Net_4;
          wire  Net_2;
          wire  Net_3;
          wire  Net_1;
          wire  Net_19;
          wire  Net_20;
          wire  Net_18;
          wire  Net_64;
          wire  Net_62;
          wire  Net_133;
          wire  Net_77;
          wire  Net_222;
          wire  Net_1567;
          wire  Net_152;
          wire  Net_162;
          wire  Net_189;
          wire  Net_186;
          wire  Net_185;
          wire  Net_188;
          wire  Net_173;
          wire  Net_1579;
          wire  Net_170;
          wire  Net_167;
          wire  Net_165;
          wire  Net_155;

    cy_mxs40_usb_v1_0 USBFS (
        .clk_usb_dev(Net_62),
        .clk_usb_host(1'b0),
        .clock_dev_brs(Net_64),
        .dma_burstend(dma_nrq[7:0]),
        .dma_req(dma_req[7:0]),
        .dsi_usb_sof(sof),
        .interrupt_hi(Net_20),
        .interrupt_lo(Net_18),
        .interrupt_med(Net_19),
        .usb_dm_pad(Net_3),
        .usb_dp_pad(Net_2));

	// ep2Dma_VM (cy_virtualmux_v1_0)
	assign dma_nrq[1] = Net_155;

    ZeroTerminal ZeroTerminal_1 (
        .z(Net_155));

	// ep1Dma_VM (cy_virtualmux_v1_0)
	assign dma_nrq[0] = Net_162;

    ZeroTerminal ZeroTerminal_2 (
        .z(Net_162));

	// ep3Dma_VM (cy_virtualmux_v1_0)
	assign dma_nrq[2] = Net_165;

    ZeroTerminal ZeroTerminal_3 (
        .z(Net_165));

	// ep4Dma_VM (cy_virtualmux_v1_0)
	assign dma_nrq[3] = Net_167;

    ZeroTerminal ZeroTerminal_4 (
        .z(Net_167));

	// ep5Dma_VM (cy_virtualmux_v1_0)
	assign dma_nrq[4] = Net_170;

    ZeroTerminal ZeroTerminal_5 (
        .z(Net_170));

	// VirtualMux_6 (cy_virtualmux_v1_0)
	assign dma_nrq[5] = Net_173;

    ZeroTerminal ep6Dma_VM (
        .z(Net_173));

	// ep7Dma_VM (cy_virtualmux_v1_0)
	assign dma_nrq[6] = Net_189;

    ZeroTerminal ZeroTerminal_7 (
        .z(Net_189));

	// ep8Dma_VM (cy_virtualmux_v1_0)
	assign dma_nrq[7] = Net_186;

    ZeroTerminal ZeroTerminal_8 (
        .z(Net_186));

	wire [0:0] tmpFB_0__Dp_net;
	wire [0:0] tmpIO_0__Dp_net;
	electrical [0:0] tmpSIOVREF__Dp_net;

	cy_mxs40_gpio_v1_0
		#(.id("729615f7-8c3c-4f82-989b-8db6cc403a70/a61270bc-07ec-447d-ac9e-34cfe85c30e9"),
		  .width(1),
		  .sio_grp_cnt(0),
		  .drive_mode("0"),
		  .ibuf_enabled("0"),
		  .init_dr_st("1"),
		  .input_sync("0"),
		  .intr_mode("0"),
		  .io_voltage(""),
		  .output_conn("1"),
		  .oe_conn("0"),
		  .output_sync("0"),
		  .oe_sync("0"),
		  .drive_strength("0"),
		  .max_frequency("1"),
		  .i2c_mode("0"),
		  .output_current_cap("8"),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .slew_rate("0"),
		  .vtrip("0"),
		  .use_annotation("0"),
		  .hotswap_needed("0"))
		Dp
		 (.oe({1'b1}),
		  .y({Net_2}),
		  .fb({tmpFB_0__Dp_net[0:0]}),
		  .io({tmpIO_0__Dp_net[0:0]}),
		  .siovref(tmpSIOVREF__Dp_net));


	wire [0:0] tmpFB_0__Dm_net;
	wire [0:0] tmpIO_0__Dm_net;
	electrical [0:0] tmpSIOVREF__Dm_net;

	cy_mxs40_gpio_v1_0
		#(.id("729615f7-8c3c-4f82-989b-8db6cc403a70/a4be9008-37b4-491d-b0dd-3133671a9d3f"),
		  .width(1),
		  .sio_grp_cnt(0),
		  .drive_mode("0"),
		  .ibuf_enabled("0"),
		  .init_dr_st("1"),
		  .input_sync("0"),
		  .intr_mode("0"),
		  .io_voltage(""),
		  .output_conn("1"),
		  .oe_conn("0"),
		  .output_sync("0"),
		  .oe_sync("0"),
		  .drive_strength("0"),
		  .max_frequency("1"),
		  .i2c_mode("0"),
		  .output_current_cap("8"),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .slew_rate("0"),
		  .vtrip("0"),
		  .use_annotation("0"),
		  .hotswap_needed("0"))
		Dm
		 (.oe({1'b1}),
		  .y({Net_3}),
		  .fb({tmpFB_0__Dm_net[0:0]}),
		  .io({tmpIO_0__Dm_net[0:0]}),
		  .siovref(tmpSIOVREF__Dm_net));



	cy_mxs40_isr_v1_0
		#(.deepsleep_required(0),
		  .int_type(2'b10))
		LowPriorityInterrupt
		 (.int_signal(Net_18));



	cy_mxs40_isr_v1_0
		#(.deepsleep_required(0),
		  .int_type(2'b10))
		MediumPriorityInterrupt
		 (.int_signal(Net_19));



	cy_mxs40_isr_v1_0
		#(.deepsleep_required(0),
		  .int_type(2'b10))
		HighPriorityInterrupt
		 (.int_signal(Net_20));



	cy_clock_v1_0
		#(.id("729615f7-8c3c-4f82-989b-8db6cc403a70/dd10a3bb-387b-4f90-a47f-ff23ce6f897f"),
		  .source_clock_id("A43118E8-985E-494A-967B-3EAE88168DCB"),
		  .divisor(0),
		  .period("0"),
		  .is_direct(1),
		  .is_digital(0))
		devClk
		 (.clock_out(Net_62));



	cy_clock_v1_0
		#(.id("729615f7-8c3c-4f82-989b-8db6cc403a70/0b0b3647-28be-498f-8d8a-5add782fc38c"),
		  .source_clock_id("2FB4EC85-8328-4C5A-9ED9-8B63060178EB"),
		  .divisor(0),
		  .period("10000000000"),
		  .is_direct(0),
		  .is_digital(0))
		busRstClk
		 (.clock_out(Net_64));


    bIncludeDriverToComponent_v1_0_0 dma ();

    bIncludeDriverToComponent_v1_0_1 usb_dev ();



endmodule

// Component: cy_analog_virtualmux_v1_0
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\cy_analog_virtualmux_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\cy_analog_virtualmux_v1_0\cy_analog_virtualmux_v1_0.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\cy_analog_virtualmux_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\cy_analog_virtualmux_v1_0\cy_analog_virtualmux_v1_0.v"
`endif

// Component: or_v1_0
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\or_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\or_v1_0\or_v1_0.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\or_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\or_v1_0\or_v1_0.v"
`endif

// CapSense_v3_0(AdcAmuxbInputEnable=false, AdcAnalogStartupDelayUs=23, AdcAzEnable=true, AdcAzTime=5, AdcDedicatedChannels=0, AdcEnable=false, AdcMeasureMode=0, AdcPinAlias=Ch0, AdcResolution=10, AdcSelectAmuxBusChannel=0, AdcTotalChannels=0, AdcVref=-1, BallisticMultiplierEnable=false, BaselineType=0, BlockOffAfterScanEnable=false, Centroid4PtsEnable=false, ClickEnable=false, Csd0IdacGainV1=4, Csd0IdacGainV2=2, Csd0PinAlias=LinearSlider0_Sns0,LinearSlider0_Sns1,LinearSlider0_Sns2,LinearSlider0_Sns3,LinearSlider0_Sns4, Csd0PinCount=5, Csd0ShieldDelayV1=0, Csd0ShieldDelayV2=0, Csd0ShieldEnable=false, Csd0ShieldPrechargeSource=0, Csd0ShieldTankEnable=false, Csd0TotalShieldCount=1, Csd1IdacGainV1=4, Csd1IdacGainV2=2, Csd1PinAlias=, Csd1PinCount=0, Csd1ShieldDelayV1=0, Csd1ShieldDelayV2=0, Csd1ShieldEnable=false, Csd1ShieldPrechargeSource=0, Csd1ShieldTankEnable=false, Csd1TotalShieldCount=1, Csd2xEnable=false, CsdAnalogStartupDelayUs=23, CsdAutoZeroEnable=false, CsdAutoZeroTime=15, CsdCommonSenseClockEnable=false, CsdDedicatedIdacCompEnable=true, CsdDualIdacLevel=50, CsdEnable=true, CsdF1PMSwitchResLowEmiInit=1, CsdF1PMSwitchResLowEmiScan=0, CsdF2PTSwitchResLowEmiInit=1, CsdF2PTSwitchResLowEmiScan=0, CsdFineInitCycles=10, CsdHCAGSwitchResLowEmiInit=1, CsdHCAGSwitchResLowEmiScan=3, CsdHCAVSwitchResLowEmiInit=1, CsdHCAVSwitchResLowEmiScan=0, CsdHCBGSwitchResLowEmiInit=1, CsdHCBGSwitchResLowEmiScan=3, CsdHCBVSwitchResLowEmiInit=1, CsdHCBVSwitchResLowEmiScan=3, CsdIdacAutoCalibrateEnable=true, CsdIdacCompEnable=true, CsdIdacConfig=0, CsdInactiveSensorConnection=0, CsdInitSwitchRes=1, CsdMFSDividerOffsetF1=1, CsdMFSDividerOffsetF2=2, CsdModClockFreq=50000, CsdNoiseMetricEnable=false, CsdNoiseMetricThreshold=1, CsdPrescanSettlingTime=5, CsdRawCountCalibrationLevel=85, CsdSenseClockFreq=3125, CsdSenseClockSource=1, CsdSenseWidthLowEmi=10, CsdSensingMethod=0, CsdShieldSwitchRes=1, CsdSnsClockConstantR=1000, CsdTuningMode=3, CsdV2AnalogWakeupDelayUs=0, CsdVrefV2=-1, Csx0IdacGainV1=4, Csx0IdacGainV2=1, Csx0PinAliasRx=, Csx0PinCountRx=0, Csx1IdacGainV1=4, Csx1IdacGainV2=1, Csx1PinAliasRx=, Csx1PinCountRx=0, Csx2xEnable=false, CsxAnalogStartupDelayUs=23, CsxAutoZeroEnable=false, CsxAutoZeroTime=15, CsxCommonTxClockEnable=false, CsxEnable=true, CsxFineInitCycles=4, CsxIdacAutoCalibrateEnable=true, CsxIdacBitsUsedV1=8, CsxIdacBitsUsedV2=7, CsxInitShieldSwitchRes=2, CsxInitSwitchRes=1, CsxMaxFingers=1, CsxMaxLocalPeaks=5, CsxMFSDividerOffsetF1=1, CsxMFSDividerOffsetF2=2, CsxModClockFreq=50000, CsxMultiphaseTxEnable=false, CsxNoiseMetricEnable=false, CsxNoiseMetricThreshold=1, CsxPinAliasMptx=, CsxPinAliasRx=Button0_Rx0,Button1_Rx0, CsxPinAliasTx=Button0_Tx, CsxPinCountMptx=0, CsxPinCountRx=2, CsxPinCountTx=1, CsxRawCountCalibrationLevel=40, CsxScanShieldSwitchRes=0, CsxScanSwitchRes=0, CsxSkipAndOversampleNodes=false, CsxTxClockFreq=300, CsxTxClockSource=1, CustomDataStructSize=0, DoxygenMode=false, GestureEnable=false, GestureGlobalEnable=false, IrefSel=0, IsAdcSupported=true, IsCapSenseSupported=true, IsGestureSupported=true, LowBaselineResetSize=8, LpModeEn=false, MultiFreqScanEnable=false, NumCentroids=1, OffDebounceEnable=false, OneFingerEdgeSwipeEnable=false, OneFingerEdgeSwipeTimeoutInterval=2000, OneFingerFlickEnable=false, OneFingerRotateEnable=false, OneFingerScrollEnable=false, PosIirFilterCoeff=128, ProxAlpFilterCoeff=2, ProxAlpFilterEnable=false, ProxAverageFilterEnable=false, ProxAverageFilterSampleSize=4, ProxCustomFilterEnable=false, ProxIirFilterBaselineN=1, ProxIirFilterBaselineType=2, ProxIirFilterEnable=false, ProxIirFilterRawCountN=128, ProxIirFilterRawCountType=1, ProxMedianFilterEnable=false, RadialSliderPosIirResetThr=35, RegisterMapSelfTest=false, RegularAlpFilterCoeff=2, RegularAlpFilterEnable=false, RegularAverageFilterEnable=false, RegularAverageFilterSampleSize=4, RegularCustomFilterEnable=false, RegularIirFilterBaselineN=1, RegularIirFilterBaselineType=2, RegularIirFilterEnable=false, RegularIirFilterRawCountN=128, RegularIirFilterRawCountType=1, RegularMedianFilterEnable=false, SecondFinger5x5FilterEnable=false, SelfTestAnalogStartupDelayUs=23, SelfTestBaselineDuplicationEnable=true, SelfTestBaselineRawCountRangeEnable=true, SelfTestEnable=false, SelfTestExtCapEnable=true, SelfTestFineInitCycles=1, SelfTestGlobalCrcEnable=true, SelfTestIntCapEnable=true, SelfTestShCapEnable=true, SelfTestSns2SnsEnable=true, SelfTestSnsCapEnable=true, SelfTestSnsShortEnable=true, SelfTestVddaEnable=true, SelfTestVddaVref=-1, SelfTestWidgetCrcEnable=true, SensorAutoResetEnable=false, SensorAutoResetMethod=0, SensorAutoResetSamplesCount=1000, SliderMultiplierMethod=0, ThresholdSize=16, TimestampInterval=1, TouchpadDisplaySettings=, TouchpadMultiplierMethod=0, TouchProxThresholdCoeff=300, TunerLayoutPreferences=, TunerOptionsPreferences=, TunerViewPreferences=, TunerWidgetData=, TwoFingerScrollEnable=false, TwoFingerSettlingTime=3, TwoFingerZoomEnable=false, VddaValue=3.3, VrefSel=0, WidgetBaselineCoeffEnable=false, WidgetData=<?xml version="1.0" encoding="utf-16"?> <WidgetData xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xsd="http://www.w3.org/2001/XMLSchema">   <Widget id="LinearSlider0" type="LinearSlider" mode="CSD" block="CSD0" slot="0">     <Angle>0</Angle>     <FlippedX>false</FlippedX>     <FlippedY>false</FlippedY>     <Flipped2D>false</Flipped2D>     <ScaleFactor>1</ScaleFactor>     <Properties>       <FingerCap>0.16</FingerCap>       <Diplexing>false</Diplexing>       <MaxPosition>100</MaxPosition>       <MaxXPosition>100</MaxXPosition>       <MaxYPosition>100</MaxYPosition>       <PositionFilter>None</PositionFilter>       <MultiphaseTx>false</MultiphaseTx>       <SenseClockFreq>3125</SenseClockFreq>       <RowSenseClockFreq>3125</RowSenseClockFreq>       <TxClockFreq>300</TxClockFreq>       <IsxTxClockFreq>1000</IsxTxClockFreq>       <ScanResolution>_12</ScanResolution>       <NumConversions>100</NumConversions>       <IdacMod>         <Idac0>32</Idac0>         <Idac1>32</Idac1>         <Idac2>32</Idac2>       </IdacMod>       <RowIdacMod>         <Idac0>32</Idac0>         <Idac1>32</Idac1>         <Idac2>32</Idac2>       </RowIdacMod>       <FingerThreshold>100</FingerThreshold>       <ProxTouchThreshold>200</ProxTouchThreshold>       <NoiseThreshold>40</NoiseThreshold>       <NegNoiseThreshold>40</NegNoiseThreshold>       <LowBaselineReset>30</LowBaselineReset>       <BaselineReset>1000</BaselineReset>       <BaselineUpdateThreshold>100</BaselineUpdateThreshold>       <Hysteresis>10</Hysteresis>       <OnDebounce>3</OnDebounce>       <OffDebounce>3</OffDebounce>       <AlpOnThreshold>60</AlpOnThreshold>       <AlpOFFThreshold>20</AlpOFFThreshold>       <Velocity>2500</Velocity>       <MedianFilter>false</MedianFilter>       <IirFilter>false</IirFilter>       <IirFilterCoeff>128</IirFilterCoeff>       <AIirFilter>false</AIirFilter>       <AverageFilter>false</AverageFilter>       <JitterFilter>false</JitterFilter>       <BallisticMultiplier>false</BallisticMultiplier>       <AIirPositionMoveTh>3</AIirPositionMoveTh>       <AIirPositionSlowMoveTh>7</AIirPositionSlowMoveTh>       <AIirPositionFastMoveTh>12</AIirPositionFastMoveTh>       <AIirCoeffMaxLimit>60</AIirCoeffMaxLimit>       <AIirCoeffMinLimit>1</AIirCoeffMinLimit>       <AIirCoeffDivisor>64</AIirCoeffDivisor>       <CentroidType>Csd3x3</CentroidType>       <CrossCouplingPosTh>5</CrossCouplingPosTh>       <EdgeCorrectionEnable>true</EdgeCorrectionEnable>       <EdgeVirtualSensorTh>100</EdgeVirtualSensorTh>       <EdgePenultimateTh>100</EdgePenultimateTh>       <TwoFingerDetection>false</TwoFingerDetection>       <FastMovementMult>9</FastMovementMult>       <SlowMovementMult>2</SlowMovementMult>       <DivisorValue>4</DivisorValue>       <SpeedThresholdX>3</SpeedThresholdX>       <SpeedThresholdY>4</SpeedThresholdY>     </Properties>     <Gestures>       <Click>         <Enabled>false</Enabled>         <XMaxPosDisplacement>30</XMaxPosDisplacement>         <YMaxPosDisplacement>30</YMaxPosDisplacement>         <MaxPosDisplacement>30</MaxPosDisplacement>         <MaxRadius>20</MaxRadius>         <MaxDisplacement>20</MaxDisplacement>         <MinTouchInterval>10</MinTouchInterval>         <MaxTouchInterval>1000</MaxTouchInterval>         <OneFingerMinTouchDuration>10</OneFingerMinTouchDuration>         <OneFingerMaxTouchDuration>1000</OneFingerMaxTouchDuration>         <TwoFingerMinTouchDuration>100</TwoFingerMinTouchDuration>         <TwoFingerMaxTouchDuration>1000</TwoFingerMaxTouchDuration>       </Click>       <OneFingerScroll>         <Enabled>false</Enabled>         <Debounce>3</Debounce>         <XPosThreshold1>20</XPosThreshold1>         <YPosThreshold1>20</YPosThreshold1>         <PosThreshold1>20</PosThreshold1>         <ScrollStep1>3</ScrollStep1>         <XPosThreshold2>24</XPosThreshold2>         <YPosThreshold2>24</YPosThreshold2>         <F1_Scroll_PosThreshold2>20</F1_Scroll_PosThreshold2>         <ScrollStep2>5</ScrollStep2>         <XPosThreshold3>30</XPosThreshold3>         <YPosThreshold3>30</YPosThreshold3>         <PosThreshold3>20</PosThreshold3>         <ScrollStep3>7</ScrollStep3>         <XPosThreshold4>40</XPosThreshold4>         <YPosThreshold4>40</YPosThreshold4>         <PosThreshold4>20</PosThreshold4>         <ScrollStep4>9</ScrollStep4>         <XPosThreshold>50</XPosThreshold>         <YPosThreshold>50</YPosThreshold>         <PosThreshold>50</PosThreshold>         <CountLevel>Inert_Low</CountLevel>       </OneFingerScroll>       <TwoFingerScroll>         <Enabled>false</Enabled>         <Debounce>3</Debounce>         <XPosThreshold1>20</XPosThreshold1>         <YPosThreshold1>20</YPosThreshold1>         <PosThreshold1>20</PosThreshold1>         <ScrollStep1>3</ScrollStep1>         <XPosThreshold2>24</XPosThreshold2>         <YPosThreshold2>24</YPosThreshold2>         <PosThreshold2>24</PosThreshold2>         <ScrollStep2>5</ScrollStep2>         <XPosThreshold3>30</XPosThreshold3>         <YPosThreshold3>30</YPosThreshold3>         <PosThreshold3>30</PosThreshold3>         <ScrollStep3>7</ScrollStep3>         <XPosThreshold4>40</XPosThreshold4>         <YPosThreshold4>40</YPosThreshold4>         <PosThreshold4>40</PosThreshold4>         <ScrollStep4>9</ScrollStep4>         <XPosThreshold>50</XPosThreshold>         <YPosThreshold>50</YPosThreshold>         <PosThreshold>50</PosThreshold>         <CountLevel>Inert_Low</CountLevel>       </TwoFingerScroll>       <OneFingerFlick>         <Enabled>false</Enabled>         <XPosThreshold>20</XPosThreshold>         <YPosThreshold>20</YPosThreshold>         <PosThreshold>20</PosThreshold>         <MaxSampleInterval>80</MaxSampleInterval>       </OneFingerFlick>       <OneFingerEdgeSwipe>         <Enabled>false</Enabled>         <PosThreshold>100</PosThreshold>         <DetectionTime>100</DetectionTime>         <TimeoutInterval>2000</TimeoutInterval>         <TopAngleThreshold>45</TopAngleThreshold>         <BottomAngleThreshold>45</BottomAngleThreshold>         <DisamRegionWidth>5</DisamRegionWidth>       </OneFingerEdgeSwipe>       <TwoFingerZoom>         <Enabled>false</Enabled>         <XPosThreshold>8</XPosThreshold>         <YPosThreshold>8</YPosThreshold>         <PosThreshold>8</PosThreshold>         <Debounce>3</Debounce>         <ScrollZoomDebounce>5</ScrollZoomDebounce>       </TwoFingerZoom>       <OneFingerRotate>         <Enabled>false</Enabled>         <RotateDebounce>0</RotateDebounce>         <PosThreshold>8</PosThreshold>       </OneFingerRotate>     </Gestures>     <Sensors>       <Sensor id="Sns0">         <Properties>           <IdacComp>             <Idac0>32</Idac0>             <Idac1>32</Idac1>             <Idac2>32</Idac2>           </IdacComp>           <Idacs />           <UseDedicatedPin>true</UseDedicatedPin>           <GangedPins />         </Properties>       </Sensor>       <Sensor id="Sns1">         <Properties>           <IdacComp>             <Idac0>32</Idac0>             <Idac1>32</Idac1>             <Idac2>32</Idac2>           </IdacComp>           <Idacs />           <UseDedicatedPin>true</UseDedicatedPin>           <GangedPins />         </Properties>       </Sensor>       <Sensor id="Sns2">         <Properties>           <IdacComp>             <Idac0>32</Idac0>             <Idac1>32</Idac1>             <Idac2>32</Idac2>           </IdacComp>           <Idacs />           <UseDedicatedPin>true</UseDedicatedPin>           <GangedPins />         </Properties>       </Sensor>       <Sensor id="Sns3">         <Properties>           <IdacComp>             <Idac0>32</Idac0>             <Idac1>32</Idac1>             <Idac2>32</Idac2>           </IdacComp>           <Idacs />           <UseDedicatedPin>true</UseDedicatedPin>           <GangedPins />         </Properties>       </Sensor>       <Sensor id="Sns4">         <Properties>           <IdacComp>             <Idac0>32</Idac0>             <Idac1>32</Idac1>             <Idac2>32</Idac2>           </IdacComp>           <Idacs />           <UseDedicatedPin>true</UseDedicatedPin>           <GangedPins />         </Properties>       </Sensor>     </Sensors>   </Widget>   <Widget id="Button0" type="Button" mode="CSX" block="CSD0" slot="0">     <Angle>0</Angle>     <FlippedX>false</FlippedX>     <FlippedY>false</FlippedY>     <Flipped2D>false</Flipped2D>     <ScaleFactor>1</ScaleFactor>     <Properties>       <FingerCap>0.16</FingerCap>       <Diplexing>false</Diplexing>       <MaxPosition>100</MaxPosition>       <MaxXPosition>100</MaxXPosition>       <MaxYPosition>100</MaxYPosition>       <PositionFilter>None</PositionFilter>       <MultiphaseTx>false</MultiphaseTx>       <SenseClockFreq>3125</SenseClockFreq>       <RowSenseClockFreq>3125</RowSenseClockFreq>       <TxClockFreq>300</TxClockFreq>       <IsxTxClockFreq>1000</IsxTxClockFreq>       <ScanResolution>_12</ScanResolution>       <NumConversions>100</NumConversions>       <IdacMod>         <Idac0>32</Idac0>         <Idac1>32</Idac1>         <Idac2>32</Idac2>       </IdacMod>       <RowIdacMod>         <Idac0>32</Idac0>         <Idac1>32</Idac1>         <Idac2>32</Idac2>       </RowIdacMod>       <FingerThreshold>100</FingerThreshold>       <ProxTouchThreshold>200</ProxTouchThreshold>       <NoiseThreshold>40</NoiseThreshold>       <NegNoiseThreshold>40</NegNoiseThreshold>       <LowBaselineReset>30</LowBaselineReset>       <BaselineReset>1000</BaselineReset>       <BaselineUpdateThreshold>100</BaselineUpdateThreshold>       <Hysteresis>10</Hysteresis>       <OnDebounce>3</OnDebounce>       <OffDebounce>3</OffDebounce>       <AlpOnThreshold>60</AlpOnThreshold>       <AlpOFFThreshold>20</AlpOFFThreshold>       <Velocity>2500</Velocity>       <MedianFilter>false</MedianFilter>       <IirFilter>false</IirFilter>       <IirFilterCoeff>128</IirFilterCoeff>       <AIirFilter>false</AIirFilter>       <AverageFilter>false</AverageFilter>       <JitterFilter>false</JitterFilter>       <BallisticMultiplier>false</BallisticMultiplier>       <AIirPositionMoveTh>3</AIirPositionMoveTh>       <AIirPositionSlowMoveTh>7</AIirPositionSlowMoveTh>       <AIirPositionFastMoveTh>12</AIirPositionFastMoveTh>       <AIirCoeffMaxLimit>60</AIirCoeffMaxLimit>       <AIirCoeffMinLimit>1</AIirCoeffMinLimit>       <AIirCoeffDivisor>64</AIirCoeffDivisor>       <CentroidType>Csd3x3</CentroidType>       <CrossCouplingPosTh>5</CrossCouplingPosTh>       <EdgeCorrectionEnable>true</EdgeCorrectionEnable>       <EdgeVirtualSensorTh>100</EdgeVirtualSensorTh>       <EdgePenultimateTh>100</EdgePenultimateTh>       <TwoFingerDetection>false</TwoFingerDetection>       <FastMovementMult>9</FastMovementMult>       <SlowMovementMult>2</SlowMovementMult>       <DivisorValue>4</DivisorValue>       <SpeedThresholdX>3</SpeedThresholdX>       <SpeedThresholdY>4</SpeedThresholdY>     </Properties>     <Gestures>       <Click>         <Enabled>false</Enabled>         <XMaxPosDisplacement>30</XMaxPosDisplacement>         <YMaxPosDisplacement>30</YMaxPosDisplacement>         <MaxPosDisplacement>30</MaxPosDisplacement>         <MaxRadius>20</MaxRadius>         <MaxDisplacement>20</MaxDisplacement>         <MinTouchInterval>10</MinTouchInterval>         <MaxTouchInterval>1000</MaxTouchInterval>         <OneFingerMinTouchDuration>10</OneFingerMinTouchDuration>         <OneFingerMaxTouchDuration>1000</OneFingerMaxTouchDuration>         <TwoFingerMinTouchDuration>100</TwoFingerMinTouchDuration>         <TwoFingerMaxTouchDuration>1000</TwoFingerMaxTouchDuration>       </Click>       <OneFingerScroll>         <Enabled>false</Enabled>         <Debounce>3</Debounce>         <XPosThreshold1>20</XPosThreshold1>         <YPosThreshold1>20</YPosThreshold1>         <PosThreshold1>20</PosThreshold1>         <ScrollStep1>3</ScrollStep1>         <XPosThreshold2>24</XPosThreshold2>         <YPosThreshold2>24</YPosThreshold2>         <F1_Scroll_PosThreshold2>20</F1_Scroll_PosThreshold2>         <ScrollStep2>5</ScrollStep2>         <XPosThreshold3>30</XPosThreshold3>         <YPosThreshold3>30</YPosThreshold3>         <PosThreshold3>20</PosThreshold3>         <ScrollStep3>7</ScrollStep3>         <XPosThreshold4>40</XPosThreshold4>         <YPosThreshold4>40</YPosThreshold4>         <PosThreshold4>20</PosThreshold4>         <ScrollStep4>9</ScrollStep4>         <XPosThreshold>50</XPosThreshold>         <YPosThreshold>50</YPosThreshold>         <PosThreshold>50</PosThreshold>         <CountLevel>Inert_Low</CountLevel>       </OneFingerScroll>       <TwoFingerScroll>         <Enabled>false</Enabled>         <Debounce>3</Debounce>         <XPosThreshold1>20</XPosThreshold1>         <YPosThreshold1>20</YPosThreshold1>         <PosThreshold1>20</PosThreshold1>         <ScrollStep1>3</ScrollStep1>         <XPosThreshold2>24</XPosThreshold2>         <YPosThreshold2>24</YPosThreshold2>         <PosThreshold2>24</PosThreshold2>         <ScrollStep2>5</ScrollStep2>         <XPosThreshold3>30</XPosThreshold3>         <YPosThreshold3>30</YPosThreshold3>         <PosThreshold3>30</PosThreshold3>         <ScrollStep3>7</ScrollStep3>         <XPosThreshold4>40</XPosThreshold4>         <YPosThreshold4>40</YPosThreshold4>         <PosThreshold4>40</PosThreshold4>         <ScrollStep4>9</ScrollStep4>         <XPosThreshold>50</XPosThreshold>         <YPosThreshold>50</YPosThreshold>         <PosThreshold>50</PosThreshold>         <CountLevel>Inert_Low</CountLevel>       </TwoFingerScroll>       <OneFingerFlick>         <Enabled>false</Enabled>         <XPosThreshold>20</XPosThreshold>         <YPosThreshold>20</YPosThreshold>         <PosThreshold>20</PosThreshold>         <MaxSampleInterval>80</MaxSampleInterval>       </OneFingerFlick>       <OneFingerEdgeSwipe>         <Enabled>false</Enabled>         <PosThreshold>100</PosThreshold>         <DetectionTime>100</DetectionTime>         <TimeoutInterval>2000</TimeoutInterval>         <TopAngleThreshold>45</TopAngleThreshold>         <BottomAngleThreshold>45</BottomAngleThreshold>         <DisamRegionWidth>5</DisamRegionWidth>       </OneFingerEdgeSwipe>       <TwoFingerZoom>         <Enabled>false</Enabled>         <XPosThreshold>8</XPosThreshold>         <YPosThreshold>8</YPosThreshold>         <PosThreshold>8</PosThreshold>         <Debounce>3</Debounce>         <ScrollZoomDebounce>5</ScrollZoomDebounce>       </TwoFingerZoom>       <OneFingerRotate>         <Enabled>false</Enabled>         <RotateDebounce>0</RotateDebounce>         <PosThreshold>8</PosThreshold>       </OneFingerRotate>     </Gestures>     <Columns>       <Sensor id="Rx0">         <Properties>           <IdacComp>             <Idac0>32</Idac0>             <Idac1>32</Idac1>             <Idac2>32</Idac2>           </IdacComp>           <Idacs>             <Idac>               <Idac0>32</Idac0>               <Idac1>32</Idac1>               <Idac2>32</Idac2>             </Idac>           </Idacs>           <UseDedicatedPin>true</UseDedicatedPin>           <GangedPins />         </Properties>       </Sensor>     </Columns>     <Rows>       <Sensor id="Tx">         <Properties>           <IdacComp>             <Idac0>32</Idac0>             <Idac1>32</Idac1>             <Idac2>32</Idac2>           </IdacComp>           <Idacs />           <UseDedicatedPin>true</UseDedicatedPin>           <GangedPins />         </Properties>       </Sensor>     </Rows>   </Widget>   <Widget id="Button1" type="Button" mode="CSX" block="CSD0" slot="0">     <Angle>0</Angle>     <FlippedX>false</FlippedX>     <FlippedY>false</FlippedY>     <Flipped2D>false</Flipped2D>     <ScaleFactor>1</ScaleFactor>     <Properties>       <FingerCap>0.16</FingerCap>       <Diplexing>false</Diplexing>       <MaxPosition>100</MaxPosition>       <MaxXPosition>100</MaxXPosition>       <MaxYPosition>100</MaxYPosition>       <PositionFilter>None</PositionFilter>       <MultiphaseTx>false</MultiphaseTx>       <SenseClockFreq>3125</SenseClockFreq>       <RowSenseClockFreq>3125</RowSenseClockFreq>       <TxClockFreq>300</TxClockFreq>       <IsxTxClockFreq>1000</IsxTxClockFreq>       <ScanResolution>_12</ScanResolution>       <NumConversions>100</NumConversions>       <IdacMod>         <Idac0>32</Idac0>         <Idac1>32</Idac1>         <Idac2>32</Idac2>       </IdacMod>       <RowIdacMod>         <Idac0>32</Idac0>         <Idac1>32</Idac1>         <Idac2>32</Idac2>       </RowIdacMod>       <FingerThreshold>100</FingerThreshold>       <ProxTouchThreshold>200</ProxTouchThreshold>       <NoiseThreshold>40</NoiseThreshold>       <NegNoiseThreshold>40</NegNoiseThreshold>       <LowBaselineReset>30</LowBaselineReset>       <BaselineReset>1000</BaselineReset>       <BaselineUpdateThreshold>100</BaselineUpdateThreshold>       <Hysteresis>10</Hysteresis>       <OnDebounce>3</OnDebounce>       <OffDebounce>3</OffDebounce>       <AlpOnThreshold>60</AlpOnThreshold>       <AlpOFFThreshold>20</AlpOFFThreshold>       <Velocity>2500</Velocity>       <MedianFilter>false</MedianFilter>       <IirFilter>false</IirFilter>       <IirFilterCoeff>128</IirFilterCoeff>       <AIirFilter>false</AIirFilter>       <AverageFilter>false</AverageFilter>       <JitterFilter>false</JitterFilter>       <BallisticMultiplier>false</BallisticMultiplier>       <AIirPositionMoveTh>3</AIirPositionMoveTh>       <AIirPositionSlowMoveTh>7</AIirPositionSlowMoveTh>       <AIirPositionFastMoveTh>12</AIirPositionFastMoveTh>       <AIirCoeffMaxLimit>60</AIirCoeffMaxLimit>       <AIirCoeffMinLimit>1</AIirCoeffMinLimit>       <AIirCoeffDivisor>64</AIirCoeffDivisor>       <CentroidType>Csd3x3</CentroidType>       <CrossCouplingPosTh>5</CrossCouplingPosTh>       <EdgeCorrectionEnable>true</EdgeCorrectionEnable>       <EdgeVirtualSensorTh>100</EdgeVirtualSensorTh>       <EdgePenultimateTh>100</EdgePenultimateTh>       <TwoFingerDetection>false</TwoFingerDetection>       <FastMovementMult>9</FastMovementMult>       <SlowMovementMult>2</SlowMovementMult>       <DivisorValue>4</DivisorValue>       <SpeedThresholdX>3</SpeedThresholdX>       <SpeedThresholdY>4</SpeedThresholdY>     </Properties>     <Gestures>       <Click>         <Enabled>false</Enabled>         <XMaxPosDisplacement>30</XMaxPosDisplacement>         <YMaxPosDisplacement>30</YMaxPosDisplacement>         <MaxPosDisplacement>30</MaxPosDisplacement>         <MaxRadius>20</MaxRadius>         <MaxDisplacement>20</MaxDisplacement>         <MinTouchInterval>10</MinTouchInterval>         <MaxTouchInterval>1000</MaxTouchInterval>         <OneFingerMinTouchDuration>10</OneFingerMinTouchDuration>         <OneFingerMaxTouchDuration>1000</OneFingerMaxTouchDuration>         <TwoFingerMinTouchDuration>100</TwoFingerMinTouchDuration>         <TwoFingerMaxTouchDuration>1000</TwoFingerMaxTouchDuration>       </Click>       <OneFingerScroll>         <Enabled>false</Enabled>         <Debounce>3</Debounce>         <XPosThreshold1>20</XPosThreshold1>         <YPosThreshold1>20</YPosThreshold1>         <PosThreshold1>20</PosThreshold1>         <ScrollStep1>3</ScrollStep1>         <XPosThreshold2>24</XPosThreshold2>         <YPosThreshold2>24</YPosThreshold2>         <F1_Scroll_PosThreshold2>20</F1_Scroll_PosThreshold2>         <ScrollStep2>5</ScrollStep2>         <XPosThreshold3>30</XPosThreshold3>         <YPosThreshold3>30</YPosThreshold3>         <PosThreshold3>20</PosThreshold3>         <ScrollStep3>7</ScrollStep3>         <XPosThreshold4>40</XPosThreshold4>         <YPosThreshold4>40</YPosThreshold4>         <PosThreshold4>20</PosThreshold4>         <ScrollStep4>9</ScrollStep4>         <XPosThreshold>50</XPosThreshold>         <YPosThreshold>50</YPosThreshold>         <PosThreshold>50</PosThreshold>         <CountLevel>Inert_Low</CountLevel>       </OneFingerScroll>       <TwoFingerScroll>         <Enabled>false</Enabled>         <Debounce>3</Debounce>         <XPosThreshold1>20</XPosThreshold1>         <YPosThreshold1>20</YPosThreshold1>         <PosThreshold1>20</PosThreshold1>         <ScrollStep1>3</ScrollStep1>         <XPosThreshold2>24</XPosThreshold2>         <YPosThreshold2>24</YPosThreshold2>         <PosThreshold2>24</PosThreshold2>         <ScrollStep2>5</ScrollStep2>         <XPosThreshold3>30</XPosThreshold3>         <YPosThreshold3>30</YPosThreshold3>         <PosThreshold3>30</PosThreshold3>         <ScrollStep3>7</ScrollStep3>         <XPosThreshold4>40</XPosThreshold4>         <YPosThreshold4>40</YPosThreshold4>         <PosThreshold4>40</PosThreshold4>         <ScrollStep4>9</ScrollStep4>         <XPosThreshold>50</XPosThreshold>         <YPosThreshold>50</YPosThreshold>         <PosThreshold>50</PosThreshold>         <CountLevel>Inert_Low</CountLevel>       </TwoFingerScroll>       <OneFingerFlick>         <Enabled>false</Enabled>         <XPosThreshold>20</XPosThreshold>         <YPosThreshold>20</YPosThreshold>         <PosThreshold>20</PosThreshold>         <MaxSampleInterval>80</MaxSampleInterval>       </OneFingerFlick>       <OneFingerEdgeSwipe>         <Enabled>false</Enabled>         <PosThreshold>100</PosThreshold>         <DetectionTime>100</DetectionTime>         <TimeoutInterval>2000</TimeoutInterval>         <TopAngleThreshold>45</TopAngleThreshold>         <BottomAngleThreshold>45</BottomAngleThreshold>         <DisamRegionWidth>5</DisamRegionWidth>       </OneFingerEdgeSwipe>       <TwoFingerZoom>         <Enabled>false</Enabled>         <XPosThreshold>8</XPosThreshold>         <YPosThreshold>8</YPosThreshold>         <PosThreshold>8</PosThreshold>         <Debounce>3</Debounce>         <ScrollZoomDebounce>5</ScrollZoomDebounce>       </TwoFingerZoom>       <OneFingerRotate>         <Enabled>false</Enabled>         <RotateDebounce>0</RotateDebounce>         <PosThreshold>8</PosThreshold>       </OneFingerRotate>     </Gestures>     <Columns>       <Sensor id="Rx0">         <Properties>           <IdacComp>             <Idac0>32</Idac0>             <Idac1>32</Idac1>             <Idac2>32</Idac2>           </IdacComp>           <Idacs>             <Idac>               <Idac0>32</Idac0>               <Idac1>32</Idac1>               <Idac2>32</Idac2>             </Idac>           </Idacs>           <UseDedicatedPin>true</UseDedicatedPin>           <GangedPins />         </Properties>       </Sensor>     </Columns>     <Rows>       <Sensor id="Tx">         <Properties>           <IdacComp>             <Idac0>32</Idac0>             <Idac1>32</Idac1>             <Idac2>32</Idac2>           </IdacComp>           <Idacs />           <UseDedicatedPin>false</UseDedicatedPin>           <GangedPins>             <Pin>Button0_Tx</Pin>           </GangedPins>         </Properties>       </Sensor>     </Rows>   </Widget> </WidgetData>, CY_API_CALLBACK_HEADER_INCLUDE=#include "cyapicallbacks.h", CY_COMMENT=, CY_COMPONENT_NAME=CapSense_v3_0, CY_CONFIG_TITLE=CapSense, CY_CONST_CONFIG=true, CY_CONTROL_FILE=<:default:>, CY_DATASHEET_FILE=<:default:>, CY_FITTER_NAME=CapSense, CY_INSTANCE_SHORT_NAME=CapSense, CY_MAJOR_VERSION=3, CY_MINOR_VERSION=0, CY_PDL_DRIVER_NAME=, CY_PDL_DRIVER_REQ_VERSION=, CY_PDL_DRIVER_SUBGROUP=, CY_PDL_DRIVER_VARIANT=, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=PSoC Creator  4.4, INSTANCE_NAME=CapSense, )
module CapSense_v3_0_3 ;


          wire  Net_609;
    electrical  Net_850;
          wire  Net_849;
          wire  Net_813;
          wire  Net_818;
          wire  Net_814;
          wire  Net_817;
          wire  Net_815;
          wire  Net_851;
          wire [15:0] Net_828;
          wire  Net_845;
          wire  Net_819;
          wire  Net_822;
          wire  Net_821;
          wire  Net_610;
          wire  Net_608;
          wire  Net_44;
    electrical  Net_34;
          wire  Net_1423;
          wire  Net_639;
          wire  Net_638;
          wire  Net_637;
          wire  Net_636;
          wire  Net_48;
          wire  Net_47;
          wire  Net_46;
          wire  Net_45;
          wire  Net_589;
    electrical  Net_314;
    electrical  Net_866;
    electrical [1:0] dedicated_io_bus;
    electrical  Net_846;
    electrical  Net_616;
    electrical  Net_82;
    electrical  Net_615;
    electrical  Net_324;
    electrical  Net_273;
    electrical  Net_848;
    electrical [1:0] Net_13;
    electrical [4:0] Net_847;
    electrical  Net_606;
          wire  Net_611;

	wire [0:0] tmpFB_0__Cmod_net;
	wire [0:0] tmpIO_0__Cmod_net;
	electrical [0:0] tmpSIOVREF__Cmod_net;

	cy_mxs40_gpio_v1_0
		#(.id("4e328e59-4f13-4f70-bb9d-9a75dfc21492/4db2e9d3-9f70-4f4e-a919-7eefada41863"),
		  .width(1),
		  .sio_grp_cnt(0),
		  .drive_mode("0"),
		  .ibuf_enabled("0"),
		  .init_dr_st("1"),
		  .input_sync("0"),
		  .intr_mode("0"),
		  .io_voltage(""),
		  .output_conn("0"),
		  .oe_conn("0"),
		  .output_sync("0"),
		  .oe_sync("0"),
		  .drive_strength("0"),
		  .max_frequency("100"),
		  .i2c_mode("0"),
		  .output_current_cap("8"),
		  .pin_aliases("Cmod"),
		  .pin_mode("A"),
		  .slew_rate("0"),
		  .vtrip("0"),
		  .use_annotation("0"),
		  .hotswap_needed("0"))
		Cmod
		 (.oe({1'b1}),
		  .y({1'b0}),
		  .fb({tmpFB_0__Cmod_net[0:0]}),
		  .analog({Net_314}),
		  .io({tmpIO_0__Cmod_net[0:0]}),
		  .siovref(tmpSIOVREF__Cmod_net));


	wire [1:0] tmpFB_1__Rx_net;
	wire [1:0] tmpIO_1__Rx_net;
	electrical [0:0] tmpSIOVREF__Rx_net;

	cy_mxs40_gpio_v1_0
		#(.id("4e328e59-4f13-4f70-bb9d-9a75dfc21492/77fd7f86-2082-41fa-a471-914d5f07502d"),
		  .width(2),
		  .sio_grp_cnt(0),
		  .drive_mode("0,0"),
		  .ibuf_enabled("0,0"),
		  .init_dr_st("1,1"),
		  .input_sync("0,0"),
		  .intr_mode("0,0"),
		  .io_voltage(","),
		  .output_conn("0,0"),
		  .oe_conn("0,0"),
		  .output_sync("0,0"),
		  .oe_sync("0,0"),
		  .drive_strength("0,0"),
		  .max_frequency("100,100"),
		  .i2c_mode("0,0"),
		  .output_current_cap("8,8"),
		  .pin_aliases("Button0_Rx0,Button1_Rx0"),
		  .pin_mode("A,A"),
		  .slew_rate("0,0"),
		  .vtrip("0,0"),
		  .use_annotation("0,0"),
		  .hotswap_needed("0,0"))
		Rx
		 (.oe({2'b11}),
		  .y({2'b0}),
		  .fb({tmpFB_1__Rx_net[1:0]}),
		  .analog({Net_13[1:0]}),
		  .io({tmpIO_1__Rx_net[1:0]}),
		  .siovref(tmpSIOVREF__Rx_net));


    ZeroTerminal ZeroTerminal_15 (
        .z(Net_589));

	wire [0:0] tmpFB_0__CintB_net;
	wire [0:0] tmpIO_0__CintB_net;
	electrical [0:0] tmpSIOVREF__CintB_net;

	cy_mxs40_gpio_v1_0
		#(.id("4e328e59-4f13-4f70-bb9d-9a75dfc21492/dbc12ae1-a607-4701-99cc-6261716a9147"),
		  .width(1),
		  .sio_grp_cnt(0),
		  .drive_mode("0"),
		  .ibuf_enabled("0"),
		  .init_dr_st("1"),
		  .input_sync("0"),
		  .intr_mode("0"),
		  .io_voltage(""),
		  .output_conn("0"),
		  .oe_conn("0"),
		  .output_sync("0"),
		  .oe_sync("0"),
		  .drive_strength("0"),
		  .max_frequency("100"),
		  .i2c_mode("0"),
		  .output_current_cap("8"),
		  .pin_aliases(""),
		  .pin_mode("A"),
		  .slew_rate("0"),
		  .vtrip("0"),
		  .use_annotation("0"),
		  .hotswap_needed("0"))
		CintB
		 (.oe({1'b1}),
		  .y({1'b0}),
		  .fb({tmpFB_0__CintB_net[0:0]}),
		  .analog({Net_615}),
		  .io({tmpIO_0__CintB_net[0:0]}),
		  .siovref(tmpSIOVREF__CintB_net));


	wire [0:0] tmpFB_0__CintA_net;
	wire [0:0] tmpIO_0__CintA_net;
	electrical [0:0] tmpSIOVREF__CintA_net;

	cy_mxs40_gpio_v1_0
		#(.id("4e328e59-4f13-4f70-bb9d-9a75dfc21492/2605f567-73b7-435d-b3b9-174766b96934"),
		  .width(1),
		  .sio_grp_cnt(0),
		  .drive_mode("0"),
		  .ibuf_enabled("0"),
		  .init_dr_st("1"),
		  .input_sync("0"),
		  .intr_mode("0"),
		  .io_voltage(""),
		  .output_conn("0"),
		  .oe_conn("0"),
		  .output_sync("0"),
		  .oe_sync("0"),
		  .drive_strength("0"),
		  .max_frequency("100"),
		  .i2c_mode("0"),
		  .output_current_cap("8"),
		  .pin_aliases(""),
		  .pin_mode("A"),
		  .slew_rate("0"),
		  .vtrip("0"),
		  .use_annotation("0"),
		  .hotswap_needed("0"))
		CintA
		 (.oe({1'b1}),
		  .y({1'b0}),
		  .fb({tmpFB_0__CintA_net[0:0]}),
		  .analog({Net_82}),
		  .io({tmpIO_0__CintA_net[0:0]}),
		  .siovref(tmpSIOVREF__CintA_net));


    cy_mxs40_csidac_v1_0 IDACMod (
        .dsi_idac_leg1_en(Net_45),
        .dsi_idac_leg2_en(Net_46),
        .dsi_idac_leg3_en(Net_47),
        .dsi_idac_pol(Net_48),
        .iout(Net_606));
    defparam IDACMod.leg3_needed = 0;

    cy_mxs40_csidac_v1_0 IDACComp (
        .dsi_idac_leg1_en(Net_636),
        .dsi_idac_leg2_en(Net_637),
        .dsi_idac_leg3_en(Net_638),
        .dsi_idac_pol(Net_639),
        .iout(Net_866));
    defparam IDACComp.leg3_needed = 1;

    ZeroTerminal ZeroTerminal_1 (
        .z(Net_45));

    ZeroTerminal ZeroTerminal_2 (
        .z(Net_46));

    ZeroTerminal ZeroTerminal_3 (
        .z(Net_47));

    ZeroTerminal ZeroTerminal_4 (
        .z(Net_48));

    ZeroTerminal ZeroTerminal_5 (
        .z(Net_636));

    ZeroTerminal ZeroTerminal_6 (
        .z(Net_637));

    ZeroTerminal ZeroTerminal_7 (
        .z(Net_638));

    ZeroTerminal ZeroTerminal_8 (
        .z(Net_639));

    ZeroTerminal ZeroTerminal_14 (
        .z(Net_1423));

    cy_analog_noconnect_v1_0 cy_analog_noconnect_1 (
        .noconnect(Net_34));

    ZeroTerminal ZeroTerminal_12 (
        .z(Net_44));

	// cy_analog_virtualmux_1 (cy_analog_virtualmux_v1_0)
	cy_connect_v1_0 cy_analog_virtualmux_1_connect(Net_314, Net_616);
	defparam cy_analog_virtualmux_1_connect.sig_width = 1;

    ZeroTerminal ZeroTerminal_13 (
        .z(Net_608));


    assign Net_610 = Net_608 | Net_611;

    cy_mxs40_csd_v1_0 CSD (
        .adc_channel(Net_324),
        .amuxa(Net_606),
        .amuxb(Net_846),
        .clock(Net_611),
        .cmod(Net_82),
        .csd_tx(Net_821),
        .csd_tx_n(Net_822),
        .csh(Net_615),
        .dedicated_io(dedicated_io_bus[1:0]),
        .dsi_adc_on(Net_819),
        .dsi_cmod(Net_845),
        .dsi_count(Net_828[15:0]),
        .dsi_count_val_sel(Net_851),
        .dsi_csh_tank(Net_815),
        .dsi_hscmp(Net_817),
        .dsi_sample_in(Net_1423),
        .dsi_sample_out(Net_814),
        .dsi_sampling(Net_818),
        .dsi_sense_in(Net_44),
        .dsi_sense_out(Net_813),
        .dsi_start(Net_589),
        .interrupt(Net_849),
        .rx(Net_13[1:0]),
        .sense(Net_847[4:0]),
        .shield(Net_273),
        .shield_pad(Net_616),
        .tx(Net_848),
        .vref_ext(Net_34),
        .vref_pass(Net_850));
    defparam CSD.adc_channel_count = 1;
    defparam CSD.dedicated_io_count = 2;
    defparam CSD.is_capsense = 1;
    defparam CSD.is_cmod_charge = 1;
    defparam CSD.rx_count = 2;
    defparam CSD.sense_as_shield = 0;
    defparam CSD.sensors_count = 5;
    defparam CSD.shield_as_sense = 0;
    defparam CSD.shield_count = 1;
    defparam CSD.tx_count = 1;


	cy_clock_v1_0
		#(.id("4e328e59-4f13-4f70-bb9d-9a75dfc21492/8273f0d6-caef-4cc7-ad3c-09656b78e2cb"),
		  .source_clock_id("2FB4EC85-8328-4C5A-9ED9-8B63060178EB"),
		  .divisor(255),
		  .period("0"),
		  .is_direct(0),
		  .is_digital(0))
		ModClk
		 (.clock_out(Net_611));


	wire [4:0] tmpFB_4__Sns_net;
	wire [4:0] tmpIO_4__Sns_net;
	electrical [0:0] tmpSIOVREF__Sns_net;

	cy_mxs40_gpio_v1_0
		#(.id("4e328e59-4f13-4f70-bb9d-9a75dfc21492/0113321b-4a37-46f6-8407-2f8646c68756"),
		  .width(5),
		  .sio_grp_cnt(0),
		  .drive_mode("0,0,0,0,0"),
		  .ibuf_enabled("0,0,0,0,0"),
		  .init_dr_st("1,1,1,1,1"),
		  .input_sync("0,0,0,0,0"),
		  .intr_mode("0,0,0,0,0"),
		  .io_voltage(",,,,"),
		  .output_conn("0,0,0,0,0"),
		  .oe_conn("0,0,0,0,0"),
		  .output_sync("0,0,0,0,0"),
		  .oe_sync("0,0,0,0,0"),
		  .drive_strength("0,0,0,0,0"),
		  .max_frequency("100,100,100,100,100"),
		  .i2c_mode("0,0,0,0,0"),
		  .output_current_cap("8,8,8,8,8"),
		  .pin_aliases("LinearSlider0_Sns0,LinearSlider0_Sns1,LinearSlider0_Sns2,LinearSlider0_Sns3,LinearSlider0_Sns4"),
		  .pin_mode("A,A,A,A,A"),
		  .slew_rate("0,0,0,0,0"),
		  .vtrip("0,0,0,0,0"),
		  .use_annotation("0,0,0,0,0"),
		  .hotswap_needed("0,0,0,0,0"))
		Sns
		 (.oe({5'b11111}),
		  .y({5'b0}),
		  .fb({tmpFB_4__Sns_net[4:0]}),
		  .analog({Net_847[4:0]}),
		  .io({tmpIO_4__Sns_net[4:0]}),
		  .siovref(tmpSIOVREF__Sns_net));


	wire [0:0] tmpFB_0__Tx_net;
	wire [0:0] tmpIO_0__Tx_net;
	electrical [0:0] tmpSIOVREF__Tx_net;

	cy_mxs40_gpio_v1_0
		#(.id("4e328e59-4f13-4f70-bb9d-9a75dfc21492/8ed50244-d239-4c9b-9207-b8e71ce193dc"),
		  .width(1),
		  .sio_grp_cnt(0),
		  .drive_mode("0"),
		  .ibuf_enabled("0"),
		  .init_dr_st("1"),
		  .input_sync("0"),
		  .intr_mode("0"),
		  .io_voltage(""),
		  .output_conn("0"),
		  .oe_conn("0"),
		  .output_sync("0"),
		  .oe_sync("0"),
		  .drive_strength("0"),
		  .max_frequency("100"),
		  .i2c_mode("0"),
		  .output_current_cap("8"),
		  .pin_aliases("Button0_Tx"),
		  .pin_mode("A"),
		  .slew_rate("0"),
		  .vtrip("0"),
		  .use_annotation("0"),
		  .hotswap_needed("0"))
		Tx
		 (.oe({1'b1}),
		  .y({1'b0}),
		  .fb({tmpFB_0__Tx_net[0:0]}),
		  .analog({Net_848}),
		  .io({tmpIO_0__Tx_net[0:0]}),
		  .siovref(tmpSIOVREF__Tx_net));



	cy_mxs40_isr_v1_0
		#(.deepsleep_required(0),
		  .int_type(2'b10))
		ISR
		 (.int_signal(Net_849));


    cy_analog_noconnect_v1_0 cy_analog_noconnect_3 (
        .noconnect(Net_850));

    ZeroTerminal ZeroTerminal_9 (
        .z(Net_851));

	// cy_analog_virtualmux_2 (cy_analog_virtualmux_v1_0)
	cy_connect_v1_0 cy_analog_virtualmux_2_connect(Net_866, Net_606);
	defparam cy_analog_virtualmux_2_connect.sig_width = 1;



endmodule

// TCPWM_PWM_PDL_v1_0(ClockPrescaler=0, Compare0=500, Compare1=16384, CountInput=7, CountInputMasked=3, DeadClocks=0, EnableCompareSwap=false, EnablePeriodSwap=false, InterruptSource=0, InvertPwm=false, InvertPwm_n=false, KillInput=7, KillInputMasked=3, KillMode=2, Period0=1000, Period1=32768, PwmAlignment=0, PwmMode=4, ReloadInput=7, ReloadInputMasked=3, Resolution=16, RunMode=0, StartInput=7, StartInputMasked=3, SwapInput=7, SwapInputMasked=3, CY_API_CALLBACK_HEADER_INCLUDE=#include "cyapicallbacks.h", CY_COMMENT=, CY_COMPONENT_NAME=TCPWM_PWM_PDL_v1_0, CY_CONFIG_TITLE=PWM_1, CY_CONST_CONFIG=true, CY_CONTROL_FILE=<:default:>, CY_DATASHEET_FILE=<:default:>, CY_FITTER_NAME=PWM_1, CY_INSTANCE_SHORT_NAME=PWM_1, CY_MAJOR_VERSION=1, CY_MINOR_VERSION=0, CY_PDL_DRIVER_NAME=tcpwm, CY_PDL_DRIVER_REQ_VERSION=1.0.0, CY_PDL_DRIVER_SUBGROUP=, CY_PDL_DRIVER_VARIANT=, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=PSoC Creator  4.4, INSTANCE_NAME=PWM_1, )
module TCPWM_PWM_PDL_v1_0_4 (
    clock,
    compare,
    count,
    interrupt,
    kill,
    ovrflw,
    pwm,
    pwm_n,
    reload,
    start,
    swap,
    undrflw);
    input       clock;
    output      compare;
    input       count;
    output      interrupt;
    input       kill;
    output      ovrflw;
    output      pwm;
    output      pwm_n;
    input       reload;
    input       start;
    input       swap;
    output      undrflw;



    cy_mxs40_tcpwm_v1_0 TCPWM (
        .capture(swap),
        .clock(clock),
        .count(count),
        .interrupt(interrupt),
        .line(pwm),
        .line_compl(pwm_n),
        .reload(reload),
        .start(start),
        .stop(kill),
        .tr_compare_match(compare),
        .tr_overflow(ovrflw),
        .tr_underflow(undrflw));
    defparam TCPWM.exact_width = 0;
    defparam TCPWM.width = 16;



endmodule

// Component: not_v1_0
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\not_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\not_v1_0\not_v1_0.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\not_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\4.4\PSoC Creator\psoc\content\CyPrimitives\cyprimitives.cylib\not_v1_0\not_v1_0.v"
`endif

// top
module top ;

          wire  Net_122;
          wire  Net_145;
          wire  Net_142;
          wire  Net_140;
          wire  Net_139;
          wire  Net_148;
          wire  Net_144;
          wire  Net_138;
          wire  Net_143;
          wire  Net_141;
          wire  Net_146;
          wire  Net_152;
          wire  Net_147;
          wire  Net_149;
          wire  Net_2;
          wire  Net_120;

    USBFS_DEV_PDL_v1_0_2 USBFS (
        .sof(Net_2));

    CapSense_v3_0_3 CapSense ();

	wire [0:0] tmpFB_0__BLUE_net;
	wire [0:0] tmpIO_0__BLUE_net;
	electrical [0:0] tmpSIOVREF__BLUE_net;

	cy_mxs40_gpio_v1_0
		#(.id("6b6e4a3b-b8f4-4d9f-b11b-44c7a611f271"),
		  .width(1),
		  .sio_grp_cnt(0),
		  .drive_mode("6"),
		  .ibuf_enabled("0"),
		  .init_dr_st("1"),
		  .input_sync("0"),
		  .intr_mode("0"),
		  .io_voltage(""),
		  .output_conn("1"),
		  .oe_conn("0"),
		  .output_sync("0"),
		  .oe_sync("0"),
		  .drive_strength("0"),
		  .max_frequency("100"),
		  .i2c_mode("0"),
		  .output_current_cap("8"),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .slew_rate("0"),
		  .vtrip("0"),
		  .use_annotation("0"),
		  .hotswap_needed("0"))
		BLUE
		 (.oe({1'b1}),
		  .y({Net_149}),
		  .fb({tmpFB_0__BLUE_net[0:0]}),
		  .io({tmpIO_0__BLUE_net[0:0]}),
		  .siovref(tmpSIOVREF__BLUE_net));



	cy_clock_v1_0
		#(.id("5f9772b4-eee5-4fd0-b50b-44b471961bce"),
		  .source_clock_id("2FB4EC85-8328-4C5A-9ED9-8B63060178EB"),
		  .divisor(0),
		  .period("1000000000"),
		  .is_direct(0),
		  .is_digital(0))
		Clock_1
		 (.clock_out(Net_147));


    TCPWM_PWM_PDL_v1_0_4 PWM_1 (
        .clock(Net_147),
        .compare(Net_146),
        .count(1'b1),
        .interrupt(Net_143),
        .kill(1'b0),
        .ovrflw(Net_144),
        .pwm(Net_148),
        .pwm_n(Net_149),
        .reload(1'b0),
        .start(1'b0),
        .swap(1'b0),
        .undrflw(Net_145));

	wire [0:0] tmpFB_0__RED_net;
	wire [0:0] tmpIO_0__RED_net;
	electrical [0:0] tmpSIOVREF__RED_net;

	cy_mxs40_gpio_v1_0
		#(.id("b35c1cd7-2d80-4ba9-a8ca-aa8f9bdb4376"),
		  .width(1),
		  .sio_grp_cnt(0),
		  .drive_mode("6"),
		  .ibuf_enabled("0"),
		  .init_dr_st("1"),
		  .input_sync("0"),
		  .intr_mode("0"),
		  .io_voltage(""),
		  .output_conn("1"),
		  .oe_conn("0"),
		  .output_sync("0"),
		  .oe_sync("0"),
		  .drive_strength("0"),
		  .max_frequency("100"),
		  .i2c_mode("0"),
		  .output_current_cap("8"),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .slew_rate("0"),
		  .vtrip("0"),
		  .use_annotation("0"),
		  .hotswap_needed("0"))
		RED
		 (.oe({1'b1}),
		  .y({Net_122}),
		  .fb({tmpFB_0__RED_net[0:0]}),
		  .io({tmpIO_0__RED_net[0:0]}),
		  .siovref(tmpSIOVREF__RED_net));


	wire [0:0] tmpFB_0__GREEN_net;
	wire [0:0] tmpIO_0__GREEN_net;
	electrical [0:0] tmpSIOVREF__GREEN_net;

	cy_mxs40_gpio_v1_0
		#(.id("558b72c1-7ded-42f4-87ff-d2d6dc003b67"),
		  .width(1),
		  .sio_grp_cnt(0),
		  .drive_mode("6"),
		  .ibuf_enabled("0"),
		  .init_dr_st("1"),
		  .input_sync("0"),
		  .intr_mode("0"),
		  .io_voltage(""),
		  .output_conn("1"),
		  .oe_conn("0"),
		  .output_sync("0"),
		  .oe_sync("0"),
		  .drive_strength("0"),
		  .max_frequency("100"),
		  .i2c_mode("0"),
		  .output_current_cap("8"),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .slew_rate("0"),
		  .vtrip("0"),
		  .use_annotation("0"),
		  .hotswap_needed("0"))
		GREEN
		 (.oe({1'b1}),
		  .y({Net_120}),
		  .fb({tmpFB_0__GREEN_net[0:0]}),
		  .io({tmpIO_0__GREEN_net[0:0]}),
		  .siovref(tmpSIOVREF__GREEN_net));



    assign Net_122 = ~Net_120;

	wire [0:0] tmpIO_0__ModeChange_net;
	electrical [0:0] tmpSIOVREF__ModeChange_net;

	cy_mxs40_gpio_v1_0
		#(.id("4840f19c-8a6e-4a1a-be96-6311d243b716"),
		  .width(1),
		  .sio_grp_cnt(0),
		  .drive_mode("0"),
		  .ibuf_enabled("1"),
		  .init_dr_st("0"),
		  .input_sync("0"),
		  .intr_mode("0"),
		  .io_voltage(""),
		  .output_conn("0"),
		  .oe_conn("0"),
		  .output_sync("0"),
		  .oe_sync("0"),
		  .drive_strength("0"),
		  .max_frequency("100"),
		  .i2c_mode("0"),
		  .output_current_cap("8"),
		  .pin_aliases(""),
		  .pin_mode("I"),
		  .slew_rate("0"),
		  .vtrip("0"),
		  .use_annotation("0"),
		  .hotswap_needed("0"))
		ModeChange
		 (.oe({1'b1}),
		  .y({1'b0}),
		  .fb({Net_120}),
		  .io({tmpIO_0__ModeChange_net[0:0]}),
		  .siovref(tmpSIOVREF__ModeChange_net));




endmodule

