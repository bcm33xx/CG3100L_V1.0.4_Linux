//****************************************************************************
//
// Copyright(c) 2007-2008 Broadcom Corporation
//
// This program is the proprietary software of Broadcom Corporation and/or
// its licensors, and may only be used, duplicated, modified or distributed
// pursuant to the terms and conditions of a separate, written license
// agreement executed between you and Broadcom (an "Authorized License").
// Except as set forth in an Authorized License, Broadcom grants no license
// (express or implied), right to use, or waiver of any kind with respect to
// the Software, and Broadcom expressly reserves all rights in and to the
// Software and all intellectual property rights therein.  IF YOU HAVE NO
// AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY WAY,
// AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF THE
// SOFTWARE.  
//
// Except as expressly set forth in the Authorized License,
//
// 1.     This program, including its structure, sequence and organization,
// constitutes the valuable trade secrets of Broadcom, and you shall use all
// reasonable efforts to protect the confidentiality thereof, and to use this
// information only in connection with your use of Broadcom integrated circuit
// products.
//
// 2.     TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED
// "AS IS" AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES, REPRESENTATIONS
// OR WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH
// RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY DISCLAIMS ANY AND ALL
// IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS FOR
// A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR COMPLETENESS, QUIET
// ENJOYMENT, QUIET POSSESSION OR CORRESPONDENCE TO DESCRIPTION. YOU ASSUME
// THE ENTIRE RISK ARISING OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
//
// 3.     TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL BROADCOM
// OR ITS LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL, SPECIAL,
// INDIRECT, OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR IN ANY WAY
// RELATING TO YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN IF BROADCOM
// HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii) ANY AMOUNT IN
// EXCESS OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF OR U.S. $1,
// WHICHEVER IS GREATER. THESE LIMITATIONS SHALL APPLY NOTWITHSTANDING ANY
// FAILURE OF ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
//
//****************************************************************************
//  $Id$
//
//  Filename:       3380_UsbRegisters.h
//  Author:         Karthik Balasubramanian
//  Creation Date:  June 12, 2008
//
//****************************************************************************
//  Description:
//      This file defines the register structure of the 3380 USB OTG cores.
//
//*************************************************************************

#ifndef __BCM3380_USBREGISTERS_H
#define __BCM3380_USBREGISTERS_H

#include "bcmtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USB_NUM_ENDPOINTS   16
#define USB_NUM_RX_CHANNELS 8
#define USB_NUM_TX_CHANNELS 8


  // These shouldn't be required for the USB driver as
  // the MBDMA driver will manage the USB cores DMA channels
  // as well.  Anyhow, no harm in defining this here.
  typedef struct usb_mbdma_rxchan_regs
  {
	  uint32 chancontrol;
	  uint32 lanmsgaddress;
	  uint32 reserved[6];
  }usb_mbdma_rxchan_regs;

  typedef struct usb_mbdma_txchan_regs
  {
	  uint32 chancontrol;
	  uint32 lanmsgaddress;
	  uint32 chancontrol_2;
	  uint32 reserved[5];
  }usb_mbdma_txchan_regs;

  typedef struct usb_mbdma_regs
  {
	  uint32				status;
		#define GBL_INTR_MSK			(0x01 << 31)
		#define TX_MSGQ_OVFL_MSK		(0x01 << 24)
		#define INVALID_TX_MSG_MSK		(0x01 << 23)
		#define UBUS_ERROR_MSK			(0x01 << 22)
		#define TOKEN_RD_ERR_MSK		(0x01 << 21)
		#define INVALID_TOKEN_MSK		(0x01 << 20)
		#define ALLOC_FIFO_EMPTY_MSK	(0x01 << 19)
		#define ALLOC_FIFO_FULL_MSK		(0x01 << 18)
		#define FREE_FIFO_EMPTY_MSK		(0x01 << 17)
		#define FREE_FIFO_FULL_MSK		(0x01 << 16)
        #define TX_MSGQ_OVFL_STS        (0x01 << 8)
	    #define INVALID_TX_MSG_STS      (0x01 << 7)
	    #define UBUS_ERROR_STS          (0x01 << 6)
        #define TOKEN_RD_ERROR_STS      (0x01 << 5)
        #define INVALID_TOKEN_STS       (0x01 << 4)
        #define ALLOC_FIFO_EMPTY_STS    (0x01 << 3)
        #define ALLOC_FIFO_FULL_STS     (0x01 << 2)
        #define FREE_FIFO_EMPTY_STS     (0x01 << 1)
        #define FREE_FIFO_FULL_STS	    (0x01)
	  uint32 				tokencachectl;
		#define ALLOC_ENABLE 			(0x01 << 31)
		#define ALLOC_MAX_BURST_SHIFT 	(24)
		#define ALLOC_THRESH_SHIFT 		(16)
		#define FREE_ENABLE				(0x01 << 15)
		#define FREE_MAX_BURST_SHIFT	(8)
	  uint32				tokenaddress;
	  uint32				globalctl;
		#define FLUSH_CACHE			(0x01 << 31)
		#define ALLOC_LIMIT_SHIFT	(8)
	  uint32				bufferbase;
	  uint32				buffersize;
	  uint32				reserved[10];
	  usb_mbdma_rxchan_regs rx_chan_regs[USB_NUM_RX_CHANNELS];
	  usb_mbdma_txchan_regs tx_chan_regs[USB_NUM_TX_CHANNELS];
  		#define MAX_BURST_SHIFT (20)
		#define RX_MSG_ID_SHIFT (12)
		#define TX_MSG_ID_SHIFT (8)
		#define RX_MAC_ID_SHIFT	(8)
		#define TX_MAC_ID_SHIFT	(4)
	  uint32 				reserved_2[176];
	  uint32                lan_tx_msgfifo[USB_NUM_TX_CHANNELS]; 
	  
  }usb_mbdma_regs;

  typedef struct UsbOtgControllerRegs
  {
	  uint32 usb_otg_intr_status;
		#define USB_ACTIVE_CHANGE   (0x01 << 12)
		#define USB_PWRFLT_CHANGE   (0x01 << 11)
		#define USB_CH7_INTERRUPT   (0x01 << 10)
		#define USB_CH6_INTERRUPT   (0x01 << 9)
		#define USB_CH5_INTERRUPT   (0x01 << 8)
		#define USB_CH4_INTERRUPT   (0x01 << 7)
		#define USB_CH3_INTERRUPT   (0x01 << 6)
		#define USB_CH2_INTERRUPT   (0x01 << 5)
		#define USB_CH1_INTERRUPT   (0x01 << 4)
		#define USB_CH0_INTERRUPT   (0x01 << 3)
		#define USB_MSG_INTERRUPT   (0x01 << 2)
		#define USB_MBDMA_INTERRUPT (0x01 << 1)
		#define USB_OTG_INTERRUPT   (0x01 << 0)
	  uint32 usb_otg_intr_mask;
	  uint32 usb_otg_misc_reg;
		#define USB_MISC             (0x3f << 26)
		#define USB_TX_ENDIAN_CTRL   (0x01 << 25)
		#define USB_RX_ENDIAN_CTRL   (0x01 << 24)
		#define USB_IGNR_EXP_FIELD7  (0x01 << 7)
		#define USB_IGNR_EXP_FIELD6  (0x01 << 6)
		#define USB_IGNR_EXP_FIELD5  (0x01 << 5)
		#define USB_IGNR_EXP_FIELD4  (0x01 << 4)
		#define USB_IGNR_EXP_FIELD3  (0x01 << 3)
		#define USB_IGNR_EXP_FIELD2  (0x01 << 2)
		#define USB_IGNR_EXP_FIELD1  (0x01 << 1)
		#define USB_IGNR_EXP_FIELD0  (0x01 << 0)
	  uint32 usb_otg_ctrl_status;
	  uint32 otg_rx_ch_ctrl[USB_NUM_RX_CHANNELS];
	  uint32 otg_tx_ch_ctrl[USB_NUM_TX_CHANNELS];
		#define USB_OTG_CH_ENABLE (0x01)
	  uint32 usb_otg_ch_intst_mask[USB_NUM_RX_CHANNELS + USB_NUM_TX_CHANNELS];
	  uint32 usb_otg_dbg_ctrl;
		#define USB_BYP_PWRFLT_FILTER (0x01 << 15)
		#define USB_BYP_ACTIVE_FILTER (0x01 << 14)
		#define USB_OTG_SEL           (0x03)
	  uint32 usb_otg_dbg;
	  uint32 usb_otg_mbist_tm;

	  // Bits 31:16 BRT_PKT_SIZE, Bits 15:0 BRT_PKT_CNT
	  uint32 usb_otg_bert_ctl_1; 

	  //
	  // BERT configuration bits 
	  // B0 -> Bert reset 
	  // B1 -> Bert start 
	  // B2 -> Bert mode; BERT (1), UTMI (0)
	  // B3 -> Not used 
	  // B4 -> HS (0), FS (1)
	  // B5 -> Not used 
	  // B6 -> Test pattern; PRBS (0), Fixed (1)
	  // B7 -> Continuous play; packet count is ignored
	  // B15:8 -> Inter-packet gap
	  // B23:16 -> Fixed test pattern
	  // B27:24 -> [P3:P2:P1:P0] TX Port enable
	  // B31:28 -> [S3:S2:S1:S0] Status of RX port (mutually exclusive)
	  // 
	  uint32 usb_otg_bert_ctl_2;    
		#define USB_BRT_CTL_2_RESET                   (0x01 << 0)
		#define USB_BRT_CTL_2_START                   (0x01 << 1)
		#define USB_BRT_CTL_2_MODE                    (0x01 << 2)
		#define USB_BRT_CTL_2_SPEED                   (0x01 << 4)
		#define USB_BRT_CTL_2_PATTERN                 (0x01 << 6)
		#define USB_BRT_CTL_2_CNT_PLAY                (0x01 << 7)
		#define USB_BRT_CTL_2_INT_PKT_GAP_BITSHIFT    (8)
		#define USB_BRT_CTL_2_TEST_PATTERN_BITSHIFT   (16)
		#define USB_BRT_CTL_2_TX_PORT_ENABLE_BITSHIFT (24)

	  //
	  // BERT status register (Read only) 
	  // 31 -> RX done 
	  // 30 -> TX done 
	  // 29:28 -> Rx state 
	  //   00 -> Idle 
	  //   01 -> SYNC 
	  //   10 -> RCV 
	  //   11 -> DONE 
	  // 27:26 -> TX state 
	  //   00 -> Idle 
	  //   01 -> SYNC 
	  //   10 -> XFR 
	  //   11 -> GAP 
	  // 25:0 -> Error count / Bad bytes
	  // 
	  uint32 usb_otg_bert_stat_1;
		#define USB_BERT_STAT_RX_DONE  (0x01 << 31)
		#define USB_BERT_STAT_TX_DONE  (0x01 << 30)
		#define USB_BERT_STAT_RX_STATE (0x03 << 28)
		#define USB_BERT_STAT_TX_STATE (0x03 << 26)

	  //
	  //  BERT status register (Read only) 
	  // 15:0 -> Rx packet size 
	  // 31:16 -> Rx packet count 
	  // 
	  uint32 usb_otg_bert_stat_2;

	  // 
	  // 29 -> DISABLE_UTMI_SRP: 1 - Session Request Protocol(SPR) not supported  
	  // 28 -> SHDN_SEL: 1 - SHDN for charge pump is always generated (Host & Device mode)
	  // 27 -> OTG_MODE:  1 - PHY in OTG Mode  
	  // 26 -> HOSTB_DEV: 
	  //       0 - Host
	  //       1 - Device 
	  // 25 -> PHY_PWDNB: 
	  //       0 - Analog drivers powered off
	  //       1 - Analog drivers powered on  
	  // 24 -> UTMI_PWDNB: 
	  //      0 - UTMI digital clocks stopped
	  //      1 - UTMI digital operational  
	  // 23 -> UTMI_SOFT_RESETB: Port UTMI software reset ( active low )  
	  // 21 -> DISCON_PHY: 1 - Causes the host to see a disconnect on the next SOF frame. 
	  // 20 -> TX_PHASE: Flips the 480Mhz phase in the bit stack logic. Purely digital loopback logic.  
	  // 19 -> CHIRP_RX: 
	  //       0 - digital mode
	  //       1 - analog mode
	  // 18:16 -> SYNC_DET_LENG  
	  //          3'b000:  sync_detect = 4 bits 
	  //          3'b001:  sync_detect = 2 bits 
	  //          3'b010:  sync_detect = 3 bits 
	  //          3'b011:  sync_detect = 4 bits 
	  //          3'b100:  sync_detect = 5 bits 
	  //          3'b101:  sync_detect = 6 bits 
	  //          3'b110:  sync_detect = 7 bits 
	  //          3'b111:  sync_detect = 8 bits  
	  // 15 -> Remote wake-up enable in the host mode 
	  // 14 -> Not used 
	  // 13 -> Non-driving for port1 
	  // 12 -> Non-driving for port0 
	  // 11 -> afe_chrpten; When set to 1 enables the chirp transmit. For test only. 
	  // 10 -> AFE loopback. When set to 1, causes the TX data to come back into Rx of the same port.
	  // 9 -> afe_cdrcken && afe_clken When set to 1 will force the 960/480/120 clocks ON. 
	  // 8 -> Disables the Digital PLL lock when set high 
	  // 7 -> afe_hstxen; When set to 1 will force the HS current To be enabled. 
	  // 6 -> Bug fix for rx_state_idle 
	  // 5 -> Disables bit stuffing and EOP errors in the UTMI logic. 
	  // 4 -> afe_rxlogicr; This is the CDR 480 clock enable forRCV, during a non-bert loopback this may be turned on. 
	  // 3-2 -> Direct control over the iost1 and iost0 in HS mode only
	  //        00:  normal operation 
	  //        01: DP=0, DM=1 
	  //        10: DP=1, DM=0 
	  //        11: DP=Z, DM=Z 
	  // 1 -> resume_filterb 
	  // 0 -> Bit stuff error enable 
	  // 
	  uint32 usb_otg_utmi_ctl_1;
		#define USB_UTMI_DISABLE_SRP           (0x01 << 29)
		#define USB_UTMI_SHDN_SEL              (0x01 << 28)
		#define USB_UTMI_OTG_MODE              (0x01 << 27)
		#define USB_UTMI_HOSTB_DEV             (0x01 << 26)
		#define USB_UTMI_PHY_PWDNB             (0x01 << 25)
		#define USB_UTMI_PWNDB                 (0x01 << 24)
		#define USB_UTMI_SOFT_RESETB           (0x01 << 23)
		#define USB_UTMI_DISCON_PHY            (0x01 << 21)
		#define USB_UTMI_TX_PHASE              (0x01 << 20)
		#define USB_UTMI_CHIRP_RX              (0x01 << 19)
		#define USB_UTMI_HOSTMODE_REMOTE_WK_EN (0x01 << 15)

      uint32 usb_otg_test_port_ctl;

	  //
	  // 31:10 -> pll ctl spare  
	  // 09 -> PLL_RESET: Reset analog PLL and dividers  
	  // 08 -> PHYPLLBYP: USB phy pll bypass control. 
	  //       0 : PLL bypass default ( pll not bypassed for normal operation; pll bypassed for testmodes ) 
	  //       1 : PLL bypass invert  ( pll bypassed for normal operation; pll not bypassed for testmodes )  
	  // 07:06 -> CLKSEL: Select  REFCLK Freq - (00)30MHz, (01)48MHz, (10)24MHz.  
	  // 05 -> PLL_SUSPEND_EN: PLL shut-down is disabled in suspend mode when 1'b1  
	  // 04 -> PLL_CALEN: 1 - PLL calibration enable  
	  // 03 -> PLL_PWRDWNB: PLL power down. 
	  //       1: PLL powered on 
	  //       0: PLL powered down. 
	  // 02 -> XTAL_PWRDWNB: Xtal Osc power down.
	  //       1: Xtal Osc powered on 
	  //       0: Xtal Osc powered down.  
	  // 01:00 -> REFCLKSEL 
	  //          2'b00 : Selects REFCLK from XTALI input pad 
	  //          2'b01 : Selects REFCLK from XTAL Oscillator 
	  //          2'b10 : Selects REFCLK from an internal 48MHz / 30Mhz Diff clk 
	  //          2'b11 : Selects REFCLK from an internal 48MHz / 30Mhz Single ended clk  
	  // 
	  uint32 usb_otg_pll_ctl_1;
		#define USB_PLL_CTL_RESET   (0x01 << 9)
		#define USB_PLL_PHYPLL_BYP  (0x01 << 8)
		#define USB_PLL_CLKSEL      (0x03 << 6)
		#define USB_PLL_SUSPEND_EN  (0x01 << 5)
		#define USB_PLL_CALEN       (0x01 << 4)
		#define USB_PLL_PWRDNB      (0x01 << 3)
		#define USB_PLL_XTAL_PWRDNB (0x01 << 2)
		#define USB_PLL_REFCLKSEL   (0x3)

	  //
	  // 31:04 -> Reserved  
	  // 03 -> USB_ACTIVE: 
	  //       1 - USB active
	  //       0 - USB inactive 
	  // 02 -> USB_PWRFL:  
	  //       1 - USB power fault
	  //       0 - USB power normal 
	  // 01 -> USB_PWRFL_POLARITY: 
	  //       1 - USB_PWRFL active low
	  //       0 - USB_PWRFL active high 
	  // 00 -> USB_PWR_ON:  Turn on USB POWER via GPIO 
	  // 
	  uint32 usb_otg_gpio;
		#define USB_GPIO_USB_ACTIVE         0x08
		#define USB_GPIO_USB_PWRFL          0x04
		#define USB_GPIO_USB_PWRFL_POLARITY 0x02
		#define USB_GPIO_USB_PWR_ON         0x01

  } UsbOtgControllerRegs;


  //
  // gotgctl: “Control and Status Register (GOTGCTL)” on page 198
  // gotgint: “Interrupt Register (GOTGINT)” on page 200
  // gahbcfg: “AHB Configuration Register (GAHBCFG)” on page 201
  // gusbcfg: “USB Configuration Register (GUSBCFG)” on page 203
  // grstctl: “Reset Register (GRSTCTL)” on page 209
  // gintsts: “Interrupt Register (GINTSTS)” on page 213
  // gintmsk: “Interrupt Mask Register (GINTMSK)” on page 219
  // grxstsr: “Receive Status Debug Read/Status Read and Pop Registers
  // grxstsr/grxstsp: on page 221
  // grxfsiz: “Receive FIFO Size Register (GRXFSIZ)” on page 223
  // gnptxfsiz: “Non-Periodic Transmit FIFO Size Register (GNPTXFSIZ)” on page 224
  // gnptxsts: “Non-Periodic Transmit FIFO/Queue Status Register (GNPTXSTS)” on page 226
  // gi2cctl: “I2C Access Register (GI2CCTL)” on page 227
  // gpvndctl: “PHY Vendor Control Register (GPVNDCTL)” on page 229
  // ggpio: “General Purpose Input/Output Register (GGPIO)” on page 230
  // guid: “User ID Register (GUID)” on page 230
  // gsnpsid: “Synopsys ID Register (GSNPSID)” on page 231
  // ghwcfg1: “User HW Config1 Register (GHWCFG1)” on page 231
  // ghwcfg2: “User HW Config2 Register (GHWCFG2)” on page 232
  // ghwcfg3: “User HW Config3 Register (GHWCFG3)” on page 234
  // ghwcfg4: “User HW Config4 Register (GHWCFG4)” on page 236
  // 054h–0FFh: Reserved
  // hptxfsiz: “Host Periodic Transmit FIFO Size Register (HPTXFSIZ)” on page 238
  // dptxfsizn: Dedicated FIFOYes “Device Periodic Transmit FIFO-n Size Register (DPTXFSIZn)” on page 239
  // dieptxfn: Dedicated FIFO Yes “Device IN Endpoint Transmit Fifo Size Register: (DIEPTXFn)”
  // 
  typedef struct OtgCoreGlblRegs
  {
	  uint32 otg_ctl;
	  uint32 otg_int;
	  uint32 ahbc_cfg;
		#define DMA_MODE_EN 		(0x01 << 5)
		#define GLOBAL_INTR_EN		(0x01)
	  uint32 usb_cfg;
	  uint32 rst_ctl;
	  uint32 int_sts;
	  uint32 int_msk;
		#define USB_SPEED_ENUM_DONE (0x01 << 13)
		#define USB_RESET           (0x01 << 12)
		#define USB_SUSPEND         (0x01 << 11)
		#define USB_EARLY_SUSPEND   (0x01 << 10)
		#define USB_RXFIFO_LVL		(0x01 << 4)
		#define USB_OTG_INTR		(0x01 << 2)
	  uint32 rxsts_r;
	  uint32 grxstsr;
	  uint32 grxfsiz;
	  uint32 gnptxfsiz;
	  uint32 gnptxsts;
	  uint32 gi2cctl;
	  uint32 gpvndctl;
	  uint32 ggpio;
	  uint32 guid;
	  uint32 gsnpsid;
	  uint32 ghwcfg1;
	  uint32 ghwcfg2;
	  uint32 ghwcfg3;
	  uint32 ghwcfg4;
	  uint8  reserved_1[0xff - 0x54];
	  uint32 hptxfsiz;
	  uint32 dptxfsizn[15];
	  uint32 reserved_2[176];
  } OtgCoreGlblRegs;

  // Undefined for now.  Just account for
  // size with a reserved array.  We'll define
  // the regs when required.
  typedef struct OtgCoreHostRegs
  {
	  uint32 reserved[0x100];
  } OtgCoreHostRegs;

  typedef struct OtgDeviceEpCtlRegs
  {
	  uint32 epctl;
		#define ENDPOINT_ENABLE    (0x01 << 31)
		#define ENDPOINT_DISABLE   (0x01 << 30)
		#define ENDPOINT_SETD1PID  (0x01 << 29)
		#define ENDPOINT_SETD0PID  (0x01 << 28)
		#define ENDPOINT_SET_NAK   (0x01 << 27)
		#define ENDPOINT_CLR_NAK   (0x01 << 26)
        #define ENDPOINT_STALL     (0x01 << 21)
		#define ENDPOINT_ACTIVE    (0x01 << 15)
		// These pre-defined MPS values only apply
	    // to the Control Endpoints. For other end-
		// points, we'll set the value.
		#define MPS_64_BYTES       (0x00)
		#define MPS_32_BYTES       (0x01)
		#define MPS_16_BYTES       (0x02)
		#define MPS_8_BYTES        (0x03)
		#define ENDPOINT_TYPE_CTL  0x00
		#define ENDPOINT_TYPE_ISO  0x01
		#define ENDPOINT_TYPE_BLK  0x02
		#define ENDPOINT_TYPE_INT  0x03
	  uint32 reserved_1;
	  uint32 epint;
	  uint32 reserved_2;
	  uint32 eptsiz;
	  uint32 epdma;
	  uint32 dtxfsts;
	  uint32 epdmab;
  } OtgDeviceEpCtlRegs;

  typedef struct OtgCoreDeviceRegs
  {
      uint32 dcfg;
	  uint32 dctl;
		#define DEVICE_SOFT_DISCON (0x01 << 1)
	  uint32 dsts;
		#define DEVICE_ENUM_SPEED (0x3 << 1)
	  uint32 unused_1;
	  uint32 diepmsk;
	  uint32 doepmsk;
	  uint32 daint;
	  uint32 daintmsk;
	  uint32 dtknqr1;
	  uint32 dtknqr2;
	  uint32 dvbusdis;
	  uint32 dvbuspulse;
	  uint32 dthrctl;
	  uint32 diepempmsk;
	  uint8  unused_2[0x8FC - 0x834];
	  OtgDeviceEpCtlRegs in_endpt_ctl_regs[USB_NUM_ENDPOINTS];
	  OtgDeviceEpCtlRegs out_endpt_ctl_regs[USB_NUM_ENDPOINTS];
  } OtgCoreDeviceRegs;


  typedef struct UsbOtgRegs
  {
	  OtgCoreGlblRegs      glbl_regs;
	  OtgCoreHostRegs      host_regs;
	  OtgCoreDeviceRegs device_regs;
  } UsbOtgRegs;

#ifdef __cplusplus
}
#endif

#endif

