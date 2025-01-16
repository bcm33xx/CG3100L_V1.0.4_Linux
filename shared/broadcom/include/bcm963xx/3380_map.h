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
//  Filename:       3380_map.h
//  Author:         Russell Enderby
//  Creation Date:  June 13, 2008
//
//****************************************************************************
//  Description:
//      This file defines addresses of major hardware components of 3380
//
//****************************************************************************

#ifndef __BCM3380_MAP_H
#define __BCM3380_MAP_H

#include "bcmtypes.h"
#include "3380_UsbRegisters.h"
#include "bcm3380.h"
#include "bcmmacro.h"

#ifdef __cplusplus
extern "C" {
#endif

#define _BCM_IUDMA          // IUDMA architecture
#define _BMIPS4350          // MIPS core is BMIPS4350
#define _LED_CONTROL_GPIO   // LED control in GPIO register block


// Helper macro to read the 3380 manufacturing bits.
#define MANUFACT_BITS_3380 ((REG32(0xfff8c20c) >> 4) & 0x0f)

#define MANUFACT_BITS_3380_0 0x00
#define MANUFACT_BITS_3380_f 0x0f
#define MANUFACT_BITS_3380_e 0x0e
#define MANUFACT_BITS_3380_d 0x0d


// ------------------- Physical Memory Map -----------------------------------

#define PHYS_DRAM_BASE           0x00000000     // Dynamic RAM Base

// provide master (host) base address for consistency - use with access macros
#define MASTER_BASE_ADDR         0xb2080000

// ***************************************************************************
// Note that the addresses above are physical addresses and that programs
// have to use converted addresses defined below:
// ***************************************************************************
#if defined (TARGETOS_Linux) || defined (TARGETOS_Qnx6)

    // for Linux/Qnx implementation virtual addresses MUST be mapped at runtime
    extern uint32 DRAM_BASE;               // cached DRAM
    extern uint32 DRAM_BASE_NOCACHE;       // uncached DRAM
    extern uint32 DRAM_PARAMETER_PAGE;
    extern uint32 EPROM_ADDR;              // cached EPROM
    extern uint32 EPROM_ADDR_NOCACHE;      // uncached EPROM
    extern uint32 FLASH_BASE;              // uncached Flash
    extern uint32 FLASH_BASE_2;            // uncached Flash
    extern uint32 DSP_BASE;                // uncached DSP
#else
//    #define DRAM_BASE           PHYS_TO_K0(PHYS_DRAM_BASE)     // cached DRAM
//    #define DRAM_BASE_NOCACHE   PHYS_TO_K1(PHYS_DRAM_BASE)     // uncached DRAM
    #define DRAM_PARAMETER_PAGE PHYS_TO_K0(PHYS_DRAM_PARAMETER_PAGE)
#endif

// matches isb_decoder.v
#if defined (TARGETOS_Linux) || defined (TARGETOS_Qnx6)

    // for Linux/Qnx implementation virtual addresses MUST be mapped at runtime
    extern uint32 DDR2_BASE;       // DDR2 dram registers
    extern uint32 FPM_BASE;        // free pool manager (FPM) registers
    extern uint32 OLD_MAC_BLOCK;   // legacy MAC block
    extern uint32 SEGDMA_BLOCK;    // segdma block
    extern uint32 TC_BLOCK_0;      // transmission convergence
    extern uint32 TC_BLOCK_1;      // transmission convergence
    extern uint32 TC_BLOCK_2;      // transmission convergence
    extern uint32 TC_BLOCK_3;      // transmission convergence
    extern uint32 USMAC30_BLOCK;   // US MAC30 block
    extern uint32 US_BASE;         // upstream phy registers
    extern uint32 CRYPTO_BASE;     // crypto registers
    extern uint32 BRIDGE_BLOCK;    // UBUS bridge
    extern uint32 DAVIC_BASE;      // DAVIC registers
    extern uint32 UTP_BLOCK;       // UTP registers
    extern uint32 DTP_BLOCK;       // DTP registers
    extern uint32 FAP_BLOCK;       // FAP registers
    extern uint32 MEP_BLOCK;       // MEP registers
    extern uint32 MSP_BLOCK;       // MSP registers
    extern uint32 DSMAC30_BLOCK;   // DS MAC30 Block
    extern uint32 DS_TOP_BLOCK;    // downstream top block
    extern uint32 OOB_TOP_BLOCK;   // out of band receiver registers
    extern uint32 PERIPH_BLOCK;    // periph block
    extern uint32 SPI_MSTER_BLOCK; // serial port interface registers
    extern uint32 USB_OTG_BLOCK_0; // USB OTG block 0 registers
    extern uint32 USB_OTG_BLOCK_1; // USB OTG block 1 registers
    extern uint32 APM_BLOCK;       // APM block
    extern uint32 BMU_BLOCK;       // BMU block
    extern uint32 PICO_IMEM;       // PICO registers

#else
    // DDR2 Block
    #define DDR2_BASE       0xb2000000   // DDR2 dram registers

    // FPM Block
    #define FPM_BASE        0xb2010000   // free pool manager (FPM) registers

    // Legacy MAC block
    #define OLD_MAC_BLOCK   0xb2020000   // legacy MAC block

    // SegDma Block
    #define SEGDMA_BLOCK    0xb2030000   // segdma block

    // transmission convergence
    // TC0 Block (legacy multi us channel macro implementation)
    #define TC_BASE        (((unsigned long)UpstreamPhyChannelNumber * 0x1000) + 0xb2034000)

    // TC0 Block
    #define TC_BLOCK_0      0xb2034000   // transmission convergence

    // TC1 Block
    #define TC_BLOCK_1      0xb2035000   // transmission convergence

    // TC2 Block
    #define TC_BLOCK_2      0xb2036000   // transmission convergence

    // TC3 Block
    #define TC_BLOCK_3      0xb2037000   // transmission convergence

    // US MAC30 Block
    #define USMAC30_BLOCK   0xb2038000   // US MAC30 block

    // Upstream Block
    #define US_BASE         0xb203c000   // upstream phy registers
	#define US_TOP_BASE     0xb203c000
	#define US_PDAC_BASE    (0xb203c200 + (UpstreamPhyChannelNumber * 0x80))
	#define US_CORE_BASE    (0xb203c800 + (UpstreamPhyChannelNumber * 0x200))

    // Crypto Block
    #define CRYPTO_BASE     0xb2040000   // crypto registers

    // GMAC0 Ethernet Block
    #define GMAC0_BLOCK     0xb2100000   // GMAC0 registers

    // GMAC1 Ethernet Block
    #define GMAC1_BLOCK     0xb2110000   // GMAC1 registers

    // DAVIC Block
    #define DAVIC_BASE      0xb2120000   // DAVIC registers

    //  IOP's Start Here
    ////////////////////

    // Upstream Token Processor
    #define UTP_BLOCK       0xb4000000   // UTP registers
    #define UTP_BLOCK_SMISB 0xb8000000   // UTP registers accessed from the SMISB Mips Bus

    // Downstream Token Processor
    #define DTP_BLOCK       0xb4200000   // DTP registers
    #define DTP_BLOCK_SMISB 0xb8200000   // DTP registers accessed from the SMISB Mips Bus

    // Forwarding Assist Processor
    #define FAP_BLOCK       0xb4400000   // FAP registers
    #define FAP_BLOCK_SMISB 0xb8400000   // FAP registers accessed from the SMISB Mips Bus

    // MPEG Encapsulation Processor
    #define MEP_BLOCK       0xb4600000   // MEP registers
    #define MEP_BLOCK_SMISB 0xb8600000   // MEP registers accessed from the SMISB Mips Bus

    // Message Processing Processor
    #define MSP_BLOCK       0xb5800000   // MSP registers
    #define MSP_BLOCK_SMISB 0xb8800000   // MSP registers accessed from the SMISB Mips Bus

    //  IOP's End Here
    ////////////////////

    // DS MAC30 Block
    #define DSMAC30_BLOCK   0xb4c00000   // DS MAC30 Block

    // Downstream Block
    #define DS_TOP_BLOCK    0xb4c10000   // downstream top block
	#define DS_BASE    (0xb4c10000 + (ChannelNumber * 0x2000))
        #define DS_CORE0    0xb4c10000   // downstream phy0 registers
        #define DS_CORE1    0xb4c12000   // downstream phy1 registers
        #define DS_CORE2    0xb4c14000   // downstream phy2 registers
        #define DS_CORE3    0xb4c16000   // downstream phy3 registers
        #define DS_CORE4    0xb4c18000   // downstream phy4 registers
        #define DS_CORE5    0xb4c1a000   // downstream phy5 registers
        #define DS_CORE6    0xb4c1c000   // downstream phy6 registers
        #define DS_CORE7    0xb4c1e000   // downstream phy7 registers
//      #define DS_TUNR_TOP 0xb4c20000   // downstream tuner top
        #define DS_TUNR_CR0 0xb4c24000   // downstream tuner core0 registers
        #define DS_TUNR_CR1 0xb4c28000   // downstream tuner core1 registers
        #define DS_TUNR_CZT 0xb4c2c000   // downstream tuner channelizer registers
		#define DS_TUNER_ANALOG_BASE (0xb4c22000 + (TunerNumber * 0x1000))
		#define DS_TUNER_CORE_BASE   (0xb4c24000 + (TunerNumber * 0x4000))

    // Out of Band Receiver Block
    #define OOB_BASE        0xb4c30000   // out of band receiver registers

    // Perhipheral Block
    #define PERIPH_BLOCK    0xb4e00000   // periph block
        #define INTC_BASE   0xb4e00000   // interrupts controller registers
        #define TIMR_BASE   0xb4e000c0   // timer registers
        #define GPIO_BASE   0xb4e00100   // gpio registers
        #define UART_BASE   0xb4e00200   // uart registers
        #define UART_BASE1  0xb4e00220   // uart registers
        #define I2C_BASE    0xb4e00e00   // uart registers
        #define LED_BASE    0xb4e00f00   // led  registers

    // SPI Master Block
    #define SPIM_BASE       0xb4e02000   // serial port interface registers

    // Bridge Block
    #define BRIDGE_BLOCK    0xb4e10000   // UBUS bridge

    // USB OTG  0
    #define USB_OTG_BLOCK_0 0xb5000000   // USB OTG block 0 registers

    // USB OTG 1
    #define USB_OTG_BLOCK_1 0xb5200000   // USB OTG block 1 registers

    #define USB_OTG_CTRLR0_BASE    0xb5000600    // USB OTG Controller 1 registers
    #define USB_OTG_CORE0_BASE     0xb5040000    // USB OTG Core 1 registers

    #define USB_OTG_CTRLR1_BASE    0xb5200600    // USB OTG Controller 2 registers
    #define USB_OTG_CORE1_BASE     0xb5240000    // USB OTG Core 2 registers


    // APM Telephony Block
    #define APM_BLOCK       0xb5400000   // APM block

    // BMU Block
    #define BMU_BLOCK       0xb5401000   // BMU block

    // PICO IMEM Block
    #define PICO_IMEM       0xb5410000   // PICO registers

#endif

// Overlay the entire chip into our auto generated register structures shared by HW
#define CHIP3380     ((volatile Bcm3380A0 * const) 0xa0000000)

#define PERIPH       ((volatile Periph * const)               &(CHIP3380->PeriphBlock))
#define INTC         ((volatile IntControlRegs * const)       &(CHIP3380->PeriphBlock.Int))
#define GPIO         ((volatile GpioRegs * const)             &(CHIP3380->PeriphBlock.Gpio))
#define LEDS         ((volatile LedRegs * const)              &(CHIP3380->PeriphBlock.Led))

#define GMAC0        ((volatile Unimac * const)               &(CHIP3380->UnimacBlock0))
#define GMAC1        ((volatile Unimac * const)               &(CHIP3380->UnimacBlock1))
#define FPM          ((volatile Fpm * const)                  &(CHIP3380->FpmBlock))
#define MSP          ((volatile FapBlockFapBlock * const)     &(CHIP3380->MspBlock))
#define DS_TUNER_TOP ((volatile DsTunerTopDsTunerTop * const) &(CHIP3380->DsTopBlock.DsTunerTop))

#define UBUS_BRIDGE  ((volatile Bridge * const)                  BRIDGE_BLOCK)

//---------------------------------------------------------------------------------------------
// these definitions correspond to the associated DMA channel (DmaChannel) from DMA_BASE
//---------------------------------------------------------------------------------------------
// DMA channel assignments
#define EMAC_RX_CHAN       1
#define EMAC_TX_CHAN       2
#define EMAC2_RX_CHAN      3
#define EMAC2_TX_CHAN      4
#define USB_CNTL_RX_CHAN   5
#define USB_CNTL_TX_CHAN   6
#define USB_BULK_RX_CHAN   7
#define USB_BULK_TX_CHAN   8


//---------------------------------------------------------------------------------------------
// IntControlRegs from PeriphBlock.Int DEFINITIONS
//---------------------------------------------------------------------------------------------
// External interrupt configuration bits
// Bit definitions for both external interrupt config registers 0,1
// 29:24 - level sense      (1 = enable level detection)
// 23:18 - edge insensitive (1 = trigger on both rising and falling edge)
// 17:12 - mask             (1 = enable)
// 11:6  - read interrupt status, write 1 to clear interrupt
//  5:0  - 0 = low level or falling edge, 1 = high level or rising edge
#define INTC_EXT_LEVEL_SHIFT    24
#define INTC_EXT_EDGE_SHIFT     18
#define INTC_EXT_MASK_SHIFT     12
#define INTC_EXT_STATUS_SHIFT    6
#define INTC_EXT_SENSE_SHIFT     0

#define PeriphIrqMask   INTC->PeriphIrqmask2.Reg32
#define PeriphIrqStatus INTC->PeriphIrqstatus2.Reg32

// IntControlRegs TimerControl bits
#define SOFT_RESET              1



//************************************************************************
//* Coprocessor 0 Status Register Bits
//************************************************************************

#define CP0SR_COP3   (1<<31)
#define CP0SR_COP2   (1<<30)
#define CP0SR_COP1   (1<<29)
#define CP0SR_COP0   (1<<28)
#define CP0SR_IST    (1<<23)
#define CP0SR_BEV    (1<<22)
#define CP0SR_SWC    (1<<17)
#define CP0SR_ISC    (1<<16)
#define CP0SR_KRNL   (1<<1)
#define CP0SR_IE     (1<<0)
#define CP0SR_IM5    (1<<15)
#define CP0SR_IM4    (1<<14)
#define CP0SR_IM3    (1<<13)
#define CP0SR_IM2    (1<<12)
#define CP0SR_IM1    (1<<11)
#define CP0SR_IM0    (1<<10)
#define CP0SR_SWM1   (1<<9)
#define CP0SR_SWM0   (1<<8)


//************************************************************************
// Coprocessor 0 Cause Register Bits
//************************************************************************
// Notes: 5:2 hold exception cause
// Notes: 29:28 hold Co-processor Number reference by Coproc unusable excptn
// Notes: 7:6, 1:0, 27:14, 30 ***UNUSED***
#define CP0CR_BD              (1<<31)
#define CP0CR_EXTIRQ4         (1<<14)
#define CP0CR_EXTIRQ3         (1<<13)
#define CP0CR_EXTIRQ2         (1<<12)
#define CP0CR_EXTIRQ1         (1<<11)
#define CP0CR_EXTIRQ0         (1<<10)
#define CP0CR_SW1             (1<<9)
#define CP0CR_SW0             (1<<8)
#define CP0CR_EXC_CAUSE_MASK  (0xf << 2)
#define CP0CR_EXC_COP_MASK    (0x3 << 28)

#define CP0_CMT_TPID (1<<31)

#define BCM3350_CPU_EXT_IRQ_MASK    (CP0CR_EXTIRQ1|CP0CR_EXTIRQ2|CP0CR_EXTIRQ3|CP0CR_EXTIRQ4)

// macro to convert logical data addresses to physical
// DMA hardware must see physical address
#define LtoP( x ) ( (uint32)x & 0x1fffffff )
#define PtoL( x ) ( LtoP(x) | 0xa0000000 )



typedef struct MemControllerRegs
{
    uint32  CS_Control[0x40];    // 000 - The CS control registers will be set in the bootloader.

    uint32  InitControl;        // 100 - This is the DDR command register.
        #define MEMC_CMD_EMRS       0x00  // Write the extended mode register (EMR) (BA[1:0]=2'b01)
        #define MEMC_CMD_MRS        0x01  // Write the mode register (MR) (BA[1:0]=2'b00)
        #define MEMC_CMD_PRE        0x02  // Precharge all banks command
        #define MEMC_CMD_AR         0x03  // Auto Refresh command
        #define MEMC_CMD_SET_SREF   0x04  // Set Self-Refresh
        #define MEMC_CMD_CLR_SREF   0x05  // Clear Self-Refresh
        #define MEMC_CMD_SET_PDN    0x06  // Set power-down, results in CKE being reset
        #define MEMC_CMD_CLR_PDN    0x07  // Clear power-down, results in CKE being set
        #define MEMC_CMD_EMRS2      0x08  // Write the extended mode2 register (EMR2) (BA[1:0]=2'b10)
        #define MEMC_CMD_EMRS3      0x09  // Write the extended mode3 register (EMR3) (BA[1:0]=2'b11)
        #define MEMC_CMD_SET_IDLE   0x0a  // Turn off DDR Pads (Based on DDR_PHY IDDQ Settings)
        #define MEMC_CMD_CLR_IDLE   0x0b  // Turn on DDR Pads  (Need to wait for 10ns)
        #define MEMC_CMD_UPDT_SVDL  0x0c  // Update Static VDL (one calibration cycle)
        #define MEMC_CMD_Reserved   0x0d  // Reserved
        #define MEMC_CMD_SET_DVDL   0x0e  // Switch to Dynamic VDL values
        #define MEMC_CMD_CLR_DVDL   0x0f  // Switch back to Static VDL values
        #define MEMC_CMD_CS0        (1 << 4)
        #define MEMC_CMD_CS1        (1 << 5)
        #define MEMC_SELF_REFRESH   (MEMC_CMD_CS0 | MEMC_CMD_CS1 | MEMC_CMD_SET_SREF) // enable self refresh mode

    uint32  Mode0;                  // 104
    uint32  Mode1;                  // 108
    uint32  Refresh;                // 10c
    uint32  ODT;                    // 110
    uint32  Timing0;                // 114
    uint32  Timing1;                // 118
    uint32  Timing2;                // 11c
    uint32  CRC;                    // 120
    uint32  OutCRC;                 // 124
    uint32  InCRC;                  // 128
    uint8   unused[0x800 - 0x12c];  // 12c

    uint32  GlobalConfig;           // 800
    uint32  LbistConfig;            // 804
    uint32  LbistSeed;              // 808
    uint32  Arbitor;                // 80c
    uint32  PIGlobalControl;        // 810
    uint32  PIUbusControl;          // 814
    uint32  PIMipsControl;          // 818
    uint32  PIDslMipsControl;       // 81c
    uint32  PIDslPhyControl;        // 820
    uint32  PIUbusPhase;            // 824
    uint32  PIMipsPhase;            // 828
    uint32  PIDslMipsPhase;         // 82c
    uint32  PIDslPhyPhase;          // 830
    uint32  PIUbusSample;           // 834

} MemControllerRegs;

#define MEMC ((volatile MemControllerRegs * const) MEMC_BASE)



typedef struct IobufRegs
{
    uint32  RevId;              // a0
    uint32  PadSstlMode;        // a4
    uint32  CommandPadControl;  // a8
    uint32  DqPadControl;       // ac
    uint32  DqsPadControl;      // b0
    uint32  ClkPadControl;      // b4
    uint32  MipsDdrPllConfig;   // b8
    uint32  PllDeskew;          // bc
    uint32  MipsPhaseControl;   // c0
    uint32  Ddr12PhaseControl;  // c4
    uint32  Ddr34PhaseControl;  // c8
    uint32  VcdlPhaseControl;   // cc
    uint32  Misc;               // d0
    uint32  Spare0;             // d4
    uint32  Spare1;             // d8
    uint32  Spare2;             // dc
    uint32  LBistControl;       // e0
    uint32  LBistCrc;           // e4
} IobufRegs;

#define IOBUF ((volatile IobufRegs * const) IOBUF_BASE)

#define PCI_ADDRMATCH0_BASE_0       0xA0000000
#define PCI_ADDRMATCH0_BASE_1       0xA1000000
#define PCI_ADDRMATCH_SIZES         0x01000000     // this is actually only 4Kb
#define PCI_VIRTUALADDR_TO_PHYADDR  0xE0000000     // Allow software access to physical space that is outside
                                                   // of MIPS capabilities. This virtual address is mapped to
                                                   // the PCI_ADDRMATCH0_BASE_0 address via a TLBMAP setup

typedef struct DMACONFIG
{
    uint32  Control;
		#define	PCI_DMA_LE			0x00000200
		#define	PCI_DMA_NOSWAP		0x00000100
		#define	PCI_DMA_HALT		0x00000008
		#define	PCI_DMA_PKTSTALL 	0x00000004
		#define	PCI_DMA_STALL		0x00000002
		#define	PCI_DMA_ENABLE		0x00000001
    uint32  IntStat;
    uint32  IntMask;
        #define PCI_DMA_NOTMY_DESC  0x00000004
        #define PCI_DMA_PKT_DONE    0x00000002
        #define PCI_DMA_DESC_DONE   0x00000001
    uint32  unused1;
    uint32  DescriptorAddress;
    uint32  Status1;
    uint32  Status2;
    uint32  unused2;
} LOCDmaConfig;

typedef struct PCIDMACONFIG
{
    uint32  Control;
    uint32  IntStat;
    uint32  IntMask;
    uint32  DescriptorAddress;
    uint32  Status1;
    uint32  Status2;
} PCIDmaConfig;

typedef struct MpiRegisters
{
    struct
    {
        uint32  Range;
        uint32  Remap;
        uint32  Config;
    } PciToSysBusAddressSpace[2];   // 00
    uint8   unused1[4];             // 18
    uint32  SysBusToPciIoControl;   // 1c
    struct
    {
        uint32  Range;
        uint32  Base;
        uint32  Remap;
    } SysBusToPciAddressSpace[3];   // 20
    uint32  PciModeSelect;          // 44
    uint32  PciIntStat;             // 48
    uint32  PciLocBusControl;       // 4c
    uint32  PciLocIntMaskStatus;    // 50
      #define PCI_LOCINTMASK_MASK         0x03ef0000
      #define PCI_EXTERNAL_INTMASK_BIT     (1<<23)
      #define PCI_MBOX1_SEND_INTENB_BIT    (1<<19)
      #define PCI_MBOX0_SEND_INTENB_BIT    (1<<18)
      #define PCI_MBOX1_RCV_INTENB_BIT     (1<<17)
      #define PCI_MBOX0_RCV_INTENB_BIT     (1<<16)
      #define PCI_EXTERNAL_INTSTATUS_BIT  (1<<7)
      #define PCI_MBOX1_SEND_INTSTATUS_BIT (1<<3)
      #define PCI_MBOX0_SEND_INTSTATUS_BIT (1<<2)
      #define PCI_MBOX1_RCV_INTSTATUS_BIT  (1<<1)
      #define PCI_MBOX0_RCV_INTSTATUS_BIT  (1<<0)
    uint8   unused2[0x70-0x54];     // 54
    uint32  MailBox0;               // 70
    uint32  MailBox1;               // 74
    uint32  L2PCfgCntrl;            // 78
    uint32  L2PCfgData;             // 7c
    LOCDmaConfig LocDmaConfig[2]; // 80, A0
        #define MPI_DMA_LOC_PCI2UBUS 0
        #define MPI_DMA_LOC_UBUS2PCI 1
    PCIDmaConfig PciDmaConfig[2];   // c0, d8
        #define MPI_DMA_PCI_UBUS2PCI 0
        #define MPI_DMA_PCI_PCI2UBUS 1
    uint32  VendorId;               // f0
    uint32  RevisionId;             // f4
} MpiRegisters;

//#define MPIC ((volatile MpiRegisters * const) MPIC_BASE)



// When the switch is attached to the SPI port, this is the device ID.
#define SPI_SWITCH_CHANNEL        0x3
// Set this to 1 if the switch could be attached to GPIOs instead of SPI port.
#define SPI_SWITCH_USES_GPIO      0
#define MSPI_SPI_SS1              0x00004000
#define MSPI_SPI_SS2              0x00000800
#define MSPI_SPI_SS3              0x00001000
#define MSPI_SPI_SS_MASK          0x00005800
#define BCM_MSPI_TX_QUEUE_SIZE           544
#define BCM_MSPI_RX_QUEUE_SIZE           544
#define BCM_SPI_LEN_BYTES               2
#define BCM_MSPI_CTRL_MODE_SELECTION     BCM_MSPI_CTRL_MODE_INTERRUPT





typedef struct Timer
{
    uint16  unused0;    // 00
    byte    TimerMask;  // 02
    byte    TimerInts;  // 03
        #define  TIMER0     0x01
        #define  TIMER1     0x02
        #define  TIMER2     0x04
        #define  WATCHDOG   0x08
        #define  TIMER0EN   0x01
        #define  TIMER1EN   0x02
        #define  TIMER2EN   0x04
    uint32  TimerCtl0;  // 04
    uint32  TimerCtl1;  // 08
    uint32  TimerCtl2;  // 0c
        #define  TIMERENABLE (1<<31)
        #define  RSTCNTCLR   (1<<30)
    uint32  TimerCnt0;  // 10
    uint32  TimerCnt1;  // 14
    uint32  TimerCnt2;  // 18

    uint32  WatchDogDefCount; // 1c

    // Write 0xff00 0x00ff to Start timer
    // Write 0xee00 0x00ee to Stop and re-load default count
    // Read from this register returns current watch dog count
    uint32  WatchDogCtl;

    // Number of ticks for WD Reset pulse to last
    uint32  WDResetCount;
} Timer;

#define TIMER ((volatile Timer * const) TIMR_BASE)



typedef struct Uart
{
    byte    unused0;
    byte    control;
        #define BRGEN           (1<<7)
        #define TXEN            (1<<6)
        #define RXEN            (1<<5)
        #define LOOPBK          (1<<4)
        #define TXPARITYEN      (1<<3)
        #define TXPARITYEVEN    (1<<2)
        #define RXPARITYEN      (1<<1)
        #define RXPARITYEVEN    (1<<0)

    byte    config;
        #define XMITBREAK   0x40
        #define BITS5SYM    0x00
        #define BITS6SYM    0x10
        #define BITS7SYM    0x20
        #define BITS8SYM    0x30
        // 4 low bits are STOP bits/char in 1/8 bit-time intervals.  Zero
        // represents 1/8 stop bit interval.  Fifteen represents 2 stop bits.
        #define ONESTOP     0x07
        #define TWOSTOP     0x0f

    byte    fifoctl;
        #define  RSTTXFIFOS  0x80
        #define  RSTRXFIFOS  0x40
        #define  RSTTXDN     0x20
        // 5-bit TimeoutCnt is in low bits of this register.
        //  This count represents the number of character
        //  idle times before setting receive Irq when below threshold

    // When we divide SysClk/2/(1+baudword) we should get 32*bit-rate
    uint32  baudword;

    // Read-only fifo depth
    byte    txf_levl;
    byte    rxf_levl;

    // Upper 4-bits are TxThresh, Lower are RxThresh.  Irq can be asserted
    //   when rxf_level > RxThresh and/or txf_level < TxThresh
    byte    fifocfg;

    // Set value of DTR & RTS if bits are enabled to GPIO_o
    byte    prog_out;
        #define UART_DTR_OUT    0x01
        #define UART_RTS_OUT    0x02

    byte    unused1;

    // Low 4-bits, set corr bit to 1 to detect irq on rising AND falling
    // edges for corresponding GPIO_if enabled (edge insensitive)
    byte    DeltaIPEdgeNoSense;

    // Upper 4 bits: 1 for posedge sense, 0 for negedge sense if
    //   not configured for edge insensitive (see above)
    // Lower 4 bits: Mask to enable change detection IRQ for corresponding
    //  GPIO_i
    byte    DeltaIPConfig_Mask;

    // Upper 4 bits show which bits have changed (may set IRQ).
    //  read automatically clears bit
    // Lower 4 bits are actual status
    byte    DeltaIP_SyncIP;


    uint16  intStatusMask;
    uint16  intStatus;
        #define  TXCHARDONE  (1<<15)
        #define  RXBRK       (1<<14)
        #define  RXPARERR    (1<<13)
        #define  RXFRAMERR   (1<<12)
        #define  RXFIFONE    (1<<11)
        #define  RXFIFOTHOLD (1<<10)
        #define  RXFIFOFULL  (1<< 9)
        #define  RXTIMEOUT   (1<< 8)
        #define  RXOVFERR    (1<< 7)
        #define  RXUNDERR    (1<< 6)
        #define  TXFIFOEMT   (1<< 5)
        #define  TXREADLATCH (1<< 4)
        #define  TXFIFOTHOLD (1<< 3)
        #define  TXOVFERR    (1<< 2)
        #define  TXUNDERR    (1<< 1)
        #define  DELTAIP     (1<< 0)

    uint16  unused2;

    // Write to TX, Read from RX.  Bits 11:8 are BRK,PAR,FRM errors
    uint16  Data;

} Uart;

//#define UART ((Uart * const) UART_BASE)
#define UART0 ((volatile Uart * const) UART_BASE)



// Each DMA channel in a particular core has this set of registers
// to configure and control the DMA channel behavior.
typedef struct IuDmaCfgRegisters
{
    uint32  cfg;           // (00) assorted configuration
        #define DMA_ENABLE      0x01
        #define DMA_PKT_HALT    0x02
        #define DMA_BURST_HALT  0x04
    uint32  intStat  ;        // (04) interrupts control and status
    uint32  intMask;          // (08) interrupts mask
        #define DMA_BUFF_DONE   0x01
        #define DMA_DONE        0x02
        #define DMA_NO_DESC     0x04
    uint32  maxBurst;         // (0c) max burst length permitted

} IuDmaCfgRegisters;

// Each DMA channel in a particular core has this set of registers
// to assign and track memory states and resources. Since this block
// is not contiguous with the above block in every core, they are
// broken up into to distinct register blocks.
typedef struct IuDmaStateRamRegisters
{
    uint32  DescriptorBase;   // (100)
    uint32  StateRam2;        // (104)
    uint32  StateRam3;        // (108)
    uint32  StateRam4;        // (10c)

} IuDmaStateRamRegisters;

// DMA buffer descriptor
typedef struct DmaDesc
{
    uint16  length;     // RX - buffer size, TX - # bytes in buffer (bits 12:00 valid for length)
        #define DMA_USE_FREEPOOL (1<<15)  // Use Free Pool Manager to allocate (Rx) or deallocate (Tx) buffer

    uint16  status;     // buffer status
        #define DMA_OWN         (1<<15)  // cleared by DMA, set by SW
        #define DMA_EOP         (1<<14)  // last buffer in packet
        #define DMA_SOP         (1<<13)  // first buffer in packet
        #define DMA_WRAP        (1<<12)  // end of buffer ring
        #define	DMA_APPEND_CRC  (1<< 8)  // Tx append CRC
        // EMAC Descriptor Status definitions
        #define EMAC_UNDERRUN   (1<< 9)  // Tx underrun
        #define EMAC_MISS       (1<< 7)  // framed address recognition failed (promiscuous)
        #define EMAC_BRDCAST    (1<< 6)  // DA is Broadcast
        #define EMAC_MULT       (1<< 5)  // DA is multicast
        #define EMAC_LG         (1<< 4)  // frame length > RX_LENGTH register value
        #define EMAC_NO         (1<< 3)  // Non-Octet aligned
        #define EMAC_RXER       (1<< 2)  // RX_ERR on MII while RX_DV assereted
        #define EMAC_CRC_ERROR  (1<< 1)  // CRC error
        #define EMAC_OV         (1<< 0)  // Overflow
        #define USB_ZERO_PKT    (1<< 0) // Set to send zero length packet

    uint32  address;    // address of data
} DmaDesc;
#define  DMA_DESC_LENGTH   0
#define  DMA_DESC_STATUS   2
#define  DMA_DESC_ADDR     4

typedef struct UsbRegisters
{
   uint32 control;		// USB Core Configuration
       #define USBD_CONTROL_APP_RESUME     0x0002
   uint32 straps;		// USB Core strapping options
       #define USBD_STRAPS_APP_SELF_PWR    0x0400
       #define USBD_STRAPS_APP_DEV_DISCON  0x0200
       #define USBD_STRAPS_APP_RAM_IF      0x0080
       #define USBD_STRAPS_APP_DEV_RMTWKUP 0x0040
       #define USBD_STRAPS_APP_PHYIF_8BIT  0x0004
       #define USBD_STRAPS_FULL_SPEED      0x0003
       #define USBD_STRAPS_LOW_SPEED	   0x0002
   uint32 stall;
   uint32 status;
       #define USBD_ENUM_SPEED    0x3000
   uint32 events;
       #define USBD_SET_CSRS      0x40
       #define USBD_SUSPEND       0x20
       #define USBD_EARLY_SUSPEND 0x10
       #define USBD_SOF           0x08
       #define USBD_ENUMON        0x04
       #define USBD_SETUP         0x02
       #define USBD_USBRESET      0x01
   uint32 events_irq;
   uint32 events_irq_mask;
   uint32 reserved1;
   uint32 txfifo_rw_ptr;
   uint32 rxfifo_rw_ptr;
   uint32 txfifo_stat_rw_ptr;
   uint32 rxfifo_stat_rw_ptr;
   uint32 reserved2[0x04];
   uint32 txfifo_address_config[5];
   uint32 reserved3[0x0B];
   uint32 rxfifo_address_config[5];
   uint32 reserved4[0x0B];
   uint32 bert_control1;
   uint32 bert_control2;
   uint32 phy_control3;
       #define USBD_PHY_CNTL3_UTMI_PWDNB (1<<14)
       #define USBD_PHY_CNTL3_PHY_PWDNB  (1<<13)
   uint32 phy_control4;
       #define USBD_PHY_CNTL4_PLL_BYP             (1<<13)
       #define USBD_PHY_CNTL4_REFCLKSEL_SNGL_CLK  0x18
       #define USBD_PHY_CNTL4_REFCLKSEL_DIFF_CLK  0x10
       #define USBD_PHY_CNTL4_REFCLKSEL_XTAL_OSC  0x08
       #define USBD_PHY_CNTL4_XTAL_PWRDWN         0x04
       #define USBD_PHY_CNTL4_CLKSEL              0x02
       #define USBD_PHY_CNTL4_PHY_PWRDWN          0x01
   uint32 reserved5[0x04];
} UsbRegisters;

typedef struct UsbIudmaChanConfig {
   UINT32 config;
       #define USBD_IUDMA_CHAN_EN_DMA 0x01
   UINT32 irq_stat;
   UINT32 irq_mask;
   	   #define USBD_IUDMA_CHAN_INT_BDONE  0x01
	   #define USBD_IUDMA_CHAN_INT_PDONE  0x02
   	   #define USBD_IUDMA_CHAN_INT_NOTVLD 0x04
   UINT32 maxburst;
} UsbIudmaChanConfig;

#define USB20_CTRL_RX_CHAN 0
#define USB20_CTRL_TX_CHAN 1
#define USB20_BULK_RX_CHAN 2
#define USB20_BULK_TX_CHAN 3
#define USB20_INTR_RX_CHAN 4
#define USB20_INTR_TX_CHAN 5

typedef struct UsbIudmaCtrlRegisters {
   UINT32 iudma_ctrl_config;
       #define USBD_IUDMA_EN 0x01
   UINT32 reserved1[0x3F];
   UsbIudmaChanConfig usb_iudma_chan_config[6];
   UINT32 reserved2[0x28];
   IuDmaStateRamRegisters iudma_state_ram[6];
} UsbIudmaCtrlRegisters;

//The emac/usb core contains 3 macs and 8 dma channels. This structure
//defines a set of "master control" registers that control and configure
//the overall dma block itself. Each channel has it's own sub-set of registers
//for configuration and control.
typedef struct EmacUsbBlockDmaRegisters
{
   uint32 EmacUsbDmaControl;
      #define IUDMA_EN              0x01
      #define IUDMA_FLOWCONTROL_CH1 0x02
      #define IUDMA_FLOWCONTROL_CH3 0x04
   uint32 EmacUsbDmaChan1FlowControlLo;
   uint32 EmacUsbDmaChan1FlowControlHi;
   uint32 EmacUsbDmaChan1BufferAlloc;
   uint32 EmacUsbDmaChan3FlowControlLo;
   uint32 EmacUsbDmaChan3FlowControlHi;
   uint32 EmacUsbDmaChan3BufferAlloc;
   uint32 EmacUsbDmaChannelReset;
   uint32 EmacUsbDmaChannelDebug;

} EmacUsbBlockDmaRegisters;

//This structure defines the highest level hierarchy
// Combination EMAC1/EMAC2/USB and associated DMA channels.
typedef struct EmacUsbBlockRegisters
{

   //Register block for the #1 ethernet mac core.
	//struct EmacRegisters             EmacCore1Registers;        //0xfff98000
	//uint8  Pad1[((0x8800 - 0x8000) - (sizeof(EmacRegisters)))];

   //Register block for the master control of the emac/usb core dma engine.
   struct EmacUsbBlockDmaRegisters  EmacUsbDmaCtrlRegisters;   //0xfff98800
   uint8  Pad4[((0x8900 - 0x8800) - (sizeof(EmacUsbBlockDmaRegisters)))];

   //Control register blocks for each individual DMA channel.
   struct IuDmaCfgRegisters         IuDmaConfigRegisters[2];   //0xfff98900
   uint8  Pad5[((0x8a00 - 0x8900) - (sizeof(IuDmaCfgRegisters)*2))];

   //RAM state register blocks for each individual DMA channel.
   struct IuDmaStateRamRegisters    IuDmaStateRegisters[2];    //0xfff98a00

} EmacUsbBlockRegisters;
#define EMUSB ((volatile EmacUsbBlockRegisters * const) EMUSB_BASE)
#define USB_MBDMA_0 ((volatile usb_mbdma_regs * const) USB_OTG_BLOCK_0)
#define USB_OTG_0 ((volatile UsbOtgControllerRegs * const) USB_OTG_CTRLR0_BASE)
#define USB_CORE_0 ((volatile UsbOtgRegs * const) USB_OTG_CORE0_BASE)

#define USB_MBDMA_1 ((volatile usb_mbdma_regs * const) USB_OTG_BLOCK_1)
#define USB_OTG_1 ((volatile UsbOtgControllerRegs * const) USB_OTG_CTRLR1_BASE)
#define USB_CORE_1 ((volatile UsbOtgRegs * const) USB_OTG_CORE1_BASE)

#define USB_HOST ((volatile UsbOtgControllerRegs * const) USB_OTG_BLOCK_0)

// ---- spi master registers ----
// Most of these registers are the same as used in other 33xx chips.  The
// names are changed so whoever implements a SPI driver won't be able to
// compile the existing driver without chamges.
typedef struct SpiMasterRegisters
{
    uint8  txdata[544];            // 00:
        #define SPIM_FULLDUPLEXRW   (0<<14)
        #define SPIM_HALFDUPLEXWR   (1<<14)
        #define SPIM_HALFDUPLEXRD   (2<<14)
        #define SPIM_RESERVED       (3<<14)

    uint8   unused1[0x400-0x220];   // 0x220 = 136*4

    uint8  rxdata[544];            // 0x400

    uint8   unused2[0x700-0x620];   // 0x620

    uint16  command;                // 0x700
        #define SPIM_ONEWIRE            (  1<<12)
        #define SPIM_ONEBYTE            (  1<<11)
        #define SPIM_PREPEND_BYTE_CNT   (  7<< 8)
        #define SPIM_DEVICE_ID          (  7<< 4)
        #define SPIM_CMND               (0xf<< 0)

    uint8   intstatus;              // 0x702
    uint8   mintstatus;             // 0x703

    uint8   intmask;                // 0x704
        // These are use for the 3 preceding fields.
        #define SPIM_RX_UNDERRUN        0x10
        #define SPIM_TX_OVERRUN         0x08
        #define SPIM_TX_UNDERRUN        0x04
        #define SPIM_RX_OVERRUN         0x02
        #define SPIM_CMND_DONE          0x01

    uint8   status;                 // 0x705
        #define SPIM_SERIAL_BUSY        0x08
        #define SPIM_CMND_BUSY          0x04
        #define SPIM_RX_EMPTY           0x02

    uint8   clkcfg;                // 0x706
        #define SPIM_BYTE_SWAP          0x80
        #define SPIM_OFF_TIME           0x38
        #define SPIM_CLK_DIV            0x07

    uint8   fillbyte;               // 0x707

    uint16  msgtail;                // 0x708
    uint16  rxtail;                 // 0x70a
} SpiMasterRegisters;

#define SPI ((volatile SpiMasterRegisters * const) SPIM_BASE)


// ==== DIAG16 output definitions ====

// ---- periph_tbus:BistVect ----
#define DIAG16_SPI_BIST_DONE            0x0001
#define DIAG16_SPI_BIST_FAIL            0x01fe
#define   DIAG16_SPI_TX_BIST_FAIL         0x001e
#define     DIAG16_SPI_TX0_BIST_FAIL        0x0002
#define     DIAG16_SPI_TX1_BIST_FAIL        0x0004
#define     DIAG16_SPI_TX2_BIST_FAIL        0x0008
#define     DIAG16_SPI_TX3_BIST_FAIL        0x0010
#define   DIAG16_SPI_RX_BIST_FAIL         0x01e0
#define     DIAG16_SPI_RX0_BIST_FAIL        0x0020
#define     DIAG16_SPI_RX1_BIST_FAIL        0x0040
#define     DIAG16_SPI_RX2_BIST_FAIL        0x0080
#define     DIAG16_SPI_RX3_BIST_FAIL        0x0100
#define DIAG16_UART_RX_BIST_DONE        0x0200
#define DIAG16_UART_RX_BIST_FAIL        0x0400
#define DIAG16_UART_TX_BIST_DONE        0x0800
#define DIAG16_UART_TX_BIST_FAIL        0x1000

// ---- emac tbus ----
#define DIAG16_EMAC_MIB_BIST_FAIL       0x0200
#define DIAG16_EMAC_MIB_BIST_DONE       0x0100
#define DIAG16_EMAC_RXSLA_BIST_FAIL     0x0080
#define DIAG16_EMAC_RXSLA_BIST_DONE     0x0040
#define DIAG16_EMAC_RXSLB_BIST_FAIL     0x0020
#define DIAG16_EMAC_RXSLB_BIST_DONE     0x0010
#define DIAG16_EMAC_RXD_BIST_FAIL       0x0008
#define DIAG16_EMAC_RXD_BIST_DONE       0x0004
#define DIAG16_EMAC_TX_BIST_FAIL        0x0002
#define DIAG16_EMAC_TX_BIST_DONE        0x0001

// ---- usb tbus ----
#define DIAG16_USB_CF_BIST_DONE         0x0020
#define DIAG16_USB_CF_BIST_FAIL         0x0010
#define DIAG16_USB_TX_BIST_DONE         0x0008
#define DIAG16_USB_TX_BIST_FAIL         0x0004
#define DIAG16_USB_RX_BIST_DONE         0x0002
#define DIAG16_USB_RX_BIST_FAIL         0x0001

// ---- ebi tbus ----
#define DIAG16_EBI_RX_BIST_FAIL         0x0080
#define DIAG16_EBI_RX_BIST_DONE         0x0040
#define DIAG16_EBI_TX_BIST_FAIL         0x0020
#define DIAG16_EBI_TX_BIST_DONE         0x0010
#define DIAG16_EBI_DATA_BIST_FAIL       0x0008
#define DIAG16_EBI_DATA_BIST_DONE       0x0004
#define DIAG16_EBI_ATTR_BIST_FAIL       0x0002
#define DIAG16_EBI_ATTR_BIST_DONE       0x0001

// ---- mips tbus ----
#define DIAG16_MIPS_DDATA_BIST_DONE     0x0001
#define DIAG16_MIPS_DDATA_BIST_DBG      0x0002
#define DIAG16_MIPS_DDATA_BIST_FAIL     0x0004
#define DIAG16_MIPS_DTAG_BIST_DONE      0x0008
#define DIAG16_MIPS_DTAG_BIST_DBG       0x0010
#define DIAG16_MIPS_DTAG_BIST_FAIL      0x0020
#define DIAG16_MIPS_IDATA_BIST_DONE     0x0040
#define DIAG16_MIPS_IDATA_BIST_DBG      0x0080
#define DIAG16_MIPS_IDATA_BIST_FAIL     0x0100
#define DIAG16_MIPS_ITAG_BIST_DONE      0x0200
#define DIAG16_MIPS_ITAG_BIST_DBG       0x0400
#define DIAG16_MIPS_ITAG_BIST_FAIL      0x0800
#define DIAG16_MIPS_TLB_BIST_DONE       0x1000
#define DIAG16_MIPS_TLB_BIST_DBG        0x2000
#define DIAG16_MIPS_TLB_BIST_FAIL       0x4000


/************************************/


#define CacheFlush      0x01        // flushes the cache (ie: does writeback)
#define CacheInvalidate 0x02        // invalidates cache entries
#define DCACHE_LINE_SIZE 16         // line size for dcache

/* -------------------------------------------------------------
     Name: ClearDcacheByAddress
 Summary: Clears data stored in the cache of the given
          address range. Data may be flushed and/or invalidated
          depending on the given mask.
------------------------------------------------------------- */
extern void
ClearDcacheByAddress(
                    uint32  wMask,          // clear mask
                    uint32  wBase,          // starting address
                    uint32  wLen);          // length in bytes


/* -------------------------------------------------------------
     Name: ClearDcacheByIndex
 Summary: Clears the Dcache by index, starting from the
          index of the base address, up to the index of
          base+length. Valid dirty cache contents will be
          committed if we are in writeback mode, else
          the index is simply invalidated.
------------------------------------------------------------- */
extern void
ClearDcacheByIndex(
                  uint32  wBase,          // starting address
                  uint32  wLen);          // length in bytes

#ifdef __cplusplus
}
#endif

#endif
