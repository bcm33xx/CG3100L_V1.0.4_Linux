//****************************************************************************
//
// (c) 2008 Broadcom Corporation
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
//  Filename:       io_proc.h
//  Author:         <author>
//  Creation Date:  <date>
//
//****************************************************************************

#ifndef io_proc_h
#define io_proc_h

/**Currently: addresses below follow byte addressing **/
#ifndef _ASM_COMPILE_
#ifndef BCMTYPES_H
#include    "bcmtypes.h"
#endif
#endif

//3380 Specfic Defines for the various processor types.
#define UTP       0
#define DTP       1
#define FAP       2
#define MSG_PROC  3
#define MPEG_PROC 4

//Define the total number of IO processors available in the chip.
#define NUM_DEFINED_IO_PROCS 5
#define NUM_DQM_QUEUES_PER_IOPROC 32

//IDs for abstracted wait command.
#define IOWAIT_NS 0
#define IOWAIT_MS 1

//Abstracted print function.
#ifndef USE_SCV
#define ioproc_print printf
#else
#define ioproc_print print_log
#endif

#define SDRAM_LO_BASE        0x00000000
#define SDRAM_LO_SIZE        (1<<15)         //32KB
#define SDRAM_LO_BASE_END    (SDRAM_LO_BASE+(sizeof(SDRAM_Lo_S))-1)
#define SDRAM_LO             ((volatile SDRAM_Lo_S *)SDRAM_LO_BASE)
#define SDRAM_LO_UCV         ((SDRAM_Lo_S *)(SDRAM_LO_BASE + 0x80000000))
#if !defined(__KERNEL__)
#ifndef FPGA_CMAPP_BUILD
#define FLASH_BASE           0x1fc00000
#endif
#define FLASH_SIZE           (1<<20)         //1MB
#define FLASH_BASE_END       (FLASH_BASE+(sizeof(Flash_S))-1)
#define FLASH                ((volatile Flash_S *)FLASH_BASE)
#define FLASH_UCV            ((volatile Flash_S *)(FLASH_BASE + 0x80000000))
#endif
#define SDRAM_HI_BASE        0x00002000
#define SDRAM_HI_SIZE        (1<<12)         //4KB
#define SDRAM_HI_BASE_END    (SDRAM_HI_BASE+(sizeof(SDRAM_Hi_S))-1)
#define SDRAM_HI             ((volatile SDRAM_Hi_S *)SDRAM_HI_BASE)
#define SDRAM_HI_UCV         ((volatile SDRAM_Hi_S *)(SDRAM_HI_BASE + 0x80000000))

#define DSPRAM_BASE          0x00002000
#define DSPRAM_VBASE         (DSPRAM_BASE | 0x80000000)
#define DSPRAM_SIZE          (1<<12)         //4KB


#define DSPRAM_IO_BASE       0x80003000
#define DSPRAM_IO_SIZE       256 

#define DspramReg            ((volatile DSPRAM_IO_Regs_S *)DSPRAM_IO_BASE)

//#ifndef FPGA_CMAPP_BUILD
#define UART4KE_BASE            0xe0000000
//#endif
#define UART4KE_BASE_END        (UART4KE_BASE+sizeof(UartRegs_S)-1)
#define UART4KE                 ((volatile UartRegs_S *)UART4KE_BASE)

#define UART8051_BASE_END        (UART8051_BASE+sizeof(Uart8051Regs_S)-1)
#define UART8051                 ((volatile Uart8051Regs_S *)UART8051_BASE)

//#define UBUS_PERIPH_BASE     SDRAM_LO_BASE        
//#define UBUS_PERIPH_END      SDRAM_LO_BASE_END        
#define UBUS_PERIPH_BASE     SDRAM_LO_BASE        //FLASH_BASE        
#define UBUS_PERIPH_END      0xdfffffff        
//#define UBUS_PERIPH_END      SDRAM_HI_BASE_END        
//#define UBUS_PERIPH_BASE     0xe0005000        
//#define UBUS_PERIPH_END      0xe0005fff        

#define CntrlReg             ((volatile CoprocCtlRegs_S *)CTRL_REG_BLOCK_BASE)
#define OgMsgReg             ((volatile OGMsgFifoRegs_S *)OG_MSG_BASE)
#define InMsgReg             ((volatile INMsgFifoRegs_S *)IN_MSG_BASE)
#define MsgIdReg             ((volatile MsgIdRegs_S *)MSG_ID_BASE)
#define DmaReg               ((volatile DMARegs_S *)DMA_BASE)
#define TknIntfReg           ((volatile TknIntfRegs_S *)TKN_INTF_BASE)
#define DqmReg               ((volatile DQMRegs_S *)DQM_BASE)
#define DqmQCntrlReg         ((volatile DQMQCntrlRegs_S *)DQMQCNTRL_BASE)
#define DqmQDataReg          ((volatile DQMQDataRegs_S *)DQMQDATA_BASE)
#define PmReg                ((volatile PMRegs_S *)PM_BASE)
#define SharedMemPtr         ((volatile uint32 *)SHARED_MEM_BASE)

// PCI to UBUS conversion
#define UBUS_IO_PROC_BASE     0x0c000000
#define UBUS_DECODE_MASK      0x00ffffff
#define pCntrlReg             ((CoprocCtlRegs_S *)(CTRL_REG_BLOCK_BASE & UBUS_DECODE_MASK | UBUS_IO_PROC_BASE))
#define pOgMsgReg             ((OGMsgFifoRegs_S *)(OG_MSG_BASE         & UBUS_DECODE_MASK | UBUS_IO_PROC_BASE))
#define pInMsgReg             ((INMsgFifoRegs_S *)(IN_MSG_BASE         & UBUS_DECODE_MASK | UBUS_IO_PROC_BASE))
#define pMsgIdReg             ((MsgIdRegs_S *)    (MSG_ID_BASE         & UBUS_DECODE_MASK | UBUS_IO_PROC_BASE))
#define pDmaReg               ((DMARegs_S *)      (DMA_BASE            & UBUS_DECODE_MASK | UBUS_IO_PROC_BASE))
#define pTknIntfReg           ((TknIntfRegs_S *)  (TKN_INTF_BASE       & UBUS_DECODE_MASK | UBUS_IO_PROC_BASE))
#define pDqmReg               ((DQMRegs_S *)      (DQM_BASE            & UBUS_DECODE_MASK | UBUS_IO_PROC_BASE))
#define pDqmQCntrlReg         ((DQMQCntrlRegs_S *)(DQMQCNTRL_BASE      & UBUS_DECODE_MASK | UBUS_IO_PROC_BASE))
#define pDqmQDataReg          ((DQMQDataRegs_S *) (DQMQDATA_BASE       & UBUS_DECODE_MASK | UBUS_IO_PROC_BASE))
#define pPmReg                ((PMRegs_S *)       (PM_BASE             & UBUS_DECODE_MASK | UBUS_IO_PROC_BASE))
#define pSharedMemPtr         ((unsigned int *)   (SHARED_MEM_BASE     & UBUS_DECODE_MASK | UBUS_IO_PROC_BASE))

//Base Addresses
#define IO_PROC_BASE        0xe0001000

#define CTRL_REG_BLOCK_BASE (IO_PROC_BASE + 0x0000) 
#define OG_MSG_BASE         (IO_PROC_BASE + 0x0100) 
#define IN_MSG_BASE         (IO_PROC_BASE + 0x0200) 
#define DMA_BASE            (IO_PROC_BASE + 0x0300)
#define TKN_INTF_BASE       (IO_PROC_BASE + 0x0400)
#define UART8051_BASE       (IO_PROC_BASE + 0x0500)
#define PM_BASE             (IO_PROC_BASE + 0x0600)   
#define MSG_ID_BASE         (IO_PROC_BASE + 0x0700)
#define DQM_BASE            (IO_PROC_BASE + 0x0800)
#define DQMQCNTRL_BASE      (IO_PROC_BASE + 0x0a00)
#define DQMQDATA_BASE       (IO_PROC_BASE + 0x0c00)
#define SHARED_MEM_BASE     0xe0004000
// DQM start address in shared mem (in words)
#define DQM_BASE_ADDR       (SHARED_MEM_BASE >> 2) 



#define CTRL_REG_BLOCK_END  (CTRL_REG_BLOCK_BASE + (sizeof(CoprocCtlRegs_S)-1))
#define OG_MSG_END          (OG_MSG_BASE         + (sizeof(OGMsgFifoRegs_S)-1)) 
#define MSG_ID_END          (MSG_ID_BASE         + (sizeof(MsgIdRegs_S)-1)) 
#define IN_MSG_END          (IN_MSG_BASE         + (sizeof(INMsgFifoRegs_S)-1)) 
#define DMA_END             (DMA_BASE            + (sizeof(DMARegs_S)-1)) 
#define DQM_END             (DQM_BASE            + (sizeof(DQMAllRegs_S)-1)) 
#define PM_END              (PM_BASE             + (sizeof(PMRegs_S)-1)) 
#define TKN_INTF_END        (TKN_INTF_BASE       + (sizeof(TknIntfRegs_S)-1)) 
#define SHARED_MEM_END      (SHARED_MEM_BASE     + (0x4000 -1))     

//IRQ bit defines. Should be the same for status and mask. Also, the same
//for 4ke regs and Host regs.
#define TIMER_IRQ_BIT      0x00000001
#define OUT_FIFO_IRQ_BIT   0x00000002
#define IN_FIFO_IRQ_BIT    0x00000004
#define DQM_IRQ_BIT        0x00000008
#define UART_IRQ_BIT       0x00000040
#define UART8051_RXIRQ_BIT 0x80000000
#define UART8051_TXIRQ_BIT 0x40000000
#define GP_INPUT_IRQ_BIT   0x00000040

#define PhysToNonCacheVirtAddr(x)   ((x&0x1fffffff)|0xa0000000)
#define PhysToCacheVirtAddr(x)      ((x&0x1fffffff)|0x80000000)
#define VirtToPhysAddr(x)           (x&0x1fffffff)

#ifndef _ASM_COMPILE_

typedef struct SDRAM_Lo_S
{
   unsigned int  data[SDRAM_LO_SIZE >> 2];
} SDRAM_Lo_S;

typedef struct Flash_S
{
   unsigned int  data[FLASH_SIZE >> 2];
} Flash_S;

typedef struct SDRAM_Hi_S
{
   unsigned int  data[SDRAM_HI_SIZE >> 2];
} SDRAM_Hi_S;

typedef struct UartRegs_S
{
   unsigned int  txDataReg;
   unsigned int  interruptEnableReg;   //Mask Register - set bits to enable.
   unsigned int  interruptStatusReg;   //Status - set bits to clear.
   unsigned int  dumpTimeStamp;        //This is to display clock time for debugging.
   unsigned int  dumpHexValue;         //This is to display hex number
   unsigned int  dumpDecValue;         //This is to display dec number
   unsigned int  dumpPredefinedStr[50];//This is to display pre-defined string
} UartRegs_S;

typedef struct Uart8051Regs_S
{
   unsigned int  scon_reg;
#define RI_bit 0x00000001
#define TI_bit 0x00000002
#define RB8_bit 0x00000004
#define TB8_bit 0x00000008
#define REN_bit 0x00000010
#define SM2_bit 0x00000020
#define SM1_bit 0x00000040
#define SM0_bit 0x00000080
   unsigned int  rxbuf_reg;            
   unsigned int  txbuf_reg;        
   unsigned int  uarttimer_reg;
#define UART_TIMER_ENABLE 0x80000000
   unsigned int  dumpTimeStamp;        //This is to display clock time for debugging.
} Uart8051Regs_S;

//#if __SYSTEMC__
#if 0

#include <systemc.h>
//Control Block
typedef struct CoprocCtlRegs_SYSC_S
{
  sc_signal<uint32>    irq_4ke_mask;
            #define CNTRL_IRQ_UART_RX_MASK             0x80000000
            #define CNTRL_IRQ_UART_TX_MASK             0x40000000
            #define CNTRL_IRQ_GP_INPUT_MASK            0x00000040
            #define CNTRL_IRQ_MBOX_OUT_MASK            0x00000020
            #define CNTRL_IRQ_MBOX_IN_MASK             0x00000010
            #define CNTRL_IRQ_DQM_MASK                 0x00000008
            #define CNTRL_IRQ_IN_FIFO_MASK             0x00000004
            #define CNTRL_IRQ_OUT_FIFO_MASK            0x00000002
            #define CNTRL_IRQ_TIMER_IRQ_MASK           0x00000001
  sc_signal<uint32>    irq_4ke_status;
  sc_signal<uint32>    irq_mips_mask;
            #define   MIPS_MBOX_OUT_IRQ_MASK           0x00000020
            #define   MIPS_MBOX_IN_IRQ_MASK            0x00000010
  sc_signal<uint32>    irq_mips_status;
  sc_signal<uint32>    gp_mask;
            #define   GP_TMR_MASK_FIELD_MASK           0x0000FFFF
            #define   MBOX_OUT_IRQ_MASK                0x00000008
            #define   MBOX_IN_IRQ_MASK                 0x00000004
            #define   TIMER1_IRQ_MASK                  0x00000002
            #define   TIMER0_IRQ_MASK                  0x00000001
  uint32              gp_status;
            #define   GP_TMR_STATUS_FIELD_MASK         0x0000FFFF
            #define   MBOX_OUT_IRQ_STATUS              0x00000008
            #define   MBOX_IN_IRQ_STATUS               0x00000004
            #define   TIMER1_IRQ_STATUS                0x00000002
            #define   TIMER0_IRQ_STATUS                0x00000001
  sc_signal<uint32>    gp_tmr0_ctl;
            #define   GP_TIMER_ENABLE                  0x80000000
            #define   TIMER_MODE_SINGLE                0x00000000
            #define   TIMER_MODE_REPEAT                0x40000000
            #define   TIMER_COUNT_MASK                 0x3fffffff
  uint32               gp_tmr0_cnt;
  sc_signal<uint32>    gp_tmr1_ctl;
  uint32               gp_tmr1_cnt;
  sc_signal<uint32>    host_mbox_in;
  sc_signal<uint32>    host_mbox_out;
  sc_signal<uint32>    gp_out;
  sc_signal<uint32>    gp_in;
            #define GP_IN_TAM_IRQ_MASK                 0x00008000
            #define GP_IN_SEGDMA_IRQ_MASK              0x00000002
            #define GP_IN_USPP_BUSY_FLAG               0x00000001
  sc_signal<uint32>    gp_in_irq_mask;
            #define GP_IN_IRQ_SENSE_MASK               0xFFFF0000
            #define GP_IN_IRQ_SENSE_SHIFT              16
            #define GP_IN_IRQ_MASK_MASK                0x0000FFFF
            #define GP_IN_IRQ_MASK_SHIFT               0
  sc_signal<uint32>    gp_in_irq_status;
            #define GP_IN_IRQ_STATUS_MASK              0x0000FFFF
            #define GP_IN_IRQ_STATUS_SHIFT             0
  uint32    dma_control;   // 00
  uint32    dma_status;   // 00
            #define DMA_STATUS_DMAi_BUSY               0x00000001
            #define DMA_STATUS_DMAi_CMD_FULL_BIT       0x00000002
            #define DMA_STATUS_DMAi_RSLT_EMPTY_BIT     0x00000004
            #define DMA_STATUS_DMAi_RSLT_FULL_BIT      0x00000008
  uint32    dma0_3_fifo_status;   // 04
  uint32    dma4_7_fifo_status;   // 08
            #define DMA_FIFO_STS_DMAi_CMD_ROOM_MSK     0x0000000F
            #define DMA_FIFO_STS_DMAi_RSLT_DEPTH_MSK   0x000000F0
            #define DMA_FIFO_STS_DMAi_RSLT_DEPTH_SHIFT 4
  uint32    dma_irq_sts;   // 00
  uint32    dma_4ke_irq_mask;   // 00
  uint32    dma_host_irq_mask;   // 00
  
} CoprocCtlRegs_SYSC_S;

#endif

//Control Block
typedef struct CoprocCtlRegs_S
{
  uint32    irq_4ke_mask;
            #define CNTRL_IRQ_UART_RX_MASK             0x80000000
            #define CNTRL_IRQ_UART_TX_MASK             0x40000000
            #define CNTRL_IRQ_E2U_TIMEOUT_MASK         0x00000080
            #define CNTRL_IRQ_GP_INPUT_MASK            0x00000040
            #define CNTRL_IRQ_MBOX_OUT_MASK            0x00000020
            #define CNTRL_IRQ_MBOX_IN_MASK             0x00000010
            #define CNTRL_IRQ_DQM_MASK                 0x00000008
            #define CNTRL_IRQ_IN_FIFO_MASK             0x00000004
            #define CNTRL_IRQ_OUT_FIFO_MASK            0x00000002
            #define CNTRL_IRQ_TIMER_IRQ_MASK           0x00000001
  uint32    irq_4ke_status;
  uint32    irq_mips_mask;
            #define   MIPS_E2U_TIMEOUT_IRQ_MASK        0x00000080
            #define   MIPS_GP_INPUT_IRQ_MASK           0x00000040
            #define   MIPS_MBOX_OUT_IRQ_MASK           0x00000020
            #define   MIPS_MBOX_IN_IRQ_MASK            0x00000010
            #define   MIPS_DQM_IRQ_MASK                0x00000008
  uint32    irq_mips_status;
  uint32    gp_mask;
            #define   GP_TMR_MASK_FIELD_MASK           0x0000FFFF
            #define   EB2UBUS_TIMEOUT_IRQ_MASK         0x00000010
            #define   MBOX_OUT_IRQ_MASK                0x00000008
            #define   MBOX_IN_IRQ_MASK                 0x00000004
            #define   TIMER1_IRQ_MASK                  0x00000002
            #define   TIMER0_IRQ_MASK                  0x00000001
  uint32    gp_status;
            #define   GP_TMR_STATUS_FIELD_MASK         0x0000FFFF
            #define   EB2UBUS_TIMEOUT_IRQ_STATUS       0x00000010
            #define   MBOX_OUT_IRQ_STATUS              0x00000008
            #define   MBOX_IN_IRQ_STATUS               0x00000004
            #define   TIMER1_IRQ_STATUS                0x00000002
            #define   TIMER0_IRQ_STATUS                0x00000001
  uint32    gp_tmr0_ctl;
            #define   TIMER_ENABLE                     0x80000000
            #define   TIMER_MODE_SINGLE                0x00000000
            #define   TIMER_MODE_REPEAT                0x40000000
            #define   TIMER_COUNT_MASK                 0x3fffffff
  uint32    gp_tmr0_cnt;
  uint32    gp_tmr1_ctl;
  uint32    gp_tmr1_cnt;
  uint32    host_mbox_in;
  uint32    host_mbox_out;
  uint32    gp_out;
  uint32    gp_in;
            #define GP_IN_TAM_IRQ_MASK                 0x80000000
            #define GP_IN_SEGDMA_IRQ_MASK              0x00000002
            #define GP_IN_USPP_BUSY_FLAG               0x00000001
  uint32    gp_in_irq_mask;
            #define GP_IN_BASE4_IRQ_MASK               0x80000000
            #define GP_IN_BASE4_IRQ_SHIFT              31
  uint32    gp_in_irq_status;
            #define GP_IN_IRQ_STATUS_MASK              0x0000FFFF
            #define GP_IN_IRQ_STATUS_SHIFT             0
  uint32    dma_control;   // 00
  uint32    dma_status;   // 00
            #define DMA_STS_DMAi_BUSY                  0x00000001
            #define DMA_STS_DMAi_CMD_FULL_BIT          0x00000002
            #define DMA_STS_DMAi_RSLT_EMPTY_BIT        0x00000004
            #define DMA_STS_DMAi_RSLT_FULL_BIT         0x00000008
  uint32    dma0_3_fifo_status;  
  uint32    dma4_7_fifo_status; 
            #define DMA_FIFO_STS_DMAi_CMD_ROOM_MSK     0x0000000F
            #define DMA_FIFO_STS_DMAi_RSLT_DEPTH_MSK   0x000000F0
            #define DMA_FIFO_STS_DMAi_RSLT_DEPTH_SHIFT 4
  uint32    dma_irq_sts;   // 00
  uint32    dma_4ke_irq_mask;   // 00
  uint32    dma_host_irq_mask;   // 00
  uint32    diag_cntrl;   // 00
  uint32    diag_hi;   // 00
  uint32    diag_lo;   // 00
  uint32    bad_address;   // 68
  uint32    addr1_mask;   // 6c
  uint32    addr1_base_in;   // 70
  uint32    addr1_base_out;   // 74
  uint32    addr2_mask;   // 00
  uint32    addr2_base_in;   // 00
  uint32    addr2_base_out;   // 00
  uint32    scratch;   // 00
  uint32    mbist_tm;   // 00
  uint32    soft_resets;   // active high
            #define SOFT_RESET_4KE                    0x00000001
            #define SOFT_RESET_BASE4                  0x00000002
            #define SOFT_RESET_DMA                    0x00000004
  uint32    eb2ubus_timeout;
            #define EB2UBUS_TIMEOUT_EN                0x80000000 
            #define EB2UBUS_TIMEOUT_MASK              0x0000FFFF
            #define EB2UBUS_TIMEOUT_SHIFT             0
  uint32    m4ke_core_status;
  uint32    gp_in_irq_sense;
  uint32    ub_slave_timeout;
            #define UB_SLAVE_TIMEOUT_EN                0x80000000 
            #define UB_SLAVE_TIMEOUT_MASK              0x0000FFFF
            #define UB_SLAVE_TIMEOUT_SHIFT             0
  uint32    diag_en;


} CoprocCtlRegs_S;

#define IOPROC_COPROCCTRL_REGS  ((volatile CoprocCtlRegs_S *)CTRL_REG_BLOCK_BASE)

typedef struct DSPRAM_IO_Regs_S
{
  uint32    word[64];
} DSPRAM_IO_Regs_S;

//Outgoing Message FIFO
typedef struct OGMsgFifoRegs_S
{
  uint32    og_msg_ctl;
  uint32    og_msg_sts;
  uint32    resv0;
  uint32    resv1;
  uint32    resv2;
  uint32    resv3;
  uint32    resv[10];
  uint32    og_msg_data;
} OGMsgFifoRegs_S;

typedef struct MsgIdRegs_S
{
  //uint32    msg_id_7_0;     
  //uint32    msg_id_15_8;  
  uint32    msg_id[64];
} MsgIdRegs_S;

// MESSAGES and TOKENS
#define MSG_ID_MASK             0xFC000000
#define MSG_ID_OFFSET           26

//Incoming Message FIFO
typedef struct INMsgFifoRegs_S
{
  uint32    in_msg_ctl;
            #define   AVAIL_FIFO_SPACE_MASK           0x0000003F
            #define   LOW_WATER_MARK_MASK             0x0000003F
            #define   MSG_RCVD_IRQ_STS_MASK           0x00001000
            #define   LOW_WTRMRK_IRQ_STS_MASK         0x00002000
            #define   ERR_IRQ_STS_MASK                0x00004000
            #define   NOT_EMPTY_IRQ_STS_MASK          0x00008000
            #define   NUM_MSG_IN_FIFO_MASK            0x001F0000
            #define   AVAIL_FIFO_SPACE_OFFSET         0
            #define   LOW_WATER_MARK_SHIFT            0
            #define   MSG_RCVD_IRQ_MSK_OFFSET         12
            #define   LOW_WTRMRK_IRQ_MSK_OFFSET       13
            #define   ERR_IRQ_STS_OFFSET              14
            #define   NOT_EMPTY_IRQ_STS_OFFSET        15
            #define   NUM_MSG_IN_FIFO_OFFSET          16
  uint32    in_msg_sts;
            #define INMSG_NOT_EMPTY_STS_BIT             0x80000000
            #define INMSG_NOT_EMPTY_STS_SHIFT           31
            #define INMSG_ERR_STS_BIT                   0x40000000
            #define INMSG_ERR_STS_SHIFT                 30
            #define INMSG_LOW_WATER_STS_BIT             0x20000000
            #define INMSG_LOW_WATER_STS_SHIFT           29
            #define INMSG_MSG_RX_STS_BIT                0x10000000
            #define INMSG_MSG_RX_STS_SHIFT              28
            #define INMSG_RESERVED1_MASK                0x0fc00000
            #define INMSG_RESERVED1_SHIFT               22
            #define INMSG_NUM_MSGS_MASK                 0x003F0000
            #define INMSG_NUM_MSGS_SHIFT                16
            #define INMSG_NOT_EMPTY_IRQ_STS_BIT         0x00008000
            #define INMSG_NOT_EMPTY_IRQ_STS_SHIFT       15
            #define INMSG_ERR_IRQ_STS_BIT               0x00004000
            #define INMSG_ERR_IRQ_STS_SHIFT             14
            #define INMSG_LOW_WATER_IRQ_STS_BIT         0x00002000
            #define INMSG_LOW_WATER_IRQ_STS_SHIFT       13
            #define INMSG_MSG_RX_IRQ_STS_BIT            0x00001000
            #define INMSG_MSG_RX_IRQ_STS_SHIFT          12
            #define INMSG_RESERVED2_MASK                0x00000fc0
            #define INMSG_RESERVED2_SHIFT               6
            #define INMSG_AVAIL_FIFO_SPACE_MASK         0x0000003f
            #define INMSG_AVAIL_FIFO_SPACE_SHIFT        0
  uint32    resv[13];
  uint32    in_msg_last;
  uint32    in_msg_data;
} INMsgFifoRegs_S;




//DQM
typedef struct DQMCtlRegs_S
{
  uint32        cfg;                        // 00
  uint32        _4KE_low_wtmk_irq_msk;      // 04
  uint32        mips_low_wtmk_irq_msk;      // 08
  uint32        low_wtmk_irq_sts;           // 0c
  uint32        _4KE_not_empty_irq_msk;     // 10
  uint32        mips_not_empty_irq_msk;     // 14
  uint32        not_empty_irq_sts;          // 18
  uint32        queue_rst;                  // 1c
  uint32        not_empty_sts;              // 20
  uint32        next_avail_mask;            // 24
  uint32        next_avail_queue;           // 28
} DQMCtlRegs_S;


/*
typedef struct DQMQRegs_S
{
  uint32        size;   // 00
  uint32        cfgA;   // 04
  uint32        cfgB;   // 08
  uint32        status; // 0c
  uint32        word0;  // 10
  uint32        word1;  // 14
  uint32        word2;  // 18
  uint32        word3;  // 1c
} DQMQRegs_S;
*/
typedef struct DQMQueueRegs_S
{
  uint32        size;   // 00
                #define Q_HEAD_PTR_MASK                     0xFFFC0000
                #define Q_HEAD_PTR_SHIFT                    18
                #define Q_TAIL_PTR_MASK                     0x0003FFF0
                #define Q_TAIL_PTR_SHIFT                    4
                #define Q_TOKEN_SIZE_MASK                   0x00000003
                #define Q_TOKEN_SIZE_SHIFT                  0
  uint32        cfgA;   // 04
                #define Q_SIZE_MASK                         0xffff0000
                #define Q_SIZE_SHIFT                        16
                #define Q_START_ADDR_MASK                   0x0000ffff
                #define Q_START_ADDR_SHIFT                  0
  uint32        cfgB;   // 08
                #define Q_NUM_TKNS_MASK                     0x3fff0000
                #define Q_NUM_TKNS_SHIFT                    16
                #define Q_LOW_WATERMARK_MASK                0x00003fff
                #define Q_LOW_WATERMARK_SHIFT               0
  uint32        status; // 0c
                #define AVAIL_TOKEN_SPACE_MASK              0x00003FFF
} DQMQueueRegs_S;

typedef struct DQMQCntrlRegs_S
{
  DQMQueueRegs_S Queue[32];
} DQMQCntrlRegs_S;


typedef struct DQMQueueDataRegs_S
{
  uint32        word0;   // 00
  uint32        word1;   // 04
  uint32        word2;   // 08
  uint32        word3;   // 0c
} DQMQueueDataRegs_S;

typedef struct DQMQDataRegs_S
{
  DQMQueueDataRegs_S Queue[32];
} DQMQDataRegs_S;

typedef struct DQMRegs_S 
{
  DQMCtlRegs_S     ctl;
} DQMRegs_S;

typedef struct DQMAllRegs_S 
{
  DQMRegs_S       DQMctl;
  uint32          resv[0x1E0]; //480
  DQMQCntrlRegs_S QCtrl;
  DQMQDataRegs_S  QData;
  uint32          MIB_NumFull[32];
  uint32          MIB_NumEmpty[32];
  uint32          MIB_TokensPushed[32];
} DQMAllRegs_S;

typedef struct PMRegs_S
{
  uint32        DCacheHit;      // 00
  uint32        DCacheMiss;     // 04
  uint32        ICacheHit;      // 08
  uint32        ICacheMiss;     // 0c
  uint32        InstnComplete;  // 10
  uint32        WTBMerge;       // 14
  uint32        WTBNoMerge;     // 18
} PMRegs_S;

typedef struct Dma_regs_S 
{
  uint32        dma_source;         // 00
  uint32        dma_dest;           // 04
  uint32        dma_cmd_list;       // 08
#define         DMA_CMD_INSERT                        0x01000000
#define         DMA_CMD_REPLACE                       0x02000000
#define         DMA_CMD_DELETE                        0x03000000
#define         DMA_CMD_CHECKSUM1                     0x04000000
#define         DMA_CMD_CHECKSUM2                     0x05000000
#define         DMA_CMD_INSERT_LENGTH                 0x06000000
#define         DMA_CMD_REPLACE_LENGTH                0x07000000
#define         DMA_CMD_MEMSET                        0x08000000
#define         DMA_CMD_OPCODE_MASK                   0xFF000000
#define         DMA_CMD_OFFSET_MASK                   0x00FFFF00
#define         DMA_CMD_LENGTH_MASK                   0x000000FF
#define         DMA_CMD_OPCODE_SHIFT                  24
#define         DMA_CMD_OFFSET_SHIFT                  8
#define         DMA_CMD_LENGTH_SHIFT                  0
  uint32        dma_len_ctl;        // 0c
#define         DMA_CTL_LEN_LEN_MASK                  0x00000FFF
#define         DMA_CTL_LEN_CONTINUE_BIT              0x00001000
#define         DMA_CTL_LEN_SRC_IS_TOKEN_BIT          0x00002000 
#define         DMA_CTL_LEN_DEST_ADDR_MASK            0x0000C000 
#define         DMA_CTL_LEN_DEST_IS_TOKEN_SHIFT       14 
#define         DMA_CTL_LEN_DEST_IS_TOKEN_MASK        0x0000C000 
#define         DMA_CTL_LEN_EXEC_CMD_LIST_BIT         0x00010000 
#define         DMA_CTL_LEN_WAIT_BIT                  0x00020000 
  uint32        dma_rslt_source;    // 10
  uint32        dma_rslt_dest;      // 14
  uint32        dma_rslt_resv;      // 18
  uint32        dma_rslt_len_stat;  // 1C
#define         DMA_RSLT_NOT_END_CMDS                 0x00100000
#define         DMA_RSLT_FLUSHED                      0x00080000
#define         DMA_RSLT_ABORTED                      0x00040000
#define         DMA_RSLT_ERR_CMD_FMT                  0x00020000
#define         DMA_RSLT_ERR_DEST                     0x00010000
#define         DMA_RSLT_ERR_SRC                      0x00008000
#define         DMA_RSLT_ERR_CMD_LIST                 0x00004000
#define         DMA_RSLT_ERR_DEST_LEN                 0x00002000
#define         DMA_RSLT_ERR_SRC_LEN                  0x00001000
} Dma_regs_S;

typedef struct DMARegs_S 
{
  Dma_regs_S    dma_ch[2];
  Dma_regs_S    resv[6];
} DMARegs_S;

typedef struct IOProcRegs_S
{
 CoprocCtlRegs_S    CoprocCtlRegs;
 OGMsgFifoRegs_S    OGMsgFifoRegs;
 INMsgFifoRegs_S    INMsgFifoRegs;
 DMARegs_S          DMARegs;
 DQMAllRegs_S       DQMRegs;
} IOProcRegs_S;

typedef struct TknIntfRegs_S
{
  uint32        tok_buf_size;   // 00
  uint32        tok_buf_base;   // 04
  uint32        tok_idx2ptr_idx;   // 08
  uint32        tok_idx2ptr_ptr; // 0c
  uint32        tok_ptr2idx_ptr;  // 10
  uint32        tok_ptr2idx_idx;  // 14
  uint32        resv[2];  // 18
} TknIntfRegs_S;
/*
typedef struct DmaCtlRegs_S
{
  uint32        dma_status;   // 00
  #define DMA_STATUS_DMAi_CMD_FULL_BIT                0x00000001
  #define DMA_STATUS_DMAi_RSLT_EMPTY_BIT              0x00000002
  #define DMA_STATUS_DMAi_RSLT_FULL_BIT               0x00000004
  uint32        dma0_3_fifo_status;   // 04
  uint32        dma4_7_fifo_status;   // 08
  #define DMA_FIFO_STATUS_DMAi_CMD_FIFO_ROOM_MASK     0x0000000F
  #define DMA_FIFO_STATUS_DMAi_RSLT_FIFO_DEPTH_MASK   0x000000F0
  uint32        resv[1]; 
} DmaCtlRegs_S;
*/


// Some definitions moved from 3380_map.h.  These are supposed to be obsolete...

#define utpCntrlRegs ((volatile CoprocCtlRegs_S *)((CTRL_REG_BLOCK_BASE&0x03ffffff)|UTP_BLOCK))
#define dtpCntrlRegs ((volatile CoprocCtlRegs_S *)((CTRL_REG_BLOCK_BASE&0x03ffffff)|DTP_BLOCK))
#define fapCntrlRegs ((volatile CoprocCtlRegs_S *)((CTRL_REG_BLOCK_BASE&0x03ffffff)|FAP_BLOCK))
#define msgProcCntrlRegs ((volatile CoprocCtlRegs_S *)((CTRL_REG_BLOCK_BASE&0x03ffffff)|MSP_BLOCK))
#define mepProcCntrlRegs ((volatile CoprocCtlRegs_S *)((CTRL_REG_BLOCK_BASE&0x03ffffff)|MEP_BLOCK))

#define utpMsgIdReg ((volatile MsgIdRegs_S *)((MSG_ID_BASE&0x03ffffff)|UTP_BLOCK))
#define dtpMsgIdReg ((volatile MsgIdRegs_S *)((MSG_ID_BASE&0x03ffffff)|DTP_BLOCK))
#define fapMsgIdReg ((volatile MsgIdRegs_S *)((MSG_ID_BASE&0x03ffffff)|FAP_BLOCK))
#define msgProcMsgIdReg ((volatile MsgIdRegs_S *)((MSG_ID_BASE&0x03ffffff)|MSP_BLOCK))
#define mepProcMsgIdReg ((volatile MsgIdRegs_S *)((MSG_ID_BASE&0x03ffffff)|MEP_BLOCK))

#define utpInMsgReg ((volatile INMsgFifoRegs_S *)((IN_MSG_BASE&0x03ffffff)|UTP_BLOCK))
#define dtpInMsgReg ((volatile INMsgFifoRegs_S *)((IN_MSG_BASE&0x03ffffff)|DTP_BLOCK))
#define fapInMsgReg ((volatile INMsgFifoRegs_S *)((IN_MSG_BASE&0x03ffffff)|FAP_BLOCK))
#define msgProcInMsgReg ((volatile INMsgFifoRegs_S *)((IN_MSG_BASE&0x03ffffff)|MSP_BLOCK))
#define mepProcInMsgReg ((volatile INMsgFifoRegs_S *)((IN_MSG_BASE&0x03ffffff)|MEP_BLOCK))

/********* UTP Accesses *************/
#include "UtpConfig.h"
#define UtpMibReg   ((volatile UtpMibs_s *)(UTP_BLOCK + UTP_MIB_ADDR_OFFSET))
#define UtpQDepth   ((volatile QDepthRegs_s *)(UTP_BLOCK + TAM_Q_DEPTH_ADDR_OFFSET))

#endif

#endif















