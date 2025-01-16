//****************************************************************************
//
// Copyright (c) 2007 Broadcom Corporation
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
//**************************************************************************
//    Filename:       fap_proc.h
//    Author:         Ashraf Awad
//    Date:           05/02/2007
//
//*****************************************************************************
//    Revision History:
//              0.1 Initial version.
//*****************************************************************************
//
//*****************************************************************************
//
//   PURPOSE: This file contains the address and register map for the FAP
//            parts of the 3.0 demo fpga.
//
//*****************************************************************************


#ifndef fap_proc_h
#define fap_proc_h

#define __x86_64__ 1
#include "io_proc.h"

/*
#define DSRM_BASE           0xe0080000
#define DSRM_RAM_BASE       0xe0080000
#define TAG_CACHE_RAM_BASE  (DSRM_BASE + 0x00000100)
#define STATE_RAM_BASE      (DSRM_BASE + 0x00001000)
#define CLSTR_RAM_BASE      (DSRM_BASE + 0x00002000)
#define TO_MAXPSN_RAM_BASE  (DSRM_BASE + 0x00004000)
#define RLD_RAM_BASE        (DSRM_BASE + 0x00005000)
#define TAG_RAM_BASE        (DSRM_BASE + 0x00006000)
#define SA_RAM_BASE         (DSRM_BASE + 0x00020000)
#define DSDMA_BASE          0x02800000 

#define TAG_CACHE_RAM_SIZE  (1<<6)
//#define STATS_RAM_SIZE      (1<< 11)
#define STATE_RAM_SIZE      (1<< 11)
#define CLSTR_RAM_SIZE      (0x500)
//#define TO_MINPSN_RAM_SIZE  (1<< 10)
#define TO_MAXPSN_RAM_SIZE  (1<< 10)
#define RLD_RAM_SIZE        (1<< 10)
#define TAG_RAM_SIZE        (0x2000)
#define SA_RAM_SIZE         (0x20000)

#define DSRM_FIFO_END       0x023F
#define TAG_CACHE_RAM_END   (TAG_CACHE_RAM_BASE+ TAG_CACHE_RAM_SIZE -1)
#define DSRM_END            (DSRM_BASE         + DSRM_FIFO_END) 
#define STATE_RAM_END       (STATE_RAM_BASE    + STATE_RAM_SIZE    -1)            
#define CLSTR_RAM_END       (CLSTR_RAM_BASE    + CLSTR_RAM_SIZE    -1)
#define TO_MAXPSN_RAM_END   (TO_MAXPSN_RAM_BASE+ TO_MAXPSN_RAM_SIZE-1)
#define RLD_RAM_END         (RLD_RAM_BASE      + RLD_RAM_SIZE      -1)
#define TAG_RAM_END         (TAG_RAM_BASE      + TAG_RAM_SIZE      -1)
#define SA_RAM_END          (SA_RAM_BASE       + SA_RAM_SIZE       -1)
#define DSDMA_END           0x02800500 
*/

#define DSRMRegs            ((volatile DSRMRegs_S *)DSRM_BASE)

#define FAP_DSPRAM_IO_BASE   0x80003000

#define FapDspramReg             ((volatile FAP_DSPRAM_IO_Regs_S *)FAP_DSPRAM_IO_BASE)

#define FAP_CTRL_REGS_BASE       (IO_PROC_BASE + 0x3f000)

#define ffe_Nugget_Mem           ((volatile unsigned int *)(IO_PROC_BASE + 0x41000))
#define ffe_Nugget_Mem_Size      256 //in long words
#define ffe_Packet_Mem           ((volatile unsigned int *)(IO_PROC_BASE + 0x42000))
#define ffe_Packet_Mem_Size      512 //in long words
#define ffe_Instruction_Mem      ((volatile unsigned int *)(IO_PROC_BASE + 0x43000))
#define ffe_Instruction_Mem_Size 4096 //in long words

//Pointer to FAP specific packet shared memory.
#define FAP_PSM_BASE    0xe0010000
#define FAP_PSM_END     0xe0018000
#define FAP_PSM_SIZE    (FAP_PSM_END - FAP_PSM_BASE)

#define HOST_FAP_QSM_BASE 0x04000
#define HOST_FAP_PSM_BASE 0x10000
//#define FAP_QSM_BASE      0xe0004000
//#define FAP_PSM_BASE      0xe0010000
#define DPE_QSM_BASE      0x4000
#define DPE_PSM_BASE      0x8000

#define HOST_DPE_BASE     0x40000
#define MIPS_DPE_BASE     0xe0040000


//This is the allocatable block size for memory that holds FAP packets
//that are DMA'ed from system memory. We eill start with 2k buffers. This 
//will probably be reduced since we may not need the entire packet.
#define FAP_PSM_PKT_BUFF_SIZE    2048

#define pcPsmMemPtr         ((volatile unsigned char *)FAP_PSM_BASE)

typedef struct FAP_DSPRAM_IO_Regs_S
{
  uint32    gp_in;             // CTL: READ ONLY
  uint32    og_msg_sts;        // OUT: READ ONLY
  uint32    in_msg_sts;        // INC: READ ONLY
  uint32    not_empty_sts;     // DQM: READ ONLY
  uint32    next_avail_queue;  // DQM: READ ONLY
  uint32    status[32];        // DQM: READ ONLY
} FAP_DSPRAM_IO_Regs_S;

typedef struct FAPCtlRegs_S
{
   unsigned int   DPE_BASIC_FFE_CMD_WORD0;         //0x40000 DPE_BASIC_FFE_CMD_WORD0 FFE command word0 
   unsigned int   DPE_BASIC_FFE_CMD_WORD1;         //0x40004 DPE_BASIC_FFE_CMD_WORD1 FFE command word1 
   unsigned int   DPE_BASIC_FFE_STS_WORD0;         //0x40008 DPE_BASIC_FFE_STS_WORD0 FFE status word0 
   unsigned int   DPE_BASIC_FFE_STS_WORD1;         //0x4000c DPE_BASIC_FFE_STS_WORD1 FFE status word1 
   unsigned int   DPE_BASIC_DPE_CONFIG;            //0x40010 DPE_BASIC_DPE_CONFIG DPE configuration register 
   unsigned int   DPE_BASIC_CMD_FIFO_1_STATUS;     //0x40014 DPE_BASIC_CMD_FIFO_1_STATUS CMD_FIFO_1_STATUS 
   unsigned int   DPE_BASIC_CMD_FIFO_2_STATUS;     //0x40018 DPE_BASIC_CMD_FIFO_2_STATUS CMD_FIFO_2_STATUS 
   unsigned int   DPE_BASIC_STS_FIFO_1_STATUS;     //0x4001c DPE_BASIC_STS_FIFO_1_STATUS STS_FIFO_1_STATUS 
   unsigned int   DPE_BASIC_STS_FIFO_2_STATUS;     //0x40020 DPE_BASIC_STS_FIFO_2_STATUS STS_FIFO_2_STATUS 
   unsigned int   DPE_BASIC_DMA_FIFO_STATUS;       //0x40024 DPE_BASIC_DMA_FIFO_STATUS DMA_FIFO_STATUS 
   unsigned int   DPE_BASIC_DMA_CTRL_FIFO_STATUS;  //0x40028 DPE_BASIC_DMA_CTRL_FIFO_STATUS DMA_FIFO_CTRL_STATUS 
   unsigned int   DPE_BASIC_FIFO_STATUS;           //0x4002c DPE_BASIC_FIFO_STATUS FIFO_STATUS 
   unsigned int   DPE_BASIC_FIFO_STATUS_MASK;      //0x40030 DPE_BASIC_FIFO_STATUS_MASK FIFO_STATUS 
   unsigned int   DPE_BASIC_MPEG_FIFO_STATUS;      //0x40034 DPE_BASIC_MPEG_FIFO_STATUS MPEG_FIFO_STATUS 
   unsigned int   DPE_BASIC_MPEG_STATUS;           //0x40038 DPE_BASIC_MPEG_STATUS MPEG_STATUS 
   unsigned int   DPE_BASIC_MPEG_STATUS_MASK;      //0x4003c DPE_BASIC_MPEG_STATUS_MASK MPEG_STATUS_MASK 
   unsigned int   DPE_BASIC_MPEG_TIMESTAMP;        //0x40040 DPE_BASIC_MPEG_TIMESTAMP MPEG_TIMESTAMP 
   unsigned int   DPE_BASIC_FFE_STATUS;            //0x40044 DPE_BASIC_FFE_STATUS FFE_STATUS 
   unsigned int   DPE_BASIC_FFE_STATUS_MASK;       //0x40048 DPE_BASIC_FFE_STATUS_MASK FFE_STATUS_MASK 
   unsigned int   DPE_BASIC_DPE_INTERRUPT;         //0x4004c DPE_BASIC_DPE_INTERRUPT DPE_INTERRUPT 
   unsigned int   DPE_BASIC_DEBUG_STATE_CODES;     //0x40050 DPE_BASIC_DEBUG_STATE_CODES DEBUG_STATE_CODES 
   unsigned int   DPE_BASIC_FFE_DEBUG1;            //0x40054 DPE_BASIC_FFE_DEBUG1 FFE_DEBUG1 
   unsigned int   DPE_BASIC_FFE_DEBUG2;            //0x40058 DPE_BASIC_FFE_DEBUG2 FFE_DEBUG2 
   unsigned int   DPE_BASIC_FFE_CTRL1;             //0x4005c DPE_BASIC_FFE_CTRL1 FFE_CTRL1 
   unsigned int   DPE_BASIC_FFE_CTRL2;             //0x40060 DPE_BASIC_FFE_CTRL2 FFE_CTRL2 
   unsigned int   DPE_BASIC_DIAG_OUT;              //0x40064 DPE_BASIC_DIAG_OUT DIAG_OUT 
} FAPCtlRegs_S;

#endif
