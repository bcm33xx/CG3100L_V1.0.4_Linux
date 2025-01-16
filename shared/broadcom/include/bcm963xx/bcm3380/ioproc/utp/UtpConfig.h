//****************************************************************************
//
// Copyright (c) 2008 Broadcom Corporation
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
//  $Id:$
//
//  Filename:       UtpConfig.h 
//  Author:         Vicheara (La) Long
//  Creation Date:  05/31/2007
//
//****************************************************************************
// 
// This file contains definitions use to initialize utp and its peripherals.

#ifndef UTP_CONFIG_H
#define UTP_CONFIG_H

#include "io_proc.h"
#include "mips_uart.h"

// Used for testing token path within UTP
//#define ENABLE_UTP_TEST_MODE

// Watchdog
#ifdef USE_SCV
    #define WATCHDOG_TIMER_COUNT            500000
#else
    // one secone for 160Mhz ubus clock
    #define WATCHDOG_TIMER_COUNT            160000000 * 1 
#endif

/******************************************************************************/
// USPP (crypto) defines
/******************************************************************************/
#define USPP_INVALID_SRC_TKN_MSG_ID         10
#define USPP_INVALID_SRC_TKN_MSG_SIZE       4
#define USPP_INVALID_DST_TKN_MSG_ID         11
#define USPP_INVALID_DST_TKN_MSG_SIZE       4

/******************************************************************************/
// UTP soft registers Definition
/******************************************************************************/
#define ENABLE_UTP_MIB
//#define ENABLE_TAM_Q_DEPTH_SOFT_REG

typedef struct UtpMibs_s
{
    uint32 SegDmaDiscardTokens;             // Discarded by SEGDMA due to request timeout 16x
    uint32 SegDmaDiscardBytes;              // Discarded by SEGDMA due to request timeout 16x
    uint32 FlowFlushDiscardTokens;          // Discarded due to flow was flushed
    uint32 FlowFlushDiscardBytes;           // Discarded due to flow was flushed
    uint32 InvalidDstTokens;                // Number of invalid destination tokens (detected by USPP and UTP)
    uint32 InvalidSrcTokens;                // Number of invalid source tokens (detected by USPP)
    uint32 DisableFlowDiscardTokens;        // Discarded due to flow not inservice
    uint32 DisableFlowDiscardBytes;         // Discarded due to flow not inservice
    uint32 Size0DiscardTokens;              // Discarded due to token size == 0
    uint32 Size0DiscardBytes;               // Discarded due to token size == 0
    uint32 InsertTimeoutDiscardTokens;      // Discarded due to timeout waiting for insert flag
    uint32 InsertTimeoutDiscardBytes;       // Discarded due to timeout waiting for insert flag
    uint32 DeleteTimeoutDiscardTokens;      // Discarded due to timeout waiting for delete flag
    uint32 UsppBusyCount;                   // Number of time USPP was busy
    uint32 TamFifoFullCount;                // DQM Q for affected flowId will not be served
    uint32 FlowResetFailCount;              // Number of attempt to reset flow failed
    uint32 InMsgUnknownType;                // Number of unknown msg type received via IncMsgFifo
    uint32 UnknownHostCmdCount;             // Number of unknown command from Host MIPS

    // Per flow MIBS.
    uint32 UsppRequestCount;                // Number of Uspp Request Msg sent to USPP
    uint32 UsppRequestBytes;                // Number of bytes sent to Uspp
    uint32 UsppResponseCount;               // Number of Uspp Response Msg received from USPP
    uint32 UsppResponseBytes;               // Number of bytes got from Uspp
    uint32 InsertTokens;                    // Number of tokens sent to TAM
    uint32 InsertBytes;                     // Number of bytes sent to TAM
    uint32 DeleteTokens;                    // Number of tokens SEGDMA sent to USMAC
    uint32 DeleteBytes;                     // Number of bytes SEGDMA sent to USMAC
    uint32 AckCelPktCount;                  // Number of AckCel packets(tokens)
    uint32 AckCelReplacedTokens;            // Number of AckCel packets that were swapped

    // For FPGA workaround only
    uint32 DstTknRlsCount;                  // When this reach a threshold we send Host an IRQ via DQM
} UtpMibs_s;

typedef struct QDepthRegs_s
{
    uint16 Queue[32];
} QDepthRegs_s;

/******************************************************************************/
// Carve out Shared mem for UTP soft registers: MIBS, queuDepth regs.  Whatever
// left we will use for DQM queues below.
/******************************************************************************/
#define UTP_MIB_BASE_ADDR       SHARED_MEM_BASE
// this is use by HOST MIPS code
#define UTP_MIB_ADDR_OFFSET     (UTP_MIB_BASE_ADDR & 0x0FFFFFFF)
#define UTP_MIB_SIZE            ((sizeof(UtpMibs_s) + 3) >> 2)

#define TAM_Q_DEPTH_BASE_ADDR   (UTP_MIB_BASE_ADDR + (UTP_MIB_SIZE * 4))
#define TAM_Q_DEPTH_ADDR_OFFSET (TAM_Q_DEPTH_BASE_ADDR & 0x03FFFFFF)

#ifdef ENABLE_TAM_Q_DEPTH_SOFT_REG
  #define TAM_Q_DEPTH_SIZE      ((sizeof(QDepthRegs_s) + 3) >> 2)
#else
  #define TAM_Q_DEPTH_SIZE      0
#endif

/******************************************************************************/
// DQM Configuration Definition
/******************************************************************************/
// SHARED memory: UTP has 16KB(4K x 32) and 8KB(2Kx11) memories
//              2K x 11 is used specifically for Cluster FPM
#define UTP_UART_DQM_Q_START_ADDR       (UTP_MIB_SIZE + TAM_Q_DEPTH_SIZE)
#define UTP_DQM_2K_11_START_ADDR        ((UTP_SHARED_8K_11_MEM_BASE - UTP_SHARED_16K_MEM_BASE) >> 2)
#define UTP_DQM_START_ADDR              (UTP_UART_DQM_Q_START_ADDR + UART_DQM_Q_MEM_SIZE)

// To be removed later.  We dont need this anymore
#define UTP_DQM_TOTAL_MEM               ((4 * 1024) + (2 * 1024) - UTP_MIB_SIZE - TAM_Q_DEPTH_SIZE - UART_DQM_Q_MEM_SIZE) 

// DQM Queues number(start with 0, for FPGA: only suppport 20 queues)
// Q 0 thru 15 use for data flow 0 thru 15
#define US_DATA_DQM_Q                   0
#define HOST2IOP_DQM_Q                  18
#define IOP2HOST_DQM_Q                  19
#define TAM_FPM_DQM_Q                   17
#define TKN_FPM_DQM_Q                   16
#define TKN_FREEMSG_DQM_Q               20  // Host use this irq to refresh it Token ring before it depleted
#define SEND_COMPLETE_DQM_Q             21

// DQM Queues Usage from 4KE point of view
#define US_DATA_DQM_Q_CONSUMER          1
#define HOST2IOP_DQM_Q_CONSUMER         1
#define IOP2HOST_DQM_Q_CONSUMER         0
#define TAM_FPM_DQM_Q_CONSUMER          1
#define TKN_FPM_DQM_Q_CONSUMER          1
#define TKN_FREEMSG_DQM_Q_CONSUMER      0

// DQM Queues Disable IRQ
#define US_DATA_DQM_Q_IRQ_DISABLE       1
#define HOST2IOP_DQM_Q_IRQ_DISABLE      1
#define IOP2HOST_DQM_Q_IRQ_DISABLE      1
#define TAM_FPM_DQM_Q_IRQ_DISABLE       1
#define TKN_FPM_DQM_Q_IRQ_DISABLE       1
#define TKN_FREEMSG_DQM_Q_IRQ_DISABLE   0

// DQM Queues Token Size(in words)
#define US_DATA_DQM_Q_TKN_SIZE          (sizeof(UsPacketMsg)>>2)
#define HOST2IOP_DQM_Q_TKN_SIZE         4
#define IOP2HOST_DQM_Q_TKN_SIZE         4
#define TAM_FPM_DQM_Q_TKN_SIZE          1
#define TKN_FPM_DQM_Q_TKN_SIZE          1
#define TKN_FREEMSG_DQM_Q_TKN_SIZE      1
#define SEND_COMPLETE_DQM_Q_TKN_SIZE    2

// DQM Queues Memory Size(in words and multiple of Q_SIZE)
// All should add up to 16KB(4KWords)
#define HOST2IOP_DQM_Q_MEM_SIZE         HOST2IOP_DQM_Q_TKN_SIZE * 16    // 64 words
#define IOP2HOST_DQM_Q_MEM_SIZE         IOP2HOST_DQM_Q_TKN_SIZE * 200   // 800 words
#define TAM_FPM_DQM_Q_MEM_SIZE          2048                            // 2048 words, use 2Kx11
#ifdef CONFIG_DQM_SIZE_EQUALLY
    #define NUMBER_DQM_DATA_QUEUES      16
    #define US_DATA_DQM_Q_MEM_SIZE      (sizeof(UsPacketMsg)>>2) * 16   // 64 words, 64x16=1024 words
    #define TKN_FPM_DQM_Q_MEM_SIZE      512                             // 512 words, 200 tokens
#else
    #define NUMBER_DQM_DATA_QUEUES      3
    #define NUMBER_DQM_UGS_QUEUES       13
    #define US_DATA_DQM_Q0_MEM_SIZE     (sizeof(UsPacketMsg)>>2) * 128 * 2  // 32 words
    #define US_DATA_DQM_Q1_2_MEM_SIZE   (sizeof(UsPacketMsg)>>2) * 64   // 32 words
    #define US_DATA_DQM_Q3_15_MEM_SIZE  (sizeof(UsPacketMsg)>>2) * 8    // 32 words
        // TEST MODE 
        #define TEST_TKN_DQM_Q          22
        #define TEST_TKN_DQM_Q_CONSUMER 1 
        #define TEST_TKN_DQM_Q_TKN_SIZE 1 
        #define TEST_TKN_DQM_Q_MEM_SIZE 64                              // 64 words
        // 
    #define TKN_FPM_DQM_Q_MEM_SIZE      1024                            // 1024 words, 2K tokens
#endif
#define TKN_FREEMSG_DQM_Q_MEM_SIZE      4                               // 4 words
#define SEND_COMPLETE_DQM_Q_MEM_SIZE    SEND_COMPLETE_DQM_Q_TKN_SIZE * 32

// DQM Queues Low Watermark 
#define US_DATA_DQM_Q_LOW_WATER         3
#define HOST2IOP_DQM_Q_LOW_WATER        3
#define IOP2HOST_DQM_Q_LOW_WATER        3
#define TAM_FPM_DQM_Q_LOW_WATER         3
#define TKN_FPM_DQM_Q_LOW_WATER         3
#define TKN_FREEMSG_DQM_Q_LOW_WATER     1
#define SEND_COMPLETE_DQM_Q_LOW_WATER   1

// When number of source token released reaches this threshold UTP will push a token
// into TKN_FREEMSG_DQM_Q to generate interrupt to the host
#define SRC_TKN_RLS_CNT_THRESHOLD       40

// INCOMING MSG FIFO
#define INC_MSG_FIFO_LOW_WATER          8

// defining command interface
enum
{
    /// HostToUtpCmd
    kDqm_HostToUtpCmd_PingRequest                   = 2,
    kDqm_HostToUtpCmd_CreateFlow                    = 100,
    kDqm_HostToUtpCmd_RemoveFlow                    = 101,
    kDqm_HostToUtpCmd_FlushFlow                     = 102,
    kDqm_HostToUtpCmd_ShowMibsRequest               = 103,
    kDqm_HostToUtpCmd_ClearMibsRequest              = 104,
    kDqm_HostToUtpCmd_ShowUartBufferRequest         = 105,
    kDqm_HostToUtpCmd_UtpTestConfigRequest          = 106,
    kDqm_HostToUtpCmd_UtpTestResultRequest          = 107,
    kDqm_HostToUtpCmd_UpdateMibsRequest             = 109,

    /// UtpToHostCmdSatus
    kDqm_UtpToHostCmdStatus_InitComplete            = 1,
    kDqm_UtpToHostCmdStatus_PingReply               = 2,
    kDqm_UtpToHostCmdStatus_Error                   = 16,
        kUtpErrCode_NoError                     = 0,
        kUtpErrCode_TokenAlreadyInserted        = 1,
        kUtpErrCode_DeletingTokenNotMatched     = 2,
        kUtpErrCode_DeletingNonInsertedToken    = 3,
        kUtpErrCode_TamError                    = 4,
        kUtpErrCode_WatchdogReset               = 5,
    kDqm_UtpToHostCmdStatus_Warning                 = 17,
    kDqm_UtpToHostCmdStatus_CreateFlowReply         = 100,
    kDqm_UtpToHostCmdStatus_RemoveFlowReply         = 101,
    kDqm_UtpToHostCmdStatus_FlushFlowReply          = 102,
    kDqm_UtpToHostCmdStatus_ShowMibsReply           = 103,
    kDqm_UtpToHostCmdStatus_ClearMibsReply          = 104,
    kDqm_UtpToHostCmdStatus_ShowUartBufferReply     = 105,
        kDqm_UtpToHostCmdStatus_Successful              = 0,
        kDqm_CreateFlowReply_Failed_FlowExisted         = 1,
        kDqm_CreateFlowReply_Failed_OutOfClusterIds     = 2,
        kDqm_RemoveFlowReply_Failed_FlowNotExisted      = 3,
        kDqm_RemoveFlowReply_Failed_FreeingTokens       = 4,
        kDqm_FlushFlowReply_Failed_FreeingTokens        = 4,
        kDqm_RemoveFlowReply_Failed_FreeingClusterIds   = 5,
        kDqm_FlushFlowReply_Failed_FlowNotExisted       = 6,
    
    kDqm_UtpToHostCmdStatus_UtpTestConfigReply      = 106,
    kDqm_UtpToHostCmdStatus_UtpTestResultReply      = 107,

    kDqm_UtpToHostCmdStatus_SendComplete            = 108,
        kDqm_SendComplete_Successful            = 0,
        kDqm_SendComplete_FailTokenNotMatched   = 1,
        kDqm_SendComplete_FailTokenQEmpty       = 2,
        kDqm_SendComplete_FailCantStoreToken    = 3,
    
    kDqm_UtpToHostCmdStatus_UpdateMibsReply         = 109,
    /// These are specific to UTP SIM
    kDqm_HostToUtpCmd_ConfigQListRamQueue           = 225,
    kDqm_HostToUtpCmd_DisplayQListRequest           = 226,
    kDqm_HostToUtpCmd_ClearQListQueueRequest        = 227,

    kDqm_UtpToHostCmdStatus_TokenInserted           = 200,
    kDqm_UtpToHostCmdStatus_TokenDeleted            = 201,
    kDqm_UtpToHostCmdStatus_TokenReplaced           = 202,
    kDqm_UtpToHostCmdStatus_ConfigQListRamQueueReply= 225,
    kDqm_UtpToHostCmdStatus_DisplayQListReply       = 226,
    kDqm_UtpToHostCmdStatus_ClearQListQueueReply    = 227,
};

// Host MIPS command/response structure
#define CMD_DONE_BIT                        0x02000000
#define CMD_DONE_SHIFT                      25

#define UTP_MSG_HDR_FLOW_MASK               0x00f00000
#define UTP_MSG_HDR_FLOW_SHIFT              20
#define UTP_MSG_HDR_Q_MASK                  0x01f00000
#define UTP_MSG_HDR_Q_SHIFT                 20

// Defines for Host to UTP and UTP to Host DQM queues
#define UTP_DQM_Q_CMD_MASK                  0x000000ff
#define UTP_DQM_Q_CMD_SHIFT                 0
#define HOST2UTP_CMD_Q_MASK                 0x01F00000
#define HOST2UTP_CMD_Q_SHIFT                20
#define UTP2HOST_CMD_SUCCESS                0x00000001
#define UTP2HOST_CMD_FAILED                 0x00000000

// Defines AckCelTag field
#define ACKCEL_TAG_MASK                     0x01ffffff
#define ACKCEL_TAG_SHIFT                    0
#define ACKCEL_COUNT_MASK                   0xfe000000
#define ACKCEL_COUNT_SHIFT                  25

// Host to UTP Request structures and defines
typedef struct Host2UtpCreateFlowRequest
{
    uint32 Command;
        #define CREATE_FLOW_SEG_HDR_ON_BIT                      0x02000000
        #define CREATE_FLOW_SEG_HDR_ON_SHIFT                    25
        #define CREATE_FLOW_SEG_OFF_CONCAT_ENABLE_BIT           0x01000000
        #define CREATE_FLOW_SEG_OFF_CONCAT_ENABLE_SHIFT         24
        #define CREATE_FLOW_ID_MASK                             0x00F00000
        #define CREATE_FLOW_ID_SHIFT                            20
        #define CREATE_FLOW_ACKCEL_ENABLE_BIT                   0x00080000
        #define CREATE_FLOW_ACKCEL_ENABLE_SHIFT                 19
        #define CREATE_FLOW_ACKCEL_Q_MASK                       0x00060000
        #define CREATE_FLOW_ACKCEL_Q_SHIFT                      17
        #define CREATE_FLOW_LOW_LATENCY_ENABLE_BIT              0x00010000
        #define CREATE_FLOW_LOW_LATENCY_ENABLE_SHIFT            16
        // use this for SIM only
        #define CREATE_FLOW_ACKCEL_MAX_COUNT_MASK               0x00007f00
        #define CREATE_FLOW_ACKCEL_MAX_COUNT_SHIFT              8
    uint32 LowQueueSize;
        #define CREATE_FLOW_LO_Q_SIZE_MASK                      0x00000fff
        #define CREATE_FLOW_LO_Q_SIZE_SHIFT                     0
    uint32 HighQueueSize;
        #define CREATE_FLOW_HI_Q_SIZE_MASK                      0x00000fff
        #define CREATE_FLOW_HI_Q_SIZE_SHIFT                     0
    uint32 QiConfig;
        #define CREATE_FLOW_QI_ENABLE_BIT                       0x80000000
        #define CREATE_FLOW_QI_ENABLE_SHIFT                     31
        #define CREATE_FLOW_QI_HIGH_WATER_THRESHOLD_MASK        0x0000ff00
        #define CREATE_FLOW_QI_HIGH_WATER_THRESHOLD_SHIFT       8
        #define CREATE_FLOW_QI_LOW_WATER_THRESHOLD_MASK         0x000000ff
        #define CREATE_FLOW_QI_LOW_WATER_THRESHOLD_SHIFT        0
} Host2UtpCreateFlowRequest;

typedef struct Host2UtpRemoveFlowRequest
{
    uint32 Command;
        #define REMOVE_FLOW_ID_MASK        0x00F00000
        #define REMOVE_FLOW_ID_SHIFT       20
        #define REMOVE_FLOW_ACK_CEL_MASK   0x00002000
        #define REMOVE_FLOW_ACK_CEL_SHIFT  13
        #define REMOVE_FLOW_SIZE_MASK      0x00001FFF
        #define REMOVE_FLOW_SIZE_SHIFT     0
    uint32 Reserved1;
    uint32 Reserved2;
    uint32 Reserved3;
} Host2UtpRemoveFlowRequest;


// UTP to Host Status or Reply structures and defines
typedef struct Utp2HostCreateFlowReply
{
    uint32 Command;
    uint32 Result;
        #define CREATE_FLOW_RESULT_BIT                          0x00000001
        #define CREATE_FLOW_RESULT_SHIFT                        0
    uint32 Reserved1;
    uint32 Reserved2;
} Utp2HostCreateFlowReply;

// DQM US data message define.
typedef struct UsPacketMsg
{
    // see USPPRequestMsg for bit definition
    uint32 msgHdr;
    uint32 srcToken;
        #define US_TOKEN_SEND_COMPLETE                           0x40000000
    uint32 akclId;
    uint32 sidHdrLen;
}UsPacketMsg;

// UTP to configure QListRAM queue with startAddr, size
typedef struct Host2UtpConfigQListQueueRequest
{
    uint32 Command;
    uint32 QListQCfg;
    uint32 QListQSize;
    uint32 Queue;
} Host2UtpConfigQListQueueRequest;

typedef struct Utp2HostConfigQListQueueReply
{
    uint32 Command;
    uint32 Result;
    uint32 Reserved1;
    uint32 Reserved2;
} Utp2HostConfigQListQueueReply;

typedef struct Utp2HostErrorStatus
{
    uint32 Command;
    uint32 ErrorCode;
    uint32 Data1;
    uint32 Data2;
} Utp2HostErrorStatus;

#ifdef USE_SCV
// UTP to display QList RAM config registers
typedef struct Host2UtpDisplayQListRequest
{
    uint32 Command;
    uint32 Reserved1;
    uint32 Reserved2;
    uint32 Reserved3;
} Host2UtpDisplayQListRequest;

typedef struct Utp2HostDisplayQListReply
{
    uint32 Command;
    uint32 Result;
    uint32 Reserved1;
    uint32 Reserved2;
} Utp2HostDisplayQListReply;

// UTP to clear QList queue 
typedef struct Host2UtpClearQListQueueRequest
{
    uint32 Command;
    uint32 Reserved1;
    uint32 Reserved2;
    uint32 Queue;
} Host2UtpClearQListQueueRequest;

typedef struct Utp2HostClearQListQueueReply
{
    uint32 Command;
    uint32 Result;
    uint32 Reserved1;
    uint32 Reserved2;
} Utp2HostClearQListQueueReply;
#endif

#endif
