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
//    Filename:      dtpHostDqmMessages.h
//    Author:        Mark Newcomer    
//    Creation Date: 10/18/2007
//
//**************************************************************************
//    Description:
//
//    This include file contains structures, #defines, and helper functions
//    to help build and parse messages passed between the host MIPS and 
//    the downstream token processor.
//  
//**************************************************************************

#ifndef FAP_BUILD
#include "bcmos.h"
#endif

#ifndef FAPHOSTMESSAGES_H
#define FAPHOSTMESSAGES_H

#define DQM_DMA_REQ_Q            0
#define DQM_DMA_RSLT_Q           1
#define DQM_PKT_FIL_REQ_Q        2
#define DQM_FFE_RSLT_Q           3
#define DQM_DPE_REQ_Q            4
#define DQM_FAP_IOPDMA0_CNTXT_Q  25
#define DQM_FFE_MEMSLOT_FPM_Q    26
#define DQM_FAP_PSM_MEMPOOL_Q    29
#define DQM_FAP_CMD_REQ_Q        30
#define DQM_FAP_CMD_REPLY_Q      31

#define DQM_CMD_Q_TIMEOUT      100
#define DQM_RESPONSE_Q_TIMEOUT 100

//Helper macros
#define WRITE_MSG_FIELD(msg,val,mask,shift)  (msg = ((val<<shift) & mask) | (msg & ~mask))
#define READ_MSG_FIELD(msg,val,mask,shift)   (val = ((msg & mask)>>shift))   
#define COMPARE_MSG_FIELD(msg, val, mask, shift) (((val << shift) & mask) != (msg & mask))

#define HOST2FAP_CMD_REQ_SIZE                4
#define HOST2FAP_CMD_RESP_SIZE               2
#define HOST2FAP_CMD_Q_DEPTH                 16

#define DQM_PKT_FIL_REQ_Q_SIZE               3
#define DQM_PKT_FIL_REQ_Q_DEPTH              64

#define DQM_FFE_RSLT_Q_SIZE                  3
#define DQM_FFE_RSLT_Q_DEPTH                 64

#define DQM_FFE_MEMSLOT_POOL_TOKSIZE         1
#define NUM_FFE_MEMSLOTS                     8

//#define DQM_DMA0_CNTXT_Q_SIZE              3
#define DQM_DMA0_CNTXT_Q_DEPTH               32

//This structure defines the message that is written to the DMA context save
//DQM Q. This info is saved off and recalled when a DMA is complete. This is
//done so then app will know what to do next with a packet.
typedef struct iopDmaContextQStruct
{
   unsigned int originalHostCmd;
#define FILTER_RESP_RESP_MASK         0x0000ffff
#define FILTER_RESP_RESP_SHIFT        0
#define FILTER_RESP_CONTEXT_ID_MASK   0xffff0000
#define FILTER_RESP_CONTEXT_ID_SHIFT  16
   unsigned int originalSrcToken;
   unsigned int destBufferAddr;
   unsigned int hostContext;
} iopDmaContextQStruct;

#define DQM_FAP_PSM_MEMPOOL_TOKSIZE          1

#define HOST2FAPMSG_COMMAND_MASK             0x000000ff
#define HOST2FAPMSG_COMMAND_SHIFT            0

//Host to FAP message request types.
#define HOST2FAP_PING_REQUEST                2
#define HOST2FAP_RESET_REQUEST               3
#define HOST2FAP_SHUTDOWN_REQUEST            4
#define HOST2FAP_LOWPOWER_REQUEST            5

//FAP to Host message reply types.
#define FAP2HOST_PING_RESPONSE               HOST2FAP_PING_REQUEST
#define FAP2HOST_INIT_COMPLETE               1

/* Host to FAP Message definitions ********************************************************/

//Host to FAP Ping Request
typedef struct Host2FapPingRequest
{
   unsigned int Command;
   unsigned int Data;
   unsigned int Reserved1;
   unsigned int Reserved2;
} Host2FapPingRequest;

//Host to FAP Reset Request
typedef struct Host2FapResetRequest
{
   unsigned int Command;
   unsigned int Reserved0;
   unsigned int Reserved1;
   unsigned int Reserved2;
} Host2FapResetRequest;

//Host to FAP Shutdown Request
typedef struct Host2FapShutdownRequest
{
   unsigned int Command;
   unsigned int Reserved0;
   unsigned int Reserved1;
   unsigned int Reserved2;
} Host2FapShutdownRequest;

//Host to FAP LowPower Request
typedef struct Host2FapLowPowerRequest
{
   unsigned int Command;
   unsigned int Reserved0;
   unsigned int Reserved1;
   unsigned int Reserved2;
} Host2FapLowPowerRequest;

//Host to FAP LowPower Request
typedef struct Host2FapPktFilterRequest
{
   unsigned int Command;
#define FILTER_CMD_CMD_MASK         0x000000ff
#define FILTER_CMD_CMD_SHIFT        0
#define FILTER_CMD_MEM_SLOT_MASK    0x000fff00
#define FILTER_CMD_MEM_SLOT_SHIFT   8
#define FILTER_CMD_CONTEXT_ID_MASK  0xfff00000
#define FILTER_CMD_CONTEXT_ID_SHIFT 20
   unsigned int Token;
   unsigned int hostContext;
} Host2FapPktFilterRequest;

/* FAP to Host Message definitions ********************************************************/

#define FAP2HOSTMSG_RESULT_BIT      0x00000001

//FAP to Host Ping Reply
typedef struct Fap2HostPingReply
{
   unsigned int Command;
   unsigned int Data;
} Fap2HostPingReply;

//FAP to Host Reset Reply
typedef struct Fap2HostResetReply
{
   unsigned int Command;
   unsigned int Data;
} Fap2HostResetReply;

//Host to FAP LowPower Request
typedef struct Host2FapPktFilterResponse
{
   unsigned int Response;
#define FILTER_RESP_RESP_MASK         0x0000ffff
#define FILTER_RESP_RESP_SHIFT        0
#define FILTER_RESP_CONTEXT_ID_MASK   0xffff0000
#define FILTER_RESP_CONTEXT_ID_SHIFT  16
   unsigned int Token;
   unsigned int hostContext;
} Host2FapPktFilterResponse;

#endif


