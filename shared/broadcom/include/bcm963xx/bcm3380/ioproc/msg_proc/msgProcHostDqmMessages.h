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
//**************************************************************************
//    Filename:      msgProcHostDqmMessages.h
//    Author:        Mark Newcomer
//    Creation Date: 6/30/2008
//
//**************************************************************************
//    Description:
//
//    This include file contains structures, #defines, and helper functions
//    to help build and parse messages passed between the host MIPS and
//    the message token processor.
//
//**************************************************************************

#include "bcmos.h"

#ifndef MSGPROCHOSTMESSAGES_H
#define MSGPROCHOSTMESSAGES_H

#define INMSG_FIFO_EMPTY_MBDMA_CHECK      1000

// High priority is greater than this value on the traffic priority
// So 0-3 = low priority,  4-7 = high priority
#define HIGH_PRIORITY_START               3

//Ethernet HAL DQM Q definitions.
#define DQM_PKT_RX_Q                      0
#define DQM_PKT_TXHI_REQ_Q                1
#define DQM_PKT_TXLO_REQ_Q                2

// Dual ethernet support
#define DQM_PKT_RX2_Q                     4
#define DQM_PKT_TXHI2_REQ_Q               5
#define DQM_PKT_TXLO2_REQ_Q               6

#define DQM_USBHOST_TX_Q				  7
#define DQM_USBHOST_TX_Q_SIZE     		  2
#define DQM_USBHOST_TX_Q_DEPTH    		  64

#define DQM_USBHOST_RX_Q				  8
#define DQM_USBHOST_RX_Q_SIZE     		  2
#define DQM_USBHOST_RX_Q_DEPTH    		  64

//USB HAL DQM
#define DQM_USBCTRL_PKT_TX_REQ_Q          16
#define DQM_USBCTRL_PKT_TX_REQ_Q_SIZE     2
#define DQM_USBCTRL_PKT_TX_REQ_Q_DEPTH    64

#define DQM_USBCTRL_PKT_RX_Q              17
#define DQM_USBCTRL_PKT_RX_Q_SIZE         2
#define DQM_USBCTRL_PKT_RX_Q_DEPTH        64

#define DQM_USBDATA_PKT_TXLO_REQ_Q        18
#define DQM_USBDATA_PKT_TXLO_REQ_Q_SIZE   2
#define DQM_USBDATA_PKT_TXLO_REQ_Q_DEPTH  64

#define DQM_USBDATA_PKT_TXHI_REQ_Q        19
#define DQM_USBDATA_PKT_TXHI_REQ_Q_SIZE   2
#define DQM_USBDATA_PKT_TXHI_REQ_Q_DEPTH  64

#define DQM_USBDATA_PKT_RX_Q              20
#define DQM_USBDATA_PKT_RX_Q_SIZE         2
#define DQM_USBDATA_PKT_RX_Q_DEPTH        64

#define DQM_USBINTR_PKT_TX_REQ_Q          21
#define DQM_USBINTR_PKT_TX_REQ_Q_SIZE     2
#define DQM_USBINTR_PKT_TX_REQ_Q_DEPTH    32

//Test ONLY. Remove for production.
#define DQM_TEST_TX_Q            3
#define DQM_TEST_TX_Q_DEPTH      64

#define DQM_MSGPROC_CMD_REQ_Q    14
#define DQM_MSGPROC_CMD_REPLY_Q  15

#define DQM_CMD_Q_TIMEOUT      100
#define DQM_RESPONSE_Q_TIMEOUT 100

//Helper macros
#define WRITE_MSG_FIELD(msg,val,mask,shift)  (msg = ((val<<shift) & mask) | (msg & ~mask))
#define READ_MSG_FIELD(msg,val,mask,shift)   (val = ((msg & mask)>>shift))
#define COMPARE_MSG_FIELD(msg, val, mask, shift) (((val << shift) & mask) != (msg & mask))

#define HOST2MSGPROC_CMD_REQ_SIZE           4
#define HOST2MSGPROC_CMD_RESP_SIZE          2
#define HOST2MSGPROC_CMD_Q_DEPTH            8

#define DQM_PKT_TXHI_REQ_Q_SIZE               2
#define DQM_PKT_TXHI_REQ_Q_DEPTH              64

#define DQM_PKT_TXLO_REQ_Q_SIZE               2
#define DQM_PKT_TXLO_REQ_Q_DEPTH              128

#define DQM_PKT_RX_Q_SIZE                   2
#define DQM_PKT_RX_Q_DEPTH                  128

#define HOST2MSGPROC_COMMAND_MASK             0x000000ff
#define HOST2MSGPROC_COMMAND_SHIFT            0

//Host to FAP message request types.
#define HOST2MSGPROC_PING_REQUEST                2
#define HOST2MSGPROC_RESET_REQUEST               3
#define HOST2MSGPROC_SHUTDOWN_REQUEST            4
#define HOST2MSGPROC_LOWPOWER_REQUEST            5

//FAP to Host message reply types.
#define MSGPROC2HOST_PING_RESPONSE               HOST2MSGPROC_PING_REQUEST
#define MSGPROC2HOST_INIT_COMPLETE               1

/* Host to FAP Message definitions ********************************************************/

//Host to FAP Ping Request
typedef struct Host2MsgProcPingRequest
{
   unsigned int Command;
   unsigned int Data;
   unsigned int Reserved1;
   unsigned int Reserved2;
} Host2MsgProcPingRequest;

//Host to FAP Reset Request
typedef struct Host2MsgProcResetRequest
{
   unsigned int Command;
   unsigned int Reserved0;
   unsigned int Reserved1;
   unsigned int Reserved2;
} Host2MsgProcResetRequest;

//Host to FAP Shutdown Request
typedef struct Host2MsgProcShutdownRequest
{
   unsigned int Command;
   unsigned int Reserved0;
   unsigned int Reserved1;
   unsigned int Reserved2;
} Host2MsgProcShutdownRequest;

//Host to FAP LowPower Request
typedef struct Host2MsgProcLowPowerRequest
{
   unsigned int Command;
   unsigned int Reserved0;
   unsigned int Reserved1;
   unsigned int Reserved2;
} Host2MsgProcLowPowerRequest;

/* FAP to Host Message definitions ********************************************************/

#define MSGPROC2HOST_RESULT_BIT      0x00000001

//FAP to Host Ping Reply
typedef struct MsgProc2HostPingReply
{
   unsigned int Command;
   unsigned int Data;
} MsgProc2HostPingReply;

//FAP to Host Reset Reply
typedef struct MsgProc2HostResetReply
{
   unsigned int Command;
   unsigned int Data;
} MsgProc2HostResetReply;

#endif


