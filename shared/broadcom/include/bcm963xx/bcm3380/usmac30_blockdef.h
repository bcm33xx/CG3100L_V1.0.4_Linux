// ****************************************************************************
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
// ****************************************************************************
//
//  Filename:         usmac30_blockdef.h
//  Generated by:     RDB Utility
//  Creation Date:    8/7/2008
//  Command Line:     
// ****************************************************************************
//
// IMPORTANT: DO NOT MODIFY, THIS IS AN AUTOGENERATED FILE. 
// Please modify the source .rdb file instead if you need to change this file. 
// Contact Jeff Bauch if you need more information.
//
// ****************************************************************************
#ifndef USMAC30_BLOCKDEF_H__
#define USMAC30_BLOCKDEF_H__

#include "usmac30.h"

  
typedef struct {
  Usmac30Usmac30ChannelRegs      ChannelRegs;
} Usmac30BlockChannel;

  
typedef struct {
  Usmac30Usmac30ChannelQueRegs   ChannelQueRegs;
} Usmac30BlockChannelQueue;

  
typedef struct {
  Usmac30Usmac30QueueRegs        QueueRegs;
} Usmac30BlockQueue;

  
typedef struct {
  Usmac30Usmac30SidRegs          SidRegs;
} Usmac30BlockSid;

  
typedef struct {
  Usmac30Usmac30GroupSidRegs     GroupSidRegs;
} Usmac30BlockGroupsid;

  
typedef struct {
  Usmac30Usmac30BurstProfileRegs BurstProfileRegs;
} Usmac30BlockBurstprofile;

  
typedef struct {
  Usmac30Usmac30SimControlRegs   SimControlRegs;
} Usmac30BlockSimcontrol;

  
typedef struct {
  Usmac30Usmac30PicoControlRegs  PicoControlRegs;
  uint8                          Pad0[0x1];
} Usmac30BlockPicocontrol;

  
typedef struct {
  Usmac30Usmac30InterruptRegs    InterruptRegs;
  uint8                          Pad0[0xf];
} Usmac30BlockInterrupt;

  
typedef struct {
  Usmac30Usmac30MibCountersRegs  MibCountersRegs;
  uint8                          Pad0[0x1];
} Usmac30BlockMibcounters;

  
typedef struct {
  Usmac30Usmac30FlowDiagMibCountersRegs FlowDiagMibCountersRegs;
  uint8                          Pad0[0x1];
} Usmac30BlockFlowdiagmibcnt;

  
typedef struct {
  union {
    Usmac30BlockChannel           Channelblk;
    struct {
      uint8                      Pad0[0x140];
      Usmac30BlockChannelQueue   ChannelQueueblk;
    };
    struct {
      uint8                      Pad1[0x170];
      Usmac30BlockQueue          Queueblk;
    };
    struct {
      uint8                      Pad2[0x370];
      Usmac30BlockSid            Sidblk;
    };
    struct {
      uint8                      Pad3[0x470];
      Usmac30BlockGroupsid       Groupsidblk;
    };
    struct {
      uint8                      Pad4[0x4f0];
      Usmac30BlockBurstprofile   Burstprofileblk;
    };
    struct {
      uint8                      Pad5[0x930];
      Usmac30BlockSimcontrol     Simcontrolblk;
    };
    struct {
      uint8                      Pad6[0x980];
      Usmac30BlockPicocontrol    Picocontrolblk;
    };
    struct {
      uint8                      Pad7[0x990];
      Usmac30BlockInterrupt      Interruptblk;
    };
    struct {
      uint8                      Pad8[0xc00];
      Usmac30BlockMibcounters    Mibcountersblk;
    };
    struct {
      uint8                      Pad9[0xe00];
      Usmac30BlockFlowdiagmibcnt Flowdiagmibcntblk;
    };
  };
  uint8                          Pad10[0x190];
}  Usmac30BlockUsmac301;

#endif 



