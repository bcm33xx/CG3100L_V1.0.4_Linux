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
//  Filename:         hs_spim_pingpong.h
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
#ifndef HS_SPIM_PINGPONG_H__
#define HS_SPIM_PINGPONG_H__



typedef union {
  struct {
    uint32 Reserved                       :17;
    uint32 SsNum                          :3; 
                                              
    uint32 Reserved2                      :1; 
    uint32 ProfileNum                     :3; 
                                              
    uint32 Reserved3                      :1; 
    uint32 TriggerNum                     :3; 
                                              
                                              
    uint32 Reserved4                      :1; 
    uint32 CommandValue                   :3; 
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  PingpongCommand;


typedef union {
  struct {
    uint32 Reserved                       :30;
    uint32 AllowMsgfifoReads              :1; 
                                              
                                              
                                              
                                              
                                              
    uint32 Reserved2                      :1; 
  } Bits;
  uint32 Reg32;
}  PingpongControl;


typedef union {
  struct {
    uint32 MsgEmpty                       :1; 
    uint32 MsgFull                        :1; 
    uint32 Reserved                       :4; 
    uint32 MsgTail                        :10;
    uint32 RxEmpty                        :1; 
    uint32 RxFull                         :1; 
    uint32 Reserved2                      :4; 
    uint32 RxTail                         :10;
  } Bits;
  uint32 Reg32;
}  PingpongFifoStatus;


typedef union {
  struct {
    uint32 Reserved                       :4; 
    uint32 ErrorByteOffset                :12;
                                              
                                              
                                              
    uint32 Reserved2                      :13;
    uint32 WaitingForTrigger              :1; 
    uint32 SourceBusy                     :1; 
    uint32 SourceGnt                      :1; 
                                              
  } Bits;
  uint32 Reg32;
}  PingpongStatus;

typedef struct {
  PingpongCommand                     Command;                 
  PingpongStatus                      Status;                  
  PingpongFifoStatus                  FifoStatus;              
  PingpongControl                     Control;                 
}  PingpongRegs;

#endif 



