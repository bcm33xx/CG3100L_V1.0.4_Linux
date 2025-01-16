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
//  Filename:         hs_spim_global.h
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
#ifndef HS_SPIM_GLOBAL_H__
#define HS_SPIM_GLOBAL_H__



typedef union {
  struct {
    uint32 Reserved                       :13;
    uint32 MosiStateWhenIdle              :1; 
                                              
    uint32 ClkStateWhenGated              :1; 
                                              
                                              
    uint32 GateClkWhenSsoff               :1; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 PllClockControl                :8; 
                                              
                                              
                                              
    uint32 SsPolarity                     :8; 
                                              
                                              
  } Bits;
  uint32 Reg32;
}  GlobalcntrlControl;


typedef union {
  struct {
    uint32 TriggerRawState                :8; 
                                              
    uint32 TriggerLatchedState            :8; 
                                              
                                              
                                              
    uint32 TriggerSense                   :8; 
                                              
                                              
                                              
    uint32 TriggerType                    :8; 
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  GlobalcntrlExternalTriggerCntrl;


typedef union {
  struct {
    uint32 Reserved                       :8; 
    uint32 EnableMultibit                 :1; 
    uint32 SsNum                          :3; 
    uint32 Reserved2                      :1; 
    uint32 ProfileNum                     :3; 
    uint32 Reserved3                      :4; 
    uint32 NumDummyBytes                  :2; 
                                              
                                              
                                              
                                              
    uint32 NumAddrBytes                   :2; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 ReadOpcode                     :8; 
                                              
  } Bits;
  uint32 Reg32;
}  GlobalcntrlFlashControl;


typedef union {
  struct {
    uint32 Pingpong1UserIrq               :4; 
                                              
                                              
    uint32 Pingpong0UserIrq               :4; 
                                              
                                              
    uint32 Reserved                       :11;
    uint32 Pingpong1CtrlInvalid           :1; 
                                              
    uint32 Pingpong1PollTimeout           :1; 
                                              
    uint32 Pingpong1TxUnderflow           :1; 
                                              
    uint32 Pingpong1RxOverflow            :1; 
                                              
    uint32 Pingpong1CmndDone              :1; 
                                              
    uint32 Reserved2                      :3; 
    uint32 Pingpong0CtrlInvalid           :1; 
                                              
    uint32 Pingpong0PollTimeout           :1; 
                                              
    uint32 Pingpong0TxUnderflow           :1; 
                                              
    uint32 Pingpong0RxOverflow            :1; 
                                              
    uint32 Pingpong0CmndDone              :1; 
                                              
  } Bits;
  uint32 Reg32;
}  GlobalcntrlInterruptMask;


typedef union {
  struct {
    uint32 Pingpong1UserIrq               :4; 
                                              
                                              
    uint32 Pingpong0UserIrq               :4; 
                                              
                                              
    uint32 Reserved                       :11;
    uint32 Pingpong1CtrlInvalid           :1; 
                                              
    uint32 Pingpong1PollTimeout           :1; 
                                              
    uint32 Pingpong1TxUnderflow           :1; 
                                              
    uint32 Pingpong1RxOverflow            :1; 
                                              
    uint32 Pingpong1CmndDone              :1; 
                                              
    uint32 Reserved2                      :3; 
    uint32 Pingpong0CtrlInvalid           :1; 
                                              
    uint32 Pingpong0PollTimeout           :1; 
                                              
    uint32 Pingpong0TxUnderflow           :1; 
                                              
    uint32 Pingpong0RxOverflow            :1; 
                                              
    uint32 Pingpong0CmndDone              :1; 
                                              
  } Bits;
  uint32 Reg32;
}  GlobalcntrlInterruptMaskedStatus;


typedef union {
  struct {
    uint32 Pingpong1UserIrq               :4; 
                                              
                                              
    uint32 Pingpong0UserIrq               :4; 
                                              
                                              
    uint32 Reserved                       :11;
    uint32 Pingpong1CtrlInvalid           :1; 
                                              
    uint32 Pingpong1PollTimeout           :1; 
                                              
    uint32 Pingpong1TxUnderflow           :1; 
                                              
    uint32 Pingpong1RxOverflow            :1; 
                                              
    uint32 Pingpong1CmndDone              :1; 
                                              
    uint32 Reserved2                      :3; 
    uint32 Pingpong0CtrlInvalid           :1; 
                                              
    uint32 Pingpong0PollTimeout           :1; 
                                              
    uint32 Pingpong0TxUnderflow           :1; 
                                              
    uint32 Pingpong0RxOverflow            :1; 
                                              
    uint32 Pingpong0CmndDone              :1; 
                                              
  } Bits;
  uint32 Reg32;
}  GlobalcntrlInterruptStatus;

typedef struct {
  GlobalcntrlControl                  Control;                 
  GlobalcntrlExternalTriggerCntrl     ExternalTriggerCntrl;    
  GlobalcntrlInterruptStatus          InterruptStatus;         
  GlobalcntrlInterruptMaskedStatus    InterruptMaskedStatus;   
  GlobalcntrlInterruptMask            InterruptMask;           
  GlobalcntrlFlashControl             FlashControl;            
}  GlobalcntrlRegs;

#endif 



