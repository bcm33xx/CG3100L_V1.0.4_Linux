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
//  Filename:         led.h
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
#ifndef LED_H__
#define LED_H__


enum LED_MODE_DEF {
  LED_MODE_DEF_OFF=0x0,                              
  LED_MODE_DEF_BLINK=0x1,                            
  LED_MODE_DEF_FLASH=0x2,                            
  LED_MODE_DEF_ON=0x3,                               
};

enum LED_LINK_MASK {
  LED_LINK_MASK_RSVD1=0x8,                           
  LED_LINK_MASK_RSVD0=0x4,                           
  LED_LINK_MASK_EPHY0_LNK1=0x2,                      
  LED_LINK_MASK_EPHY0_LNK0=0x1,                      
};

enum LED_ACT_MASK {
  LED_ACT_MASK_USB1_ACT=0x8,                         
  LED_ACT_MASK_USB0_ACT=0x4,                         
  LED_ACT_MASK_EPHY1_ACT=0x2,                        
  LED_ACT_MASK_EPHY0_ACT=0x1,                        
};


typedef union {
  struct {
    uint32 Reserved                       :8; 
    uint32 Led23Hwdis                     :1; 
                                              
                                              
                                              
    uint32 Led22Hwdis                     :1; 
    uint32 Led21Hwdis                     :1; 
    uint32 Led20Hwdis                     :1; 
    uint32 Led19Hwdis                     :1; 
    uint32 Led18Hwdis                     :1; 
    uint32 Led17Hwdis                     :1; 
    uint32 Led16Hwdis                     :1; 
    uint32 Led15Hwdis                     :1; 
    uint32 Led14Hwdis                     :1; 
    uint32 Led13Hwdis                     :1; 
    uint32 Led12Hwdis                     :1; 
    uint32 Led11Hwdis                     :1; 
    uint32 Led10Hwdis                     :1; 
    uint32 Led9Hwdis                      :1; 
    uint32 Led8Hwdis                      :1; 
    uint32 Led7Hwdis                      :1; 
    uint32 Led6Hwdis                      :1; 
    uint32 Led5Hwdis                      :1; 
    uint32 Led4Hwdis                      :1; 
    uint32 Led3Hwdis                      :1; 
    uint32 Led2Hwdis                      :1; 
    uint32 Led1Hwdis                      :1; 
    uint32 Led0Hwdis                      :1; 
  } Bits;
  uint32 Reg32;
}  LedLedHwDis;


typedef union {
  struct {
    uint32 LedTest                        :1; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 ShiftTest                      :1; 
                                              
                                              
                                              
    uint32 Reserved                       :14;
    uint32 SerialLedDataPpol              :1; 
                                              
                                              
                                              
    uint32 SerialLedClkNpol               :1; 
                                              
                                              
                                              
                                              
                                              
    uint32 SerialLedMuxSel                :1; 
                                              
                                              
    uint32 SerialLedEn                    :1; 
                                              
                                              
    uint32 FastIntv                       :6; 
                                              
                                              
                                              
    uint32 SlowIntv                       :6; 
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  LedLedInit;


typedef union {
  struct {
    uint32 Led7LinkMask                   :4; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 Led6LinkMask                   :4; 
    uint32 Led5LinkMask                   :4; 
    uint32 Led4LinkMask                   :4; 
    uint32 Led7ActMask                    :4; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 Led6ActMask                    :4; 
    uint32 Led5ActMask                    :4; 
    uint32 Led4ActMask                    :4; 
  } Bits;
  uint32 Reg32;
}  LedLedMask;


typedef union {
  struct {
    uint32 Reserved                       :16;
    uint32 Led7Mode                       :2; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 Led6Mode                       :2; 
    uint32 Led5Mode                       :2; 
    uint32 Led4Mode                       :2; 
    uint32 Led3Mode                       :2; 
    uint32 Led2Mode                       :2; 
    uint32 Led1Mode                       :2; 
    uint32 Led0Mode                       :2; 
  } Bits;
  uint32 Reg32;
}  LedLedMode0;


typedef union {
  struct {
    uint32 Led23Mode                      :2; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 Led22Mode                      :2; 
    uint32 Led21Mode                      :2; 
    uint32 Led20Mode                      :2; 
    uint32 Led19Mode                      :2; 
    uint32 Led18Mode                      :2; 
    uint32 Led17Mode                      :2; 
    uint32 Led16Mode                      :2; 
    uint32 Led15Mode                      :2; 
    uint32 Led14Mode                      :2; 
    uint32 Led13Mode                      :2; 
    uint32 Led12Mode                      :2; 
    uint32 Led11Mode                      :2; 
    uint32 Led10Mode                      :2; 
    uint32 Led9Mode                       :2; 
    uint32 Led8Mode                       :2; 
  } Bits;
  uint32 Reg32;
}  LedLedMode1;


typedef union {
  struct {
    uint32 Reserved                       :8; 
    uint32 Led23In                        :1; 
                                              
                                              
    uint32 Led22In                        :1; 
    uint32 Led21In                        :1; 
    uint32 Led20In                        :1; 
    uint32 Led19In                        :1; 
    uint32 Led18In                        :1; 
    uint32 Led17In                        :1; 
    uint32 Led16In                        :1; 
    uint32 Led15In                        :1; 
    uint32 Led14In                        :1; 
    uint32 Led13In                        :1; 
    uint32 Led12In                        :1; 
    uint32 Led11In                        :1; 
    uint32 Led10In                        :1; 
    uint32 Led9In                         :1; 
    uint32 Led8In                         :1; 
    uint32 Led7In                         :1; 
    uint32 Led6In                         :1; 
    uint32 Led5In                         :1; 
    uint32 Led4In                         :1; 
    uint32 Led3In                         :1; 
    uint32 Led2In                         :1; 
    uint32 Led1In                         :1; 
    uint32 Led0In                         :1; 
  } Bits;
  uint32 Reg32;
}  LedLedReadBack;


typedef union {
  struct {
    uint32 Reserved                       :8; 
    uint32 Led23Strobe                    :1; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 Led22Stb                       :1; 
    uint32 Led21Stb                       :1; 
    uint32 Led20Stb                       :1; 
    uint32 Led19Stb                       :1; 
    uint32 Led18Stb                       :1; 
    uint32 Led17Stb                       :1; 
    uint32 Led16Stb                       :1; 
    uint32 Led15Stb                       :1; 
    uint32 Led14Stb                       :1; 
    uint32 Led13Stb                       :1; 
    uint32 Led12Stb                       :1; 
    uint32 Led11Stb                       :1; 
    uint32 Led10Stb                       :1; 
    uint32 Led9Stb                        :1; 
    uint32 Led8Stb                        :1; 
    uint32 Led7Stb                        :1; 
    uint32 Led6Stb                        :1; 
    uint32 Led5Stb                        :1; 
    uint32 Led4Stb                        :1; 
    uint32 Led3Stb                        :1; 
    uint32 Led2Stb                        :1; 
    uint32 Led1Stb                        :1; 
    uint32 Led0Stb                        :1; 
  } Bits;
  uint32 Reg32;
}  LedLedStrobe;

typedef struct {
  LedLedInit                          LedInit;                 
  LedLedMode0                         LedMode0;                
  LedLedMode1                         LedMode1;                
  LedLedHwDis                         LedHwdis;                
  LedLedStrobe                        LedStrobe;               
  LedLedMask                          LedMask;                 
  LedLedReadBack                      LedReadback;             
}  LedRegs;

#endif 



