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
//  Filename:         sim_top.h
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
#ifndef SIM_TOP_H__
#define SIM_TOP_H__



typedef union {
  struct {
    uint32 Simen                          :1; 
    uint32 Recover                        :1; 
    uint32 Pts                            :1; 
                                              
                                              
                                              
                                              
    uint32 Clkf                           :1; 
                                              
    uint32 Fifoen                         :1; 
    uint32 Gcten                          :1; 
    uint32 Vccen                          :1; 
    uint32 Vppen                          :1; 
    uint32 Txretry                        :4; 
                                              
                                              
                                              
    uint32 Rxretry                        :4; 
                                              
    uint32 Srst                           :1; 
    uint32 Dmaen                          :1; 
    uint32 Reserved                       :1; 
    uint32 Etucks                         :2; 
                                              
                                              
                                              
                                              
    uint32 Reserved2                      :1; 
    uint32 Parsel                         :1; 
                                              
    uint32 Daten                          :1; 
                                              
    uint32 Reserved3                      :3; 
    uint32 CheckBackEn                    :1; 
    uint32 Stop                           :1; 
                                              
    uint32 Stps                           :1; 
                                              
                                              
    uint32 SimdatMaskEn                   :1; 
                                              
                                              
    uint32 Rsts                           :1; 
                                              
  } Bits;
  uint32 Reg32;
}  SimiScr;


typedef union {
  struct {
    uint32 Reserved                       :19;
    uint32 Pare                           :1; 
                                              
    uint32 Reserved2                      :12;
  } Bits;
  uint32 Reg32;
}  SimiSdr;


typedef union {
  struct {
    uint32 Reserved                       :24;
    uint32 Secgtr                         :8; 
                                              
  } Bits;
  uint32 Reg32;
}  SimiSecgtr;


typedef union {
  struct {
    uint32 Flush                          :1; 
    uint32 Fifocnt                        :5; 
    uint32 RxTout                         :10;
    uint32 Reserved                       :4; 
    uint32 TxThre                         :4; 
    uint32 Reserved2                      :3; 
    uint32 RxThre                         :5; 
  } Bits;
  uint32 Reg32;
}  SimiSfcr;


typedef union {
  struct {
    uint32 Reserved                       :8; 
    uint32 Sgccr                          :24;
                                              
                                              
  } Bits;
  uint32 Reg32;
}  SimiSgccr;


typedef union {
  struct {
    uint32 Reserved                       :8; 
    uint32 Sgcvr                          :24;
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  SimiSgcvr;


typedef union {
  struct {
    uint32 Reserved                       :23;
    uint32 Gcnti                          :1; 
    uint32 Txthre                         :1; 
    uint32 Rxtout                         :1; 
    uint32 Rxthre                         :1; 
    uint32 Rovf                           :1; 
    uint32 Rdr                            :1; 
    uint32 Txdone                         :1; 
    uint32 Terr                           :1; 
    uint32 Perr                           :1; 
  } Bits;
  uint32 Reg32;
}  SimiSier;


typedef union {
  struct {
    uint32 Reserved                       :1; 
    uint32 Simstate                       :7; 
    uint32 Fifowpt                        :4; 
    uint32 Reserved2                      :1; 
    uint32 Fiforpt                        :5; 
    uint32 Reserved3                      :1; 
    uint32 Rxstate                        :4; 
    uint32 Reserved4                      :1; 
    uint32 Txstate                        :3; 
    uint32 Reserved5                      :2; 
    uint32 Txmode                         :1; 
    uint32 Order                          :1; 
                                              
    uint32 Sense                          :1; 
                                              
  } Bits;
  uint32 Reg32;
}  SimiSimdebug;


typedef union {
  struct {
    uint32 Reserved                       :16;
    uint32 Txmode                         :1; 
    uint32 Reserved2                      :4; 
    uint32 RxRepeat                       :1; 
                                              
                                              
    uint32 Txidle                         :1; 
                                              
                                              
    uint32 Gcnti                          :1; 
    uint32 Txthre                         :1; 
                                              
                                              
    uint32 Rxtout                         :1; 
                                              
                                              
    uint32 Rxthre                         :1; 
                                              
                                              
    uint32 Rovf                           :1; 
                                              
    uint32 Rdr                            :1; 
                                              
    uint32 Txdone                         :1; 
                                              
    uint32 Terr                           :1; 
                                              
    uint32 Perr                           :1; 
                                              
                                              
  } Bits;
  uint32 Reg32;
}  SimiSsr;


typedef union {
  struct {
    uint32 Reserved                       :24;
    uint32 Stgtr                          :8; 
                                              
  } Bits;
  uint32 Reg32;
}  SimiStgtr;

typedef struct {
  SimiScr                             Scr;                     
  SimiSsr                             Ssr;                     
  SimiSdr                             Sdr;                     
  SimiSier                            Sier;                    
  SimiSfcr                            Sfcr;                    
  SimiSecgtr                          Secgtr;                  
  SimiStgtr                           Stgtr;                   
  SimiSgccr                           Sgccr;                   
  SimiSgcvr                           Sgcvr;                   
  uint8                               Pad0[0xc];
  SimiSimdebug                        Simdebug;                
}  SimiRegisters0;

#endif 



