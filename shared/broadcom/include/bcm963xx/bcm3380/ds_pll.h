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
//  Filename:         ds_pll.h
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
#ifndef DS_PLL_H__
#define DS_PLL_H__



typedef union {
  struct {
    uint32 ReservedForEco                 :10;
    uint32 DlyCh1                         :2; 
                                              
                                              
                                              
                                              
    uint32 EnCmlbuf1                      :1; 
                                              
                                              
    uint32 PwrdnCh1                       :1; 
                                              
                                              
    uint32 DisableClkoutCh1               :1; 
                                              
                                              
                                              
    uint32 BypenCh1                       :1; 
                                              
                                              
                                              
    uint32 ReservedForEco2                :8; 
    uint32 M1Div                          :8; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DsRelPllPllCh1;


typedef union {
  struct {
    uint32 ReservedForEco                 :10;
    uint32 DlyCh2                         :2; 
                                              
                                              
                                              
                                              
    uint32 EnCmlbuf2                      :1; 
                                              
                                              
    uint32 PwrdnCh2                       :1; 
                                              
                                              
    uint32 DisableClkoutCh2               :1; 
                                              
                                              
                                              
    uint32 BypenCh2                       :1; 
                                              
                                              
                                              
    uint32 ReservedForEco2                :8; 
    uint32 M2Div                          :8; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DsRelPllPllCh2;


typedef union {
  struct {
    uint32 ReservedForEco                 :10;
    uint32 DlyCh3                         :2; 
                                              
                                              
                                              
                                              
    uint32 EnCmlbuf3                      :1; 
                                              
                                              
    uint32 PwrdnCh3                       :1; 
                                              
                                              
    uint32 DisableClkoutCh3               :1; 
                                              
                                              
                                              
    uint32 BypenCh3                       :1; 
                                              
                                              
                                              
    uint32 ReservedForEco2                :8; 
    uint32 M3Div                          :8; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DsRelPllPllCh3;


typedef union {
  struct {
    uint32 ReservedForEco                 :7; 
    uint32 NdivInt                        :9; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 ReservedForEco2                :4; 
    uint32 P2div                          :4; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 ReservedForEco3                :4; 
    uint32 P1div                          :4; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DsRelPllPllDiv1;


typedef union {
  struct {
    uint32 ReservedForEco                 :8; 
    uint32 NdivFrac                       :24;
                                              
  } Bits;
  uint32 Reg32;
}  DsRelPllPllDiv2;


typedef union {
  struct {
    uint32 Plltest                        :1; 
                                              
                                              
    uint32 ReservedForEco                 :2; 
    uint32 NdivDitherMfb                  :1; 
                                              
                                              
    uint32 ReservedForEco2                :1; 
    uint32 NdivMode                       :3; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 ReservedForEco3                :2; 
    uint32 VcoRng                         :2; 
                                              
                                              
                                              
    uint32 TestSel                        :4; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 TestEn                         :1; 
                                              
                                              
                                              
                                              
    uint32 NdivPwrdn                      :1; 
                                              
                                              
    uint32 RefcompPwrdn                   :1; 
                                              
                                              
    uint32 Pwrdn                          :1; 
                                              
                                              
                                              
    uint32 ReservedForEco4                :2; 
    uint32 DisableClkout                  :1; 
                                              
                                              
                                              
    uint32 Inpsel                         :1; 
                                              
                                              
    uint32 ReservedForEco5                :2; 
    uint32 BypassSdmod                    :1; 
                                              
                                              
    uint32 Bypen                          :1; 
                                              
                                              
                                              
                                              
                                              
    uint32 ReservedForEco6                :2; 
    uint32 Dreset                         :1; 
                                              
                                              
                                              
    uint32 Areset                         :1; 
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DsRelPllPllMisc1;


typedef union {
  struct {
    uint32 ReservedForEco                 :26;
    uint32 PllCtrl3732                    :6; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DsRelPllPllMisc3;

typedef struct {
  DsRelPllPllDiv1                     Div1;                    
  DsRelPllPllDiv2                     Div2;                    
  DsRelPllPllCh1                      Ch1;                     
  DsRelPllPllCh2                      Ch2;                     
  DsRelPllPllCh3                      Ch3;                     
  DsRelPllPllMisc1                    Misc1;                   
  uint32                              Misc2;                   
  DsRelPllPllMisc3                    Misc3;                   
}  DsRelPllRegisters;

#endif 



