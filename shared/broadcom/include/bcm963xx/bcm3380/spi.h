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
//  Filename:         spi.h
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
#ifndef SPI_H__
#define SPI_H__



typedef union {
  struct {
    uint32 Reserved                       :9; 
    uint32 Msgtail                        :7; 
                                              
    uint32 Reserved2                      :9; 
    uint32 Rxtail                         :7; 
                                              
  } Bits;
  uint32 Reg32;
}  SpiSpiTail;


typedef union {
  struct {
    uint32 Reserved                       :3; 
    uint32 Mode                           :1; 
                                              
                                              
    uint32 OneByte                        :1; 
                                              
                                              
    uint32 ByteCnt                        :3; 
                                              
                                              
                                              
                                              
                                              
    uint32 Reserved2                      :1; 
    uint32 DeviceId                       :3; 
                                              
                                              
                                              
                                              
                                              
    uint32 Cmnd                           :4; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 Reserved3                      :3; 
    uint32 RxUnderflow                    :1; 
                                              
                                              
    uint32 TxOverflow                     :1; 
                                              
                                              
    uint32 TxUnderflow                    :1; 
                                              
                                              
    uint32 RxOverflow                     :1; 
                                              
                                              
    uint32 CmndDone                       :1; 
                                              
                                              
    uint32 Reserved4                      :3; 
    uint32 RxUnderflowMsk                 :1; 
    uint32 TxOverflowMsk                  :1; 
    uint32 TxUnderflowMsk                 :1; 
    uint32 RxOverflowMsk                  :1; 
    uint32 CmndDoneMsk                    :1; 
  } Bits;
  uint32 Reg32;
}  SpiSpiControl1;


typedef union {
  struct {
    uint32 Reserved                       :3; 
    uint32 RxUnder                        :1; 
    uint32 TxOver                         :1; 
    uint32 TxUnder                        :1; 
    uint32 RxOver                         :1; 
    uint32 CmndDone                       :1; 
    uint32 Reserved2                      :4; 
    uint32 SerialBusy                     :1; 
                                              
                                              
    uint32 CmndBusy                       :1; 
                                              
                                              
                                              
    uint32 RxEmpty                        :1; 
                                              
    uint32 Reserved3                      :1; 
    uint32 ByteSwap                       :1; 
                                              
                                              
                                              
    uint32 Reserved4                      :1; 
    uint32 SsOffTime                      :3; 
                                              
                                              
                                              
                                              
                                              
    uint32 SpiClkDiv                      :3; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 SpiFillByte                    :8; 
                                              
                                              
  } Bits;
  uint32 Reg32;
}  SpiSpiControl2;

typedef struct {
  uint8                               SPIMSGDATA[544];         
  uint8                               Pad0[0x1e0];
  uint8                               SPIRXDATA[544];          
  uint8                               Pad1[0xe0];
  SpiSpiControl1                      SpiControl1;             
  SpiSpiControl2                      SpiControl2;             
  SpiSpiTail                          Spitail;                 
}  SpiRegs;

#endif 



