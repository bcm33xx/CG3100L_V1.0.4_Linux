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
//  Filename:         crypto.h
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
#ifndef CRYPTO_H__
#define CRYPTO_H__



typedef union {
  struct {
    uint32 Reserved                       :10;
    uint32 Stat5Sel                       :1; 
                                              
                                              
    uint32 Tmb1                           :4; 
    uint32 Tma1                           :4; 
    uint32 Tmb0                           :4; 
    uint32 Tma0                           :4; 
    uint32 RamEnable4096                  :1; 
                                              
    uint32 RamEnable1024                  :1; 
                                              
    uint32 IrqTestEnable                  :1; 
                                              
                                              
    uint32 OverwriteSource                :1; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 Reserved2                      :1; 
  } Bits;
  uint32 Reg32;
}  CryptoCryptoDebug1;


typedef union {
  struct {
    uint32 CmdState                       :3; 
    uint32 TokenState                     :4; 
    uint32 HdrState                       :4; 
    uint32 ReplyState                     :4; 
    uint32 ArbState                       :4; 
    uint32 TxpState                       :4; 
    uint32 RxpState                       :4; 
    uint32 CrpState                       :5; 
  } Bits;
  uint32 Reg32;
}  CryptoCryptoStat1;


typedef union {
  struct {
    uint32 AesState                       :3; 
    uint32 DesState                       :4; 
    uint32 PhsState                       :4; 
    uint32 Reserved                       :11;
    uint32 AesFifoErrFull                 :1; 
    uint32 AesFifoErrEmpty                :1; 
    uint32 DesFifoErrFull                 :1; 
    uint32 DesFifoErrEmpty                :1; 
    uint32 CmdFifoErrFull                 :1; 
    uint32 CmdFifoErrEmpty                :1; 
    uint32 RxFifoErrFull                  :1; 
    uint32 RxFifoErrEmpty                 :1; 
    uint32 TxFifoErrFull                  :1; 
    uint32 TxFifoErrEmpty                 :1; 
  } Bits;
  uint32 Reg32;
}  CryptoCryptoStat2;


typedef union {
  struct {
    uint32 Reserved                       :11;
    uint32 UnsupportedModeErr             :1; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 ReqMsgLenInvalid               :1; 
                                              
    uint32 CmdFifoFull                    :1; 
                                              
    uint32 WatchdogTimeout                :1; 
                                              
                                              
                                              
    uint32 RepinErrorSeen                 :1; 
                                              
    uint32 UsppBusy2utp                   :1; 
                                              
    uint32 InvalidReqMsgId                :1; 
                                              
    uint32 ReqMsgIdRcvd                   :6; 
                                              
    uint32 ReplyComplete                  :1; 
    uint32 TxpDone                        :1; 
    uint32 DmaStartPulse                  :1; 
                                              
    uint32 InvalidSrcToken                :1; 
                                              
    uint32 InvalidDstToken                :1; 
                                              
    uint32 StartDoneMismatch              :1; 
                                              
    uint32 InvalidHdrLen                  :1; 
                                              
    uint32 HdrOffsetZero                  :1; 
                                              
                                              
  } Bits;
  uint32 Reg32;
}  CryptoCryptoStat3;


typedef union {
  struct {
    uint32 ReplyPktCount                  :16;
    uint32 StartPktCount                  :16;
  } Bits;
  uint32 Reg32;
}  CryptoCryptoStat4;


typedef union {
  struct {
    uint32 Reserved                       :8; 
    uint32 CurrRxByteCnt                  :12;
                                              
                                              
                                              
                                              
    uint32 CurrTxByteCnt                  :12;
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  CryptoCryptoStat5;


typedef union {
  struct {
    uint32 CmdFifoThreshold               :8; 
                                              
                                              
    uint32 Reserved                       :1; 
    uint32 ReqMsgFilter                   :1; 
                                              
                                              
                                              
                                              
                                              
    uint32 RespMsgId                      :6; 
                                              
                                              
                                              
    uint32 ReqMsgIdSetting                :6; 
                                              
    uint32 BuffSize                       :3; 
                                              
    uint32 Reserved2                      :4; 
    uint32 RxFifoFlush                    :1; 
                                              
    uint32 TxFifoFlush                    :1; 
                                              
    uint32 Reserved3                      :1; 
  } Bits;
  uint32 Reg32;
}  CryptoCtl1;


typedef union {
  struct {
    uint32 ErrResponeMsgId                :6; 
                                              
                                              
                                              
    uint32 Reserved                       :18;
    uint32 EnableAutoNorm                 :1; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 EnableAutoErr                  :1; 
                                              
                                              
                                              
    uint32 ForceReply                     :1; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 Reserved2                      :1; 
    uint32 ClearCmdFifo                   :1; 
                                              
                                              
                                              
                                              
    uint32 ClearCurrentPkt                :1; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 ResetStatus                    :1; 
                                              
                                              
    uint32 PauseCrypto                    :1; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  CryptoErrorResponse;


typedef union {
  struct {
    uint32 FutureInt1                     :22;
    uint32 AesFifoErrEmpty                :1; 
                                              
    uint32 AesFifoErrFull                 :1; 
                                              
    uint32 DesFifoErrEmpty                :1; 
                                              
    uint32 DesFifoErrFull                 :1; 
                                              
    uint32 CmdFifoErrFull                 :1; 
                                              
    uint32 CmdFifoErrEmpty                :1; 
                                              
    uint32 RxFifoErrFull                  :1; 
                                              
    uint32 RxFifoErrEmpty                 :1; 
                                              
    uint32 TxFifoErrFull                  :1; 
                                              
    uint32 TxFifoErrEmpty                 :1; 
                                              
  } Bits;
  uint32 Reg32;
}  CryptoIntStatus1;


typedef union {
  struct {
    uint32 FutureInt2                     :8; 
    uint32 CryptoBusy                     :1; 
                                              
                                              
                                              
    uint32 CmdFifoReadErr                 :1; 
                                              
                                              
    uint32 CmdFifoAddrErr                 :1; 
                                              
                                              
                                              
                                              
    uint32 UnsupportedModeErr             :1; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 ReqMsgLenInvalid               :1; 
                                              
                                              
                                              
                                              
    uint32 CmdFifoFull                    :1; 
                                              
                                              
                                              
    uint32 WatchDogTimeout                :1; 
                                              
    uint32 RepinErrorSeen                 :1; 
                                              
                                              
    uint32 UsppBusy2utp                   :1; 
                                              
                                              
    uint32 InvalidReqMsgId                :1; 
                                              
                                              
    uint32 NotUsed                        :6; 
    uint32 ReplyComplete                  :1; 
                                              
    uint32 TxpDone                        :1; 
                                              
    uint32 DmaStartPulse                  :1; 
                                              
                                              
    uint32 InvalidSrcToken                :1; 
                                              
                                              
    uint32 InvalidDstToken                :1; 
                                              
                                              
    uint32 StartDoneMismatch              :1; 
                                              
                                              
    uint32 InvalidHdrLen                  :1; 
                                              
                                              
    uint32 HdrOffsetZero                  :1; 
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  CryptoIntStatus2;


typedef union {
  struct {
    uint32 Reserved                       :30;
    uint32 OddKeyValid                    :1; 
                                              
                                              
                                              
                                              
    uint32 EvenKeyValid                   :1; 
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  CryptoKeyCfg1;


typedef union {
  struct {
    uint32 Reserved                       :10;
    uint32 PhsMaskIo                      :1; 
                                              
                                              
                                              
    uint32 PhsMaskValid                   :1; 
                                              
                                              
    uint32 PhsMaskLen                     :4; 
                                              
                                              
                                              
                                              
    uint32 PhsIndex                       :16;
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  CryptoPhsIndexTable;


typedef union {
  struct {
    uint32 Reserved                       :20;
    uint32 UtpPid                         :4; 
    uint32 DdrPid                         :4; 
                                              
    uint32 FpmPid                         :4; 
  } Bits;
  uint32 Reg32;
}  CryptoPidCfg1;


typedef union {
  struct {
    uint32 Reserved                       :16;
    uint32 CryptoRevision                 :16;
  } Bits;
  uint32 Reg32;
}  CryptoRevisionReg;


typedef union {
  struct {
    uint32 FpmReady                       :1; 
                                              
                                              
                                              
                                              
    uint32 Reserved                       :15;
    uint32 AutoFlushOnInvalidSrc          :1; 
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 Reserved2                      :1; 
    uint32 InvalidSrcTknId                :6; 
                                              
                                              
                                              
                                              
    uint32 AutoFlushOnInvalidDst          :1; 
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 Reserved3                      :1; 
    uint32 InvalidDstTknId                :6; 
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  CryptoTknCfg1;


typedef union {
  struct {
    uint32 Reserved                       :26;
    uint32 MaxUbusSize                    :6; 
  } Bits;
  uint32 Reg32;
}  CryptoUbusCtl1;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid0;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid1;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid10;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid11;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid12;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid13;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid14;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid15;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid2;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid3;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid4;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid5;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid6;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid7;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid8;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 UsSid                          :14;
  } Bits;
  uint32 Reg32;
}  CryptoUsSid9;

typedef struct {
  CryptoUbusCtl1                      UbusCtl1;                
  CryptoCtl1                          CrpCtl1;                 
  uint32                              WatchDogSetting;         
  CryptoErrorResponse                 ErrorResponse;           
  uint32                              ReqMsg00;                
  uint32                              ReqMsg04;                
  uint32                              ReqMsg08;                
  uint32                              ReqMsg0c;                
  uint32                              ReqMsg10;                
  CryptoCryptoStat5                   CryptoStat5;             
  CryptoIntStatus1                    IntStatus1;              
  CryptoIntStatus2                    IntStatus2;              
  uint32                              IntMask1;                
  uint32                              IntMask2;                
  uint32                              FpmAddr;                 
  uint32                              ReplyAddr;               
  uint8                               Pad0[0x4];
  uint32                              BuffBase;                
  uint32                              NextNewToken;            
  uint8                               Pad1[0x4];
  CryptoCryptoStat1                   CryptoStat1;             
  CryptoCryptoStat2                   CryptoStat2;             
  CryptoCryptoStat3                   CryptoStat3;             
  CryptoCryptoStat4                   CryptoStat4;             
  CryptoCryptoDebug1                  CryptoDebug1;            
  CryptoPidCfg1                       PidCfg1;                 
  CryptoTknCfg1                       TknCfg1;                 
  CryptoKeyCfg1                       KeyCfg1;                 
  uint32                              UsEvenKeyA;              
  uint32                              UsEvenKeyB;              
  uint32                              UsEvenKeyC;              
  uint32                              UsEvenKeyD;              
  uint32                              UsOddKeyA;               
  uint32                              UsOddKeyB;               
  uint32                              UsOddKeyC;               
  uint32                              UsOddKeyD;               
  uint32                              UsEvenIvA;               
  uint32                              UsEvenIvB;               
  uint32                              UsEvenIvC;               
  uint32                              UsEvenIvD;               
  uint32                              UsOddIvA;                
  uint32                              UsOddIvB;                
  uint32                              UsOddIvC;                
  uint32                              UsOddIvD;                
  CryptoUsSid0                        UsSid0;                  
  CryptoUsSid1                        UsSid1;                  
  CryptoUsSid2                        UsSid2;                  
  CryptoUsSid3                        UsSid3;                  
  CryptoUsSid4                        UsSid4;                  
  CryptoUsSid5                        UsSid5;                  
  CryptoUsSid6                        UsSid6;                  
  CryptoUsSid7                        UsSid7;                  
  CryptoUsSid8                        UsSid8;                  
  CryptoUsSid9                        UsSid9;                  
  CryptoUsSid10                       UsSid10;                 
  CryptoUsSid11                       UsSid11;                 
  CryptoUsSid12                       UsSid12;                 
  CryptoUsSid13                       UsSid13;                 
  CryptoUsSid14                       UsSid14;                 
  CryptoUsSid15                       UsSid15;                 
  uint32                              PhsMaskExtBase;          
  uint32                              PhsMask1;                
  uint32                              PhsMask2;                
  uint32                              PhsMask3;                
  uint32                              PhsMask4;                
  uint32                              PhsMask5;                
  uint32                              PhsMask6;                
  uint32                              PhsMask7;                
  uint32                              PhsMask8;                
  CryptoRevisionReg                   RevisionReg;             
  uint8                               Pad2[0x3ee8];
  CryptoPhsIndexTable                 PhsIndexTable;           
  uint8                               Pad3[0x3ffc];
  uint32                              PhsMaskTable;            
}  CryptoRegs;

#endif 



