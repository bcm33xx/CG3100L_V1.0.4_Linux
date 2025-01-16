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
//  Filename:         utp.h
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
#ifndef UTP_H__
#define UTP_H__



typedef union {
  struct {
    uint32 Reserved                       :24;
    uint32 MaxCnt                         :8; 
  } Bits;
  uint32 Reg32;
}  UtpAckCelMaxCnt;


typedef union {
  struct {
    uint32 Reserved                       :4; 
    uint32 QueueListStartAddr             :12;
    uint32 Reserved2                      :16;
  } Bits;
  uint32 Reg32;
}  UtpAckCelQueueListConfig;


typedef union {
  struct {
    uint32 Reserved                       :21;
    uint32 ClusterId                      :11;
  } Bits;
  uint32 Reg32;
}  UtpClusterId;


typedef union {
  struct {
    uint32 Reserved                       :31;
    uint32 Delete                         :1; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  UtpDeleteTokenCommand;


typedef union {
  struct {
    uint32 Reserved                       :7; 
    uint32 Priority                       :1; 
    uint32 FlowId                         :4; 
    uint32 Reserved2                      :4; 
    uint32 TokenId                        :16;
  } Bits;
  uint32 Reg32;
}  UtpDeleteTokenConfig;


typedef union {
  struct {
    uint32 Reserved                       :12;
    uint32 SmisbError                     :1; 
    uint32 GcAvailSizeGtMax               :1; 
                                              
    uint32 TokenIdOutOfRange              :1; 
    uint32 InsertReplacedToken            :1; 
    uint32 Reserved2                      :7; 
    uint32 TamSharedWr                    :1; 
                                              
    uint32 TamSharedRd                    :1; 
                                              
    uint32 QListPtrRd                     :1; 
    uint32 QListConfigPtrRd               :1; 
                                              
    uint32 InsertDeletePtrWr              :1; 
                                              
    uint32 InsertDeletePtrRd              :1; 
                                              
    uint32 SegPtrWr                       :1; 
                                              
    uint32 SegPtrRd                       :1; 
                                              
    uint32 DmaPtrWr                       :1; 
                                              
  } Bits;
  uint32 Reg32;
}  UtpDiagCaptStopMask;


typedef union {
  struct {
    uint32 Reserved                       :31;
    uint32 Tbd                            :1; 
  } Bits;
  uint32 Reg32;
}  UtpDiagCntrlEn;


typedef union {
  struct {
    uint32 Reserved                       :12;
    uint32 SmisbError                     :1; 
    uint32 GcAvailSizeGtMax               :1; 
                                              
    uint32 TokenIdOutOfRange              :1; 
    uint32 InsertReplacedToken            :1; 
    uint32 Reserved2                      :7; 
    uint32 TamSharedWr                    :1; 
    uint32 TamSharedRd                    :1; 
    uint32 QListPtrRd                     :1; 
    uint32 QListConfigPtrRd               :1; 
    uint32 InsertDeletePtrWr              :1; 
                                              
    uint32 InsertDeletePtrRd              :1; 
                                              
    uint32 SegPtrWr                       :1; 
                                              
    uint32 SegPtrRd                       :1; 
                                              
    uint32 DmaPtrWr                       :1; 
  } Bits;
  uint32 Reg32;
}  UtpErrorIrqMask;


typedef union {
  struct {
    uint32 Reserved                       :12;
    uint32 SmisbError                     :1; 
                                              
    uint32 GcAvailSizeGtMax               :1; 
                                              
                                              
    uint32 TokenIdOutOfRange              :1; 
                                              
                                              
    uint32 InsertReplacedToken            :1; 
                                              
                                              
                                              
    uint32 Reserved2                      :7; 
    uint32 TamSharedWr                    :1; 
                                              
    uint32 TamSharedRd                    :1; 
                                              
    uint32 QListPtrRd                     :1; 
                                              
    uint32 QListConfigPtrRd               :1; 
                                              
    uint32 InsertDeletePtrWr              :1; 
                                              
                                              
    uint32 InsertDeletePtrRd              :1; 
                                              
                                              
    uint32 SegPtrWr                       :1; 
                                              
                                              
    uint32 SegPtrRd                       :1; 
                                              
                                              
    uint32 DmaPtrWr                       :1; 
                                              
  } Bits;
  uint32 Reg32;
}  UtpErrorIrqStatus;


typedef union {
  struct {
    uint32 DeletePtr                      :16;
    uint32 InsertPtr                      :16;
  } Bits;
  uint32 Reg32;
}  UtpHeadTail;


typedef union {
  struct {
    uint32 Reserved                       :16;
    uint32 AckCelDepth                    :16;
                                              
  } Bits;
  uint32 Reg32;
}  UtpInsertAckCelDepth;


typedef union {
  struct {
    uint32 ReplacedToken                  :6; 
    uint32 ReplacedTag                    :26;
  } Bits;
  uint32 Reg32;
}  UtpInsertReplacedTagAndCount;


typedef union {
  struct {
    uint32 Reserved                       :31;
    uint32 Insert                         :1; 
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  UtpInsertTokenCommand;


typedef union {
  struct {
    uint32 Reserved                       :7; 
    uint32 Priority                       :1; 
    uint32 FlowId                         :4; 
    uint32 Reserved2                      :20;
  } Bits;
  uint32 Reg32;
}  UtpInsertTokenConfig;


typedef union {
  struct {
    uint32 Reserved                       :28;
    uint32 Tbd                            :4; 
  } Bits;
  uint32 Reg32;
}  UtpMbistTm;


typedef union {
  struct {
    uint32 Enable                         :1; 
    uint32 Reserved                       :15;
    uint32 HighThreshold                  :8; 
    uint32 LowThreshold                   :8; 
  } Bits;
  uint32 Reg32;
}  UtpQiThreshold;


typedef union {
  struct {
    uint32 Reserved                       :31;
    uint32 Go                             :1; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  UtpQiTokenCommand;


typedef union {
  struct {
    uint32 Reserved                       :7; 
    uint32 Priority                       :1; 
    uint32 FlowId                         :4; 
    uint32 Reserved2                      :4; 
    uint32 TokenId                        :16;
  } Bits;
  uint32 Reg32;
}  UtpQiTokenConfig;


typedef union {
  struct {
    uint32 Reserved                       :16;
    uint32 AvailableSize                  :16;
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  UtpQueueAvailSize;


typedef union {
  struct {
    uint32 Reserved                       :4; 
    uint32 QueueListStartAddr             :12;
    uint32 Reserved2                      :10;
    uint32 AckCelQueue                    :6; 
  } Bits;
  uint32 Reg32;
}  UtpQueueListConfig;


typedef union {
  struct {
    uint32 Reserved                       :19;
    uint32 Size                           :13;
                                              
                                              
  } Bits;
  uint32 Reg32;
}  UtpQueueListSize;


typedef union {
  struct {
    uint32 MaxClear                       :1; 
    uint32 Reserved                       :7; 
    uint32 MaxTime                        :16;
                                              
    uint32 TimeOutValue                   :8; 
  } Bits;
  uint32 Reg32;
}  UtpSmisbMonitor;


typedef union {
  struct {
    uint32 Valid                          :1; 
                                              
    uint32 Reserved                       :1; 
    uint32 PoolNum                        :2; 
    uint32 Index                          :16;
    uint32 Size                           :12;
  } Bits;
  uint32 Reg32;
}  UtpToken;

typedef struct {
  UtpAckCelMaxCnt                     Queue16AckCelMax;        
  UtpAckCelMaxCnt                     Queue17AckCelMax;        
  UtpAckCelMaxCnt                     Queue18AckCelMax;        
  UtpAckCelMaxCnt                     Queue19AckCelMax;        
  UtpAckCelMaxCnt                     Queue20AckCelMax;        
  UtpAckCelMaxCnt                     Queue21AckCelMax;        
  UtpAckCelMaxCnt                     Queue22AckCelMax;        
  UtpAckCelMaxCnt                     Queue23AckCelMax;        
  UtpAckCelMaxCnt                     Queue24AckCelMax;        
  UtpAckCelMaxCnt                     Queue25AckCelMax;        
  UtpAckCelMaxCnt                     Queue26AckCelMax;        
  UtpAckCelMaxCnt                     Queue27AckCelMax;        
  UtpAckCelMaxCnt                     Queue28AckCelMax;        
  UtpAckCelMaxCnt                     Queue29AckCelMax;        
  UtpAckCelMaxCnt                     Queue30AckCelMax;        
  UtpAckCelMaxCnt                     Queue31AckCelMax;        
}  UtpUtpAckCelMaxCntRegs;

typedef struct {
  UtpHeadTail                         QueueList0HeadTail;      
  UtpHeadTail                         QueueList1HeadTail;      
  UtpHeadTail                         QueueList2HeadTail;      
  UtpHeadTail                         QueueList3HeadTail;      
  UtpHeadTail                         QueueList4HeadTail;      
  UtpHeadTail                         QueueList5HeadTail;      
  UtpHeadTail                         QueueList6HeadTail;      
  UtpHeadTail                         QueueList7HeadTail;      
  UtpHeadTail                         QueueList8HeadTail;      
  UtpHeadTail                         QueueList9HeadTail;      
  UtpHeadTail                         QueueList10HeadTail;     
  UtpHeadTail                         QueueList11HeadTail;     
  UtpHeadTail                         QueueList12HeadTail;     
  UtpHeadTail                         QueueList13HeadTail;     
  UtpHeadTail                         QueueList14HeadTail;     
  UtpHeadTail                         QueueList15HeadTail;     
  UtpHeadTail                         QueueList16HeadTail;     
  UtpHeadTail                         QueueList17HeadTail;     
  UtpHeadTail                         QueueList18HeadTail;     
  UtpHeadTail                         QueueList19HeadTail;     
  UtpHeadTail                         QueueList20HeadTail;     
  UtpHeadTail                         QueueList21HeadTail;     
  UtpHeadTail                         QueueList22HeadTail;     
  UtpHeadTail                         QueueList23HeadTail;     
  UtpHeadTail                         QueueList24HeadTail;     
  UtpHeadTail                         QueueList25HeadTail;     
  UtpHeadTail                         QueueList26HeadTail;     
  UtpHeadTail                         QueueList27HeadTail;     
  UtpHeadTail                         QueueList28HeadTail;     
  UtpHeadTail                         QueueList29HeadTail;     
  UtpHeadTail                         QueueList30HeadTail;     
  UtpHeadTail                         QueueList31HeadTail;     
}  UtpUtpHeadTailRegs;

typedef struct {
  uint32                              TokensInserted;          
  uint32                              TokensDeleted;           
  uint32                              InsertMaxCycles;         
  uint32                              DeleteMaxCycles;         
  uint32                              InsertConflicts;         
  uint32                              SegConflicts;            
                                                                 
  UtpDiagCntrlEn                      DiagCntrlEn;             
  uint32                              DiagCntrl;               
  uint32                              DiagA;                   
  uint32                              DiagB;                   
  UtpErrorIrqStatus                   ErrorIrqStatus;          
  UtpErrorIrqMask                     ErrorIrqMask;            
  UtpMbistTm                          MBistTm;                 
  UtpSmisbMonitor                     SmisbMonitor;            
  UtpDiagCaptStopMask                 DiagStopMask;            
}  UtpUtpMiscRegs;

typedef struct {
  UtpClusterId                        ClusterID[4096];         
}  UtpUtpQListMem;

typedef struct {
  UtpQiThreshold                      Queue16Threshold;        
  UtpQiThreshold                      Queue17Threshold;        
  UtpQiThreshold                      Queue18Threshold;        
  UtpQiThreshold                      Queue19Threshold;        
  UtpQiThreshold                      Queue20Threshold;        
  UtpQiThreshold                      Queue21Threshold;        
  UtpQiThreshold                      Queue22Threshold;        
  UtpQiThreshold                      Queue23Threshold;        
  UtpQiThreshold                      Queue24Threshold;        
  UtpQiThreshold                      Queue25Threshold;        
  UtpQiThreshold                      Queue26Threshold;        
  UtpQiThreshold                      Queue27Threshold;        
  UtpQiThreshold                      Queue28Threshold;        
  UtpQiThreshold                      Queue29Threshold;        
  UtpQiThreshold                      Queue30Threshold;        
  UtpQiThreshold                      Queue31Threshold;        
}  UtpUtpQiThresholdRegs;

typedef struct {
  UtpQueueAvailSize                   QueueAvailSize[32];      
}  UtpUtpQueueAvailSizeRegs;

typedef struct {
  UtpQueueListConfig                  QueueList0Config;        
  UtpQueueListSize                    QueueList0Size;          
  UtpQueueListConfig                  QueueList1Config;        
  UtpQueueListSize                    QueueList1Size;          
  UtpQueueListConfig                  QueueList2Config;        
  UtpQueueListSize                    QueueList2Size;          
  UtpQueueListConfig                  QueueList3Config;        
  UtpQueueListSize                    QueueList3Size;          
  UtpQueueListConfig                  QueueList4Config;        
  UtpQueueListSize                    QueueList4Size;          
  UtpQueueListConfig                  QueueList5Config;        
  UtpQueueListSize                    QueueList5Size;          
  UtpQueueListConfig                  QueueList6Config;        
  UtpQueueListSize                    QueueList6Size;          
  UtpQueueListConfig                  QueueList7Config;        
  UtpQueueListSize                    QueueList7Size;          
  UtpQueueListConfig                  QueueList8Config;        
  UtpQueueListSize                    QueueList8Size;          
  UtpQueueListConfig                  QueueList9Config;        
  UtpQueueListSize                    QueueList9Size;          
  UtpQueueListConfig                  QueueList10Config;       
  UtpQueueListSize                    QueueList10Size;         
  UtpQueueListConfig                  QueueList11Config;       
  UtpQueueListSize                    QueueList11Size;         
  UtpQueueListConfig                  QueueList12Config;       
  UtpQueueListSize                    QueueList12Size;         
  UtpQueueListConfig                  QueueList13Config;       
  UtpQueueListSize                    QueueList13Size;         
  UtpQueueListConfig                  QueueList14Config;       
  UtpQueueListSize                    QueueList14Size;         
  UtpQueueListConfig                  QueueList15Config;       
  UtpQueueListSize                    QueueList15Size;         
  UtpQueueListConfig                  QueueList16Config;       
  UtpQueueListSize                    QueueList16Size;         
  UtpQueueListConfig                  QueueList17Config;       
  UtpQueueListSize                    QueueList17Size;         
  UtpQueueListConfig                  QueueList18Config;       
  UtpQueueListSize                    QueueList18Size;         
  UtpQueueListConfig                  QueueList19Config;       
  UtpQueueListSize                    QueueList19Size;         
  UtpQueueListConfig                  QueueList20Config;       
  UtpQueueListSize                    QueueList20Size;         
  UtpQueueListConfig                  QueueList21Config;       
  UtpQueueListSize                    QueueList21Size;         
  UtpQueueListConfig                  QueueList22Config;       
  UtpQueueListSize                    QueueList22Size;         
  UtpQueueListConfig                  QueueList23Config;       
  UtpQueueListSize                    QueueList23Size;         
  UtpQueueListConfig                  QueueList24Config;       
  UtpQueueListSize                    QueueList24Size;         
  UtpQueueListConfig                  QueueList25Config;       
  UtpQueueListSize                    QueueList25Size;         
  UtpQueueListConfig                  QueueList26Config;       
  UtpQueueListSize                    QueueList26Size;         
  UtpQueueListConfig                  QueueList27Config;       
  UtpQueueListSize                    QueueList27Size;         
  UtpQueueListConfig                  QueueList28Config;       
  UtpQueueListSize                    QueueList28Size;         
  UtpQueueListConfig                  QueueList29Config;       
  UtpQueueListSize                    QueueList29Size;         
  UtpQueueListConfig                  QueueList30Config;       
  UtpQueueListSize                    QueueList30Size;         
  UtpQueueListConfig                  QueueList31Config;       
  UtpQueueListSize                    QueueList31Size;         
  UtpAckCelQueueListConfig            QueueList32Config;       
  UtpQueueListSize                    QueueList32Size;         
  UtpAckCelQueueListConfig            QueueList33Config;       
  UtpQueueListSize                    QueueList33Size;         
  UtpAckCelQueueListConfig            QueueList34Config;       
  UtpQueueListSize                    QueueList34Size;         
  UtpAckCelQueueListConfig            QueueList35Config;       
  UtpQueueListSize                    QueueList35Size;         
}  UtpUtpQueueListConfigRegs;

typedef struct {
  UtpDeleteTokenConfig                DeleteTokenConfig;       
  UtpDeleteTokenCommand               DeleteTokenCommand;      
  uint32                              DeleteTokenToken;        
}  UtpUtpTokenDeletionRegs;

typedef struct {
  UtpInsertTokenConfig                InsertTokenConfig;       
  UtpInsertTokenCommand               InsertTokenCommand;      
  uint32                              InsertTokenToken;        
  uint32                              InsertTokenAckCelTag;    
  uint32                              InsertReplacedToken;     
  UtpInsertReplacedTagAndCount        InsertReplacedTagAndCount;
  UtpInsertAckCelDepth                AckCelDepth;             
}  UtpUtpTokenInsertionRegs;

typedef struct {
  UtpToken                            Token[16384];            
}  UtpUtpTokenMem;

typedef struct {
  UtpQiTokenConfig                    QiTokenConfig;           
  UtpQiTokenCommand                   QiTokenCommand;          
  uint32                              QiPhyAddr;               
}  UtpUtpTokenQiRegs;

  
typedef struct {
  UtpUtpQListMem                 QListMem;
} UtpQlistmeMxx;

  
typedef struct {
  UtpUtpQueueListConfigRegs      QueueListConfig;
  uint8                          Pad0[0xee0];
} UtpQueueListConfigBlockdef;

  
typedef struct {
  UtpUtpHeadTailRegs             QueueListHeadTail;
} UtpQueueListHeadTailBlockdef;

  
typedef struct {
  UtpUtpTokenInsertionRegs       TokenInsertion;
  uint8                          Pad0[0x4];
  UtpUtpTokenDeletionRegs        TokenDeletion;
  uint8                          Pad1[0x14];
  UtpUtpQueueAvailSizeRegs       QueueAvailSize;
  UtpUtpMiscRegs                 TamMisc;
  uint8                          Pad2[0xf04];
} UtpTokenCommandBlockdef;

  
typedef struct {
  UtpUtpQiThresholdRegs          QiThreshold;
} UtpQueueQiThresholdBlockdef;

  
typedef struct {
  UtpUtpAckCelMaxCntRegs         AckCelMaxCnt;
} UtpQueueAckCelBlockdef;

  
typedef struct {
  UtpUtpTokenMem                 TokenMem;
} UtpTokenRamBlockdef;

  
typedef struct {
  union {
    UtpQlistmeMxx                 Qlistmem;
    struct {
      uint8                      Pad0[0x5000];
      UtpQueueListConfigBlockdef Qlistconfig;
    };
    struct {
      uint8                      Pad1[0x5200];
      UtpQueueListHeadTailBlockdef Qlistheadtail;
    };
    struct {
      uint8                      Pad2[0x5300];
      UtpTokenCommandBlockdef    Tokencommand;
    };
    struct {
      uint8                      Pad3[0x5400];
      UtpQueueQiThresholdBlockdef Qithreshold;
    };
    struct {
      uint8                      Pad4[0x5500];
      UtpQueueAckCelBlockdef     Ackcel;
    };
    struct {
      uint8                      Pad5[0x10000];
      UtpTokenRamBlockdef        Tokenram;
    };
  };
}  UtpTamBlock;

#endif 



