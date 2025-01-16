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
//  Filename:         ddr_ctl.h
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
#ifndef DDR_CTL_H__
#define DDR_CTL_H__



typedef union {
  struct {
    uint32 Reserved                       :13;
    uint32 ArbMode                        :3; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 ArbTimer                       :16;
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlArb;


typedef union {
  struct {
    uint32 Reserved                       :10;
    uint32 Bnk02                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Bnk01                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Bnk00                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlBnk10;


typedef union {
  struct {
    uint32 Reserved                       :10;
    uint32 Bnk02                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Bnk01                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Bnk00                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlBnk32;


typedef union {
  struct {
    uint32 Reserved                       :15;
    uint32 ClksRefDisable                 :1; 
                                              
                                              
                                              
                                              
    uint32 ClksRefRate                    :8; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 Reserved2                      :8; 
  } Bits;
  uint32 Reg32;
}  DdrCtlClks;


typedef union {
  struct {
    uint32 Reserved                       :8; 
    uint32 CsMode                         :8; 
                                              
                                              
                                              
                                              
                                              
    uint32 Reserved2                      :10;
    uint32 CsIntlv0                       :6; 
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlCnfg;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 Col03                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Col02                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Col01                          :6; 
    uint32 Reserved4                      :2; 
    uint32 Col00                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlCol000;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 Col07                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Col06                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Col05                          :6; 
    uint32 Reserved4                      :2; 
    uint32 Col04                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlCol001;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 Col11                          :6; 
    uint32 Reserved2                      :10;
    uint32 Col09                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Col08                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlCol010;


typedef union {
  struct {
    uint32 Reserved                       :10;
    uint32 Col14                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Col13                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Col12                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlCol011;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 Col03                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Col02                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Col01                          :6; 
    uint32 Reserved4                      :2; 
    uint32 Col00                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlCol200;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 Col07                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Col06                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Col05                          :6; 
    uint32 Reserved4                      :2; 
    uint32 Col04                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlCol201;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 Col11                          :6; 
    uint32 Reserved2                      :10;
    uint32 Col09                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Col08                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlCol210;


typedef union {
  struct {
    uint32 Reserved                       :10;
    uint32 Col14                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Col13                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Col12                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlCol211;


typedef union {
  struct {
    uint32 Reserved                       :7; 
    uint32 Cs1End                         :9; 
                                              
    uint32 Reserved2                      :7; 
    uint32 Cs0End                         :9; 
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlCsend;


typedef union {
  struct {
    uint32 Reserved                       :7; 
    uint32 Cs1Start                       :9; 
                                              
    uint32 Reserved2                      :7; 
    uint32 Cs0Start                       :9; 
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlCsst;


typedef union {
  struct {
    uint32 Reserved                       :15;
    uint32 CtlCrcEnable                   :1; 
    uint32 CtlCrc                         :16;
  } Bits;
  uint32 Reg32;
}  DdrCtlCtlCrc;


typedef union {
  struct {
    uint32 Reserved                       :15;
    uint32 DramCmdReq                     :1; 
                                              
    uint32 Reserved2                      :10;
    uint32 DcmdMemCmdCs1                  :1; 
    uint32 DcmdMemCmdCs0                  :1; 
    uint32 DcmdMemCmd                     :4; 
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlDcmd;


typedef union {
  struct {
    uint32 Reserved                       :1; 
    uint32 DmodeModeData                  :15;
                                              
                                              
                                              
                                              
                                              
    uint32 Reserved2                      :1; 
    uint32 DmodeEModeData                 :15;
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlDmode0;


typedef union {
  struct {
    uint32 Reserved                       :20;
    uint32 DmodeDramSleep                 :1; 
                                              
                                              
                                              
                                              
                                              
                                              
    uint32 Dmode2tAddrCmd                 :1; 
                                              
                                              
                                              
                                              
    uint32 DmodePgPolicy                  :2; 
                                              
                                              
                                              
    uint32 Reserved2                      :2; 
    uint32 DmodeX32Mode                   :1; 
                                              
                                              
                                              
    uint32 Reserved3                      :1; 
    uint32 DmodeDramType                  :4; 
  } Bits;
  uint32 Reg32;
}  DdrCtlDmode1;


typedef union {
  struct {
    uint32 GcfgAliasDisabled              :1; 
    uint32 GcfgForceSeq                   :1; 
    uint32 Reserved                       :6; 
    uint32 GcfgMaxAge                     :4; 
    uint32 Reserved2                      :12;
    uint32 GcfgDramSize2                  :4; 
    uint32 GcfgDramSize1                  :4; 
  } Bits;
  uint32 Reg32;
}  DdrCtlGcfg;


typedef union {
  struct {
    uint32 Reserved                       :24;
    uint32 LbistDataPass                  :1; 
    uint32 LbistCtlPass                   :1; 
    uint32 LbistDataDone                  :1; 
    uint32 LbistCtlDone                   :1; 
    uint32 Reserved2                      :2; 
    uint32 LbistClkDisable                :1; 
    uint32 LbistEnable                    :1; 
  } Bits;
  uint32 Reg32;
}  DdrCtlLbistCfg;


typedef union {
  struct {
    uint32 Reserved                       :23;
    uint32 OdtCsOddOdtEn                  :1; 
    uint32 Cs1OdtWrCs1                    :1; 
                                              
    uint32 Cs1OdtWrCs0                    :1; 
                                              
    uint32 Cs1OdtRdCs1                    :1; 
                                              
    uint32 Cs1OdtRdCs0                    :1; 
                                              
    uint32 Cs0OdtWrCs1                    :1; 
    uint32 Cs0OdtWrCs0                    :1; 
    uint32 Cs0OdtRdCs1                    :1; 
    uint32 Cs0OdtRdCs0                    :1; 
  } Bits;
  uint32 Reg32;
}  DdrCtlOdt;


typedef union {
  struct {
    uint32 Reserved                       :11;
    uint32 PiDslMipsCtlHwCntrEn           :1; 
                                              
    uint32 PiDslMipsCtlCntrCyc            :4; 
                                              
    uint32 Reserved2                      :1; 
    uint32 PiDslMipsCtlPhCntrMax          :15;
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlPiDslMipsCtl;


typedef union {
  struct {
    uint32 Reserved                       :17;
    uint32 PiDslMipsStCout                :15;
  } Bits;
  uint32 Reg32;
}  DdrCtlPiDslMipsSt;


typedef union {
  struct {
    uint32 Reserved                       :11;
    uint32 PiDslPhyCtlHwCntrEn            :1; 
                                              
    uint32 PiDslPhyCtlCntrCyc             :4; 
                                              
    uint32 Reserved2                      :1; 
    uint32 PiDslPhyCtlPhCntrMax           :15;
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlPiDslPhyCtl;


typedef union {
  struct {
    uint32 Reserved                       :17;
    uint32 PiDslPhyStCout                 :15;
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlPiDslPhySt;


typedef union {
  struct {
    uint32 Reserved                       :9; 
    uint32 PiGcfMaskMonitor               :7; 
                                              
                                              
                                              
    uint32 Reserved2                      :13;
    uint32 PiGcfRegOverride               :1; 
                                              
    uint32 PiGcfPllLoadPh                 :1; 
    uint32 PiGcfCntrEn                    :1; 
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlPiGcf;


typedef union {
  struct {
    uint32 Reserved                       :11;
    uint32 PiMipsCtlHwCntrEn              :1; 
                                              
    uint32 PiMipsCtlCntrCyc               :4; 
                                              
    uint32 Reserved2                      :1; 
    uint32 PiMipsCtlPhCntrMax             :15;
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlPiMipsCtl;


typedef union {
  struct {
    uint32 Reserved                       :17;
    uint32 PiMipsStCout                   :15;
  } Bits;
  uint32 Reg32;
}  DdrCtlPiMipsSt;


typedef union {
  struct {
    uint32 Reserved                       :11;
    uint32 PiUbusCtlHwCntrEn              :1; 
    uint32 PiUbusCtlCntrCyc               :4; 
                                              
    uint32 Reserved2                      :1; 
    uint32 PiUbusCtlPhCntrMax             :15;
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlPiUbusCtl;


typedef union {
  struct {
    uint32 PiUbusSmplSamplingPeriod       :4; 
                                              
                                              
    uint32 PiUbusSmplSamplingInc          :1; 
    uint32 PiUbusSmplSampleN              :3; 
                                              
    uint32 Reserved                       :1; 
    uint32 PiUbusSmplSample               :7; 
                                              
    uint32 Reserved2                      :1; 
    uint32 PiUbusSmplMask                 :7; 
                                              
    uint32 PiUbusSmplUpdate               :1; 
                                              
                                              
    uint32 PiUbusSmplPiError              :1; 
    uint32 PiUbusSmplCount                :6; 
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlPiUbusSmpl;


typedef union {
  struct {
    uint32 Reserved                       :17;
    uint32 PiUbusStCout                   :15;
  } Bits;
  uint32 Reg32;
}  DdrCtlPiUbusSt;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 Row03                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Row02                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Row01                          :6; 
    uint32 Reserved4                      :2; 
    uint32 Row00                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlRow000;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 Row07                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Row06                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Row05                          :6; 
    uint32 Reserved4                      :2; 
    uint32 Row04                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlRow001;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 Row11                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Row10                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Row09                          :6; 
    uint32 Reserved4                      :2; 
    uint32 Row08                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlRow010;


typedef union {
  struct {
    uint32 Reserved                       :10;
    uint32 Row14                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Row13                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Row12                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlRow011;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 Row03                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Row02                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Row01                          :6; 
    uint32 Reserved4                      :2; 
    uint32 Row00                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlRow200;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 Row07                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Row06                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Row05                          :6; 
    uint32 Reserved4                      :2; 
    uint32 Row04                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlRow201;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 Row11                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Row10                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Row09                          :6; 
    uint32 Reserved4                      :2; 
    uint32 Row08                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlRow210;


typedef union {
  struct {
    uint32 Reserved                       :10;
    uint32 Row14                          :6; 
    uint32 Reserved2                      :2; 
    uint32 Row13                          :6; 
    uint32 Reserved3                      :2; 
    uint32 Row12                          :6; 
  } Bits;
  uint32 Reg32;
}  DdrCtlRow211;


typedef union {
  struct {
    uint32 Reserved                       :18;
    uint32 TestmodeLmbRepoutD             :2; 
    uint32 TestmodeUbusReqinD             :2; 
    uint32 TestmodeUbusReqinA             :2; 
    uint32 TestmodeUbusReqinH             :2; 
    uint32 TestmodeUbusRepoutD            :2; 
    uint32 TestmodeUbusRepoutA            :2; 
    uint32 TestmodeUbusRepoutH            :2; 
  } Bits;
  uint32 Reg32;
}  DdrCtlTestmode;


typedef union {
  struct {
    uint32 TestAddr                       :28;
    uint32 Reserved                       :4; 
  } Bits;
  uint32 Reg32;
}  DdrCtlTestAddr;


typedef union {
  struct {
    uint32 Reserved                       :16;
    uint32 TestAddrUpdt                   :16;
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlTestAddrUpdt;


typedef union {
  struct {
    uint32 Reserved                       :15;
    uint32 TestVictSweepEnable            :1; 
    uint32 TestVictEnable                 :1; 
    uint32 TestPrbsOrder                  :2; 
    uint32 TestDataMode                   :5; 
    uint32 TestAddrMode                   :2; 
    uint32 TestCmd                        :2; 
    uint32 Reserved2                      :1; 
    uint32 TestError                      :1; 
    uint32 TestDone                       :1; 
    uint32 TestEnable                     :1; 
  } Bits;
  uint32 Reg32;
}  DdrCtlTestCfg1;


typedef union {
  struct {
    uint32 Reserved                       :2; 
    uint32 TestCurrVictCnt                :6; 
                                              
                                              
                                              
                                              
    uint32 Reserved2                      :2; 
    uint32 TestVictCnt                    :6; 
                                              
                                              
    uint32 TestCount                      :16;
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlTestCount;


typedef union {
  struct {
    uint32 TestRcvCount                   :16;
                                              
                                              
    uint32 TestCurrCount                  :16;
                                              
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlTestCurrCount;


typedef union {
  struct {
    uint32 TestPattern1                   :16;
    uint32 TestPattern0                   :16;
  } Bits;
  uint32 Reg32;
}  DdrCtlTestPat;


typedef union {
  struct {
    uint32 Reserved                       :3; 
    uint32 Tim1TRCw                       :5; 
                                              
                                              
    uint32 Tim1TRrd                       :4; 
                                              
                                              
    uint32 Tim1TRp                        :4; 
    uint32 Tim1TWl                        :4; 
    uint32 Tim1TWr                        :3; 
    uint32 Reserved2                      :1; 
    uint32 Tim1TCl                        :4; 
    uint32 Tim1TRcd                       :4; 
  } Bits;
  uint32 Reg32;
}  DdrCtlTim10;


typedef union {
  struct {
    uint32 Reserved                       :1; 
    uint32 Tim1TR2r                       :1; 
                                              
                                              
                                              
    uint32 Tim1TR2w                       :2; 
                                              
                                              
                                              
                                              
    uint32 Tim1TW2r                       :2; 
                                              
                                              
                                              
                                              
    uint32 Tim1TFifo                      :2; 
                                              
    uint32 Tim1TRfc                       :8; 
    uint32 Reserved2                      :2; 
    uint32 Tim1TFaw                       :6; 
    uint32 Reserved3                      :3; 
    uint32 Tim1TRCr                       :5; 
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlTim11;


typedef union {
  struct {
    uint32 Reserved                       :22;
    uint32 Tim2TW2w                       :2; 
                                              
                                              
                                              
    uint32 Reserved2                      :1; 
    uint32 Tim2TRtp                       :3; 
    uint32 Tim2TAl                        :4; 
                                              
                                              
  } Bits;
  uint32 Reg32;
}  DdrCtlTim2;

typedef struct {
  DdrCtlCnfg                          Cnfg;                    
  DdrCtlCsst                          Csst;                    
  DdrCtlCsend                         Csend;                   
  uint8                               Pad0[0x4];
  DdrCtlRow000                        Row000;                  
  DdrCtlRow001                        Row001;                  
  DdrCtlRow010                        Row010;                  
  DdrCtlRow011                        Row011;                  
  uint8                               Pad1[0x10];
  DdrCtlRow200                        Row200;                  
  DdrCtlRow201                        Row201;                  
  DdrCtlRow210                        Row210;                  
  DdrCtlRow211                        Row211;                  
  uint8                               Pad2[0x10];
  DdrCtlCol000                        Col000;                  
  DdrCtlCol001                        Col001;                  
  DdrCtlCol010                        Col010;                  
  DdrCtlCol011                        Col011;                  
  uint8                               Pad3[0x10];
  DdrCtlCol200                        Col200;                  
  DdrCtlCol201                        Col201;                  
  DdrCtlCol210                        Col210;                  
  DdrCtlCol211                        Col211;                  
  uint8                               Pad4[0x10];
  DdrCtlBnk10                         Bnk10;                   
  DdrCtlBnk32                         Bnk32;                   
  uint8                               Pad5[0x68];
  DdrCtlDcmd                          Dcmd;                    
  DdrCtlDmode0                        Dmode0;                  
  DdrCtlDmode1                        Dmode1;                  
  DdrCtlClks                          Clks;                    
  DdrCtlOdt                           Odt;                     
  DdrCtlTim10                         Tim10;                   
  DdrCtlTim11                         Tim11;                   
  DdrCtlTim2                          Tim2;                    
  DdrCtlCtlCrc                        CtlCrc;                  
  uint32                              DoutCrc;                 
  uint32                              DinCrc;                  
  uint8                               Pad6[0x6d4];
  DdrCtlGcfg                          Gcfg;                    
  DdrCtlLbistCfg                      LbistCfg;                
  uint32                              LbistSeed;               
  DdrCtlArb                           Arb;                     
  DdrCtlPiGcf                         PiGcf;                   
  DdrCtlPiUbusCtl                     PiUbusCtl;               
  DdrCtlPiMipsCtl                     PiMipsCtl;               
  DdrCtlPiDslMipsCtl                  PiDslMipsCtl;            
  DdrCtlPiDslPhyCtl                   PiDslPhyCtl;             
  DdrCtlPiUbusSt                      PiUbusSt;                
  DdrCtlPiMipsSt                      PiMipsSt;                
  DdrCtlPiDslMipsSt                   PiDslMipsSt;             
  DdrCtlPiDslPhySt                    PiDslPhySt;              
  DdrCtlPiUbusSmpl                    PiUbusSmpl;              
  DdrCtlTestmode                      Testmode;                
  DdrCtlTestCfg1                      TestCfg1;                
  DdrCtlTestPat                       TestPat;                 
  DdrCtlTestCount                     TestCount;               
  DdrCtlTestCurrCount                 TestCurrCount;           
  DdrCtlTestAddrUpdt                  TestAddrUpdt;            
  DdrCtlTestAddr                      TestAddr;                
  uint32                              TestData0;               
  uint32                              TestData1;               
  uint32                              TestData2;               
  uint32                              TestData3;               
}  DdrCtlRegisters;

#endif 



