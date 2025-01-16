//****************************************************************************
//
// (c) 2008 Broadcom Corporation
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
//****************************************************************************
//
//  Filename:       InOutMsgFifo.h
//  Author:         La Long/Mark Newcomer
//  Creation Date:  <date>
//
//  Description: Contains defines and typedefs used by functions that drive
//                 the Out Going Message Fifo in the IO Processor
//
//****************************************************************************

#include <bcmtypes.h>

#ifndef INOUTMSGFIFO_H
#define INOUTMSGFIFO_H

// Token Defines
#define TKN_VALID_TKN_BIT       0x80000000
#define TKN_VALID_TKN_SHIFT     31
#define TKN_RSVD_BIT            0x40000000
#define TKN_RSVD_SHIFT          30
#define TKN_BUF_INDX_MASK       0x3FFFF000
#define TKN_BUF_INDX_SHIFT      12
#define TKN_BUF_POOL_NUM_MASK   0x30000000
#define TKN_BUF_POOL_NUM_SHIFT  28
#define TKN_BUF_POOL_INDX_MASK  0x0FFFF000
#define TKN_BUF_POOL_INDX_SHIFT 12
#define TKN_SIZE_MASK           0x00000FFF
#define TKN_SIZE_SHIFT          0

//Data Message Defines.
enum MsgIds
{
    // Start of Incoming/Outgoing MsgIds
    LANRxMsg=0,
    LANTxMsg,
    TXStatusMsg,
    DSPktMsg,
    DSBpiSeqErrMsg,
    DSPhsDiscardMsg,
    USPPRequestMsg,
    USPPResponseMsg,
    USReleaseTokenMsg,
   // Start of DQM MsgIds
    CREATEFlowCmdMsg=32,
    REMOVEFlowCmdMsg,
    CREATEFlowRspMsg,
    REMOVEFlowRspMsg,
    USPacketMsg,
    SHOWMibsCmdMsg,
    CLEARMibsCmdMsg,
    FLUSHFlowCmdMsg,
    TESTModeCmdMsg,
    // Start of test MsgIds
    INSERTTokenMsg=58,
    FLOWNotInserviceMsg=59,
    TESTStart=60,
    TESTStop,
    TESTEndPass,
    TESTEndFail
};

//Data Types.
//General structure of the message sent to the fifo.
//The data portion will hold the actual message body.
typedef struct MsgStruct
{
    enum MsgIds msgId;       //Will be or'ed in to first word. 6 MSB of first word.
    uint32      msgLen;      //In 32 bit words.
    uint32      destAddr;    //UBUS destination address if use for outgoing msg fifo
                             //UBUS source address if use for incoming msg fifo
                             //DQM queue number(0-31) if use for DQM
    uint32      msgData[16]; //Provides space for 16 words including headers.
} MsgStruct;

typedef struct MsgStats_s
{
    uint32 srcTknCnt;
    uint32 dstTknCnt;
    uint32 srcRlsCnt;
    uint32 dstRlsCnt;
} MsgStats_s;

typedef struct Msg_s
{
    enum MsgIds msgId;      //Will be or'ed in to first word. 6 MSB of first word.
    uint16      dqmQueue;   //DQM queue number(0-31), for FPGA only(0-5)
    uint32      msgLen;     //In 32 bit words.
    uint32      ubusAddr;   //UBUS destination address if use for outgoing msg fifo
                            //UBUS source address if use for incoming msg fifo
    uint32    * msgData;    //pointer to data buffer
    MsgStats_s *msgStats;
} Msg_s;

//Message ID, READ and WRITE macro.
#define ID_MASK              0xfc000000
#define ID_SHIFT             26
#define FLOW_MASK            0x01F00000
#define FLOW_SHIFT           20
#define WRITE_HDR_FIELD(msg,val,mask,shift)  (msg = ((val<<shift) & mask) | (msg & ~mask))
#define GET_MSG_ID(msg)                      (((msg&ID_MASK) >> ID_SHIFT))
#define READ_HDR_FIELD(msg,val,mask,shift)   (val = ((msg & mask)>>shift))
#define COMPARE_FIELD(msg, val, mask, shift) (((val << shift) & mask) != (msg & mask))

//LAN RX Message
//Producer  LAN MAC DMA
//Consumer  RX Message Processor (4ke)
typedef struct LanRxMsg
{
    uint32 msgHdr;
        #define LANRX_MAC_ID_MASK     0x03C00000
        #define LANRX_MAC_ID_SHIFT    22
        #define LANRX_QOS_MASK        0x000f0000
        #define LANRX_QOS_SHIFT       16
    uint32 token;
} LanRxMsg;

//LAN TX Message
//Producer  TX Message Processor
//Consumer  LAN MAC TX DMA
typedef struct LanTxMsg
{
    uint32 msgHdr;
        #define LANTX_MAC_ID_MASK     0x03c00000
        #define LANTX_MAC_ID_SHIFT    22
        #define LANTX_REL_TKN_MASK    0x00200000
        #define LANTX_REL_TKN_SHIFT   21
        #define LANTX_STS_REQ_BIT     0x00100000
        #define LANTX_STS_REQ_SHIFT   20
        #define LANTX_QOS_MASK        0x000f0000
        #define LANTX_QOS_SHIFT       16
        #define LANTX_EOP_MASK        0x00008000
        #define LANTX_EOP_SHIFT       15
    uint32 token;
} LanTxMsg;

//TX STATUS Message
//Producer  LAN MAC TX DMA
//Consumer  TX Message Processor
typedef struct TxStatusMsg
{
    uint32 msgHdr;
        #define TXSTS_MAC_ID_MASK     0x03c00000
        #define TXSTS_MAC_ID_SHIFT    22
		#define TXSTS_TX_QD_MASK      0x003f0000
		#define TXSTS_TX_QD_SHIFT     16
        #define TXSTS_TX_STS_MASK     0x000007ff
        #define TXSTS_STS_SHIFT       0
    uint32 token;
} TxStatusMsg;

//DS Packet Message
//Producer  DS MAC DMA
//Consumer  DS Token Processor
typedef struct DsPktMsg
{
    uint32 msgHdr;
        #define DSPKT_DS_CHAN_SHIFT     20
        #define DSPKT_DS_CHAN_MASK      (0x3f << DSPKT_DS_CHAN_SHIFT)
        #define DSPKT_QOS_TAG_SHIFT     16
        #define DSPKT_QOS_TAG_MASK      (0x0f << DSPKT_QOS_TAG_SHIFT)
        #define DSPKT_PKT_SEQ_SHIFT     0
        #define DSPKT_PKT_SEQ_MASK      (0xffff << DSPKT_PKT_SEQ_SHIFT)
    uint32 token;
    uint32 dsid;
        #define DSPKT_DSID_RSVD_SHIFT   29
        #define DSPKT_DSID_RSVD_MASK    (0x07 << DSPKT_DSID_RSVD_SHIFT)
        #define DSPKT_MAC_MSG_SHIFT     28
        #define DSPKT_MAC_MSG_MASK      (0x01 << DSPKT_MAC_MSG_SHIFT)
        #define DSPKT_PHSR_INVLD_SHIFT  27
        #define DSPKT_PHSR_INVLD_MASK   (0x01 << DSPKT_PHSR_INVLD_SHIFT)
        #define DSPKT_DSID_VLD_SHIFT    26
        #define DSPKT_DSID_VLD_MASK     (0x01 << DSPKT_DSID_VLD_SHIFT)
        #define DSPKT_PSN_VLD_SHIFT     25
        #define DSPKT_PSN_VLD_MASK      (0x01 << DSPKT_PSN_VLD_SHIFT)
        #define DSPKT_PRI_VLD_SHIFT     24
        #define DSPKT_PRI_VLD_MASK      (0x01 << DSPKT_PRI_VLD_SHIFT)
        #define DSPKT_TRF_PRI_SHIFT     21
        #define DSPKT_TRF_PRI_MASK      (0x07 << DSPKT_TRF_PRI_SHIFT)
        #define DSPKT_SEQ_CHG_SHIFT     20
        #define DSPKT_SEQ_CHG_MASK      (0x01 << DSPKT_SEQ_CHG_SHIFT)
        #define DSPKT_DSID_SHIFT        0
        #define DSPKT_DSID_MASK         (0xfffff << DSPKT_DSID_SHIFT)
} DsPktMsg;

//DS BPI Sequence Error Message
//Producer  DS MAC DMA
//Consumer  DS Token Processor
typedef struct DsBpiSeqErrMsg
{
    uint32 msgHdr;
        #define DSBPI_DS_CHAN_SHIFT     20
        #define DSBPI_DS_CHAN_MASK      (0x3f << DSBPI_DS_CHAN_SHIFT)
        #define DSBPI_BPI_KSQ_SHIFT     16
        #define DSBPI_BPI_KSQ_MASK      (0x0f << DSBPI_BPI_KSQ_SHIFT)
        #define DSBPI_BPI_EN_SHIFT      15
        #define DSBPI_BPI_EN_MASK       (0x01 << DSBPI_BPI_EN_SHIFT)
        #define DSBPI_BPI_O_E_SHIFT     14
        #define DSBPI_BPI_O_E_MASK      (0x01 << DSBPI_BPI_O_E_SHIFT)
        #define DSBPI_SAID_SHIFT        0
        #define DSBPI_SAID_MASK         (0x3fff << DSBPI_SAID_SHIFT)
} DsBpiSeqErrMsg;

//DS PHS Discard Message
//Producer  DS MAC DMA
//Consumer  DS Token Processor
typedef struct DsPhsDiscardMsg
{
    uint32 msgHdr;
        #define DSPHS_DS_CHAN_SHIFT     20
        #define DSPHS_DS_CHAN_MASK      (0x3f << DSPHS_DS_CHAN_SHIFT)
        #define DSPHS_DSID_MC_SHIFT     14
        #define DSPHS_DSID_MC_MASK      (0x3f << DSPHS_DSID_MC_SHIFT)
        #define DSPHS_RSVD_SHIFT        8
        #define DSPHS_RSVD_MASK         (0x3f << DSPHS_RSVD_SHIFT)
        #define DSPHS_PHS_IDX_SHIFT     0
        #define DSPHS_PHS_IDX_MASK      (0xff << DSPHS_PHS_IDX_SHIFT)
} DsPhsDiscardMsg;

//USPP Request Message
//Producer  US Token Processor
//Consumer  US Packet Preprocessor
typedef struct UsppRequestMsg
{
    uint32 msgHdr;
        // This field became legacy US mode field
        #define USPP_REQ_SEG_HDR_ON_BIT     0x02000000
        #define USPP_REQ_SEG_HDR_ON_SHIFT   25

        #define USPP_REQ_LEGACY_FLAG_BIT    0x02000000
        #define USPP_REQ_LEGACY_FLAG_SHIFT  25
        #define USPP_REQ_FLOW_PRI_BIT       0x01000000
        #define USPP_REQ_FLOW_PRI_SHIFT     24
        #define USPP_REQ_FLOW_ID_MASK       0x00f00000
        #define USPP_REQ_FLOW_ID_SHIFT      20
        #define USPP_REQ_HDR_MODE_MASK      0x000c0000
        #define USPP_REQ_HDR_MODE_SHIFT     18
        #define USPP_REQ_UGS_EHDR_BIT       0x00020000
        #define USPP_REQ_UGS_EHDR_SHIFT     17
        #define USPP_REQ_TOGGLE_QI_BIT      0x00010000
        #define USPP_REQ_TOGGLE_QI_SHIFT    16
        #define USPP_REQ_GEN_HCS_BIT        0x00008000
        #define USPP_REQ_GEN_HCS_SHIFT      15
        #define USPP_REQ_GEN_CRC_BIT        0x00004000
        #define USPP_REQ_GEN_CRC_SHIFT      14
        #define USPP_REQ_BPI_MODE_MASK      0x00003000
        #define USPP_REQ_BPI_MODE_SHIFT     12
        #define USPP_REQ_BPI_KSQ_MASK       0x00000f00
        #define USPP_REQ_BPI_KSQ_SHIFT      8
        #define USPP_REQ_PHS_INDX_MASK      0x000000ff
        #define USPP_REQ_PHS_INDX_SHIFT     0
    uint32 srcToken;
    uint32 dstToken;
    uint32 akclId;
        #define NO_AKCL_ID                  0x00000000
    uint32 sidHdrLen;
        // SID field had been removed from spec
        #define USPP_REQ_SID_MASK           0xfffc0000
        #define USPP_REQ_SID_SHIFT          18

        #define USPP_REQ_MAC_MSG_BIT        0x02000000
        #define USPP_REQ_MAC_MSG_SHIFT      25
        #define USPP_REQ_UGS_GPI_MASK       0x01fc0000
        #define USPP_REQ_UGS_GPI_SHIFT      18
        #define USPP_REQ_NEW_TKN_BIT        0x00020000
        #define USPP_REQ_NEW_TKN_SHIFT      17
        #define USPP_REQ_HDR_OFS_MASK       0x0001ff00
        #define USPP_REQ_HDR_OFS_SHIFT       8
        #define USPP_REQ_HDR_LEN_MASK       0x000000ff
        #define USPP_REQ_HDR_LEN_SHIFT      0
} UsppRequestMsg;

//USPP Response Message
//Producer  US Packet Preprocessor
//Consumer  US Token Processor
typedef struct UsppResponseMsg
{
    uint32 msgHdr;
        #define USPP_RSPN_SEG_HDR_ON_BIT    0x02000000
        #define USPP_RSPN_SEG_HDR_ON_SHIFT  25
        #define USPP_RSPN_FLOW_MASK         0x01f00000
        #define USPP_RSPN_FLOW_SHIFT        20
        #define USPP_RSPN_FLOW_PRI_BIT      0x01000000
        #define USPP_RSPN_FLOW_PRI_SHIFT    24
        #define USPP_RSPN_FLOW_ID_MASK      0x00f00000
        #define USPP_RSPN_FLOW_ID_SHIFT     20
        #define USPP_RSPN_RQD_MASK          0x000fc000
        #define USPP_RSPN_RQD_SHIFT         14
        #define USPP_RSPN_KSQ_INV_BIT       0x00002000
        #define USPP_RSPN_KSQ_INV_SHIFT     13
        #define USPP_RSPN_PHSI_INV_BIT      0x00001000
        #define USPP_RSPN_PHSI_INV_SHIFT    12
        #define USPP_RSPN_BPI_KSQ_MASK      0x00000f00
        #define USPP_RSPN_BPI_KSQ_SHIFT     8
        #define USPP_RSPN_PHS_INDX_MASK     0x000000ff
        #define USPP_RSPN_PHS_INDX_SHIFT    0
    uint32 srcToken;
    uint32 dstToken;
    uint32 akclId;
} UsppResponseMsg;

//US Token Free Message
//Producer  US DMA Processor
//Consumer  US Token Processor
typedef struct UsReleaseTokenMsg
{
    uint32 msgHdr;
        #define USRELEASE_FLOW_PRI_BIT      0x01000000
        #define USRELEASE_FLOW_PRI_SHIFT    24
        #define USRELEASE_FLOW_ID_MASK      0x00f00000
        #define USRELEASE_FLOW_ID_SHIFT     20
        #define USRELEASE_FLUSH_DONE_BIT    0x00000001
        #define USRELEASE_FLUSH_DONE_SHIFT  0
    uint32 tokenId;
        #define USRELEASE_TKN_ID_MASK       0xffff0000
        #define USRELEASE_TKN_ID_SHIFT      16
        #define USRELEASE_NUM_TKN_MASK      0x0000ffff
        #define USRELEASE_NUM_TKN_SHIFT     0
} UsReleaseTokenMsg;

//
// Prototype for the main init function for the generis message
// fifo register set. This function clears the fifos and sets
// up the message lengths for all defined MSG fifo message types.
//
int InitMsgFifo(void);

#endif


