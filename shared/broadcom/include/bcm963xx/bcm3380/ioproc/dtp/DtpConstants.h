//****************************************************************************
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
//****************************************************************************
//  $Id$
//
//  Filename:       DtpConstants.h
//  Author:         David Pullen
//  Creation Date:  July 16 2007
//
//****************************************************************************

#ifndef DtpConstants_H
#define DtpConstants_H

//********************** Include Files ***************************************

//********************** Global Types ****************************************

//********************** Global Constants ************************************

/// These are values for the DQM queues for communications between the DTP and
/// host MIPS.  The _Queue constant gives the queue # (0..31); the
/// _EntrySizeWords constant (1..4) gives the size of each message in the queue;
/// the _NumEntries constant (1..N) determines the number of entries in the
/// queue; and the _LowWater constant (0..N) determines the point at which the
/// low water mark interrupt will be asserted.

enum
{
    //@{ HostToDtpCmd
    ///
    /// This DQM queue is used by the host MIPS to send commands to the DTP.
    kDqm_HostToDtpCmd_Queue = 30,
    kDqm_HostToDtpCmd_EntrySizeWords = 4,
    kDqm_HostToDtpCmd_NumEntries = 8,
    kDqm_HostToDtpCmd_LowWater = 0,

    /// Values 0 and 1 deliberately skipped...
    kDqm_HostToDtpCmd_Reserved0 = 0,
    kDqm_HostToDtpCmd_Reserved1 = 1,

    /// These should be common across all IOPs.
    kDqm_HostToDtpCmd_PingRequest = 2,
    kDqm_HostToDtpCmd_ResetRequest = 3,
    kDqm_HostToDtpCmd_ShutdownRequest = 4,
    kDqm_HostToDtpCmd_LowPowerRequest = 5,

    /// These are specific to the DTP.
    kDqm_HostToDtpCmd_CreateReseqDsidRequest = 100,
    kDqm_HostToDtpCmd_ChangeReseqDsidRequest = 101,
    kDqm_HostToDtpCmd_DeleteReseqDsidRequest = 102,
    kDqm_HostToDtpCmd_IdleReseqDsidRequest = 103,
    kDqm_HostToDtpCmd_PrintDsidCounters = 104,
    kDqm_HostToDtpCmd_PrintDsidState = 105,

    kDqm_HostToDtpCmd_ErrorDetected = 255,

    //@}
};


enum
{
    //@{ DtpToHostCmdStatus
    ///
    /// This DQM queue is used by the DTP to send commands and status to the
    /// host MIPS.
    /// 
    /// The general format of messages in this queue is:
    /// 
    /// /verbatim
    /// +--------------------------------+--------------------------------+
    /// | cmd/status code                | data                           |
    /// +--------------------------------+--------------------------------+
    ///   32 bits                          32 bits
    /// /endverbatim
    /// 
    kDqm_DtpToHostCmdStatus_Queue = 31,
    kDqm_DtpToHostCmdStatus_EntrySizeWords = 2,
    kDqm_DtpToHostCmdStatus_NumEntries = 8,
    kDqm_DtpToHostCmdStatus_LowWater = 0,

    /// Value 0 deliberately skipped...
    kDqm_DtpToHostCmdStatus_Reserved0 = 0,

    /// These should be common across all IOPs.
    kDqm_DtpToHostCmdStatus_InitComplete = 1,
        kDqm_DtpToHostCmdStatus_InitCompleteSuccess = 1,
        kDqm_DtpToHostCmdStatus_InitCompleteFailure = 2,
    kDqm_DtpToHostCmdStatus_PingReply = 2,
    kDqm_DtpToHostCmdStatus_ResetReply = 3,

    //@{
    /// This reply message is sent by the DTP in response to a ShutdownRequest
    /// command.  The first data value in the request is echoed back.
    /// 
    /// The DTP performs the same operations as it would when processing a
    /// ResetRequest command, and then further de-initializes h/w state machines
    /// and RAM.  It may take a non-trivial amount of time for this to complete.
    /// 
    /// /verbatim
    /// +-------------------+-----------------------------+
    /// | ShutdownReply (4) | Data (from ShutdownRequest) |
    /// +-------------------+-----------------------------+
    /// /endverbatim
    kDqm_DtpToHostCmdStatus_ShutdownReply = 4,
    //@}

    /// This is used to indicate that an error occurred.  The error cause
    /// is IOP-specific.
    kDqm_DtpToHostCmdStatus_Error = 6,

        /// These are the DTP-specific error causes.
        kDtpErrorCause_None = 0,
        kDtpErrorCause_Unspecified = 1,
        kDtpErrorCause_WaitForCompleteTimeout = 2,
        kDtpErrorCause_DsrmLookupTimeout = 3,
        kDtpErrorCause_DsrmBypassInsertTimeout = 4,
        kDtpErrorCause_DsrmInsertTimeout = 5,
        kDtpErrorCause_LegacyDataDqmOverflow = 6,
        kDtpErrorCause_MacMessageDqmOverflow = 7,
        kDtpErrorCause_BadToken = 8,

    /// These are specific to the DTP.
    kDqm_DtpToHostCmdStatus_CreateReseqDsidReply = 100,
        kCreateReseqDsidCause_Unspecified = 0,
        kCreateReseqDsidCause_DuplicateDsid = 1,
        kCreateReseqDsidCause_TooManyDsids = 2,
        kCreateReseqDsidCause_InsufficientMemory = 3,
        kCreateReseqDsidCause_InvalidReseqWarnThresh = 4,
        kCreateReseqDsidCause_DsidSlotNotAvailable = 5,
        kCreateReseqDsidCause_NumClusters0 = 6,
        kCreateReseqDsidCause_MaxDelayMsTooLarge = 7,
    kDqm_DtpToHostCmdStatus_ChangeReseqDsidReply = 101,
        kChangeReseqDsidCause_Unspecified = 0,
        kChangeReseqDsidCause_DsidNotFound = 1,
        kChangeReseqDsidCause_InsufficientMemory = 2,
        kChangeReseqDsidCause_InvalidReseqWarnThresh = 3,
        kChangeReseqDsidCause_NumClusters0 = 4,
        kChangeReseqDsidCause_MaxDelayMsTooLarge = 5,
    kDqm_DtpToHostCmdStatus_DeleteReseqDsidReply = 102,
        kDeleteReseqDsidCause_Unspecified = 0,
        kDeleteReseqDsidCause_DsidNotFound = 1,
    kDqm_DtpToHostCmdStatus_IdleReseqDsidReply = 103,
   kDqm_DtpToHostCmdStatus_PrintDsidCountersReply = 104,
    kDqm_DtpToHostCmdStatus_PrintDsidStateReply = 105,

};


enum
{
    //@{ DtpToHostMacMessage
    ///
    /// This DQM is used by the DTP to send tokens containing MAC messages to
    /// the host MIPS.
    kDqm_DtpToHostMacMessage_Queue = 29,
    kDqm_DtpToHostMacMessage_EntrySizeWords = 2,
    kDqm_DtpToHostMacMessage_NumEntries = 16,
    kDqm_DtpToHostMacMessage_LowWater = 0,
    //@}
};


enum
{
    //@{ DtpToHostLegacyData
    ///
    /// This DQM is used by the DTP to send legacy (non-DSID-labelled) tokens
    /// containing data packets to the host MIPS.
    kDqm_DtpToHostLegacyData_Queue = 28,
    kDqm_DtpToHostLegacyData_EntrySizeWords = 1,
    kDqm_DtpToHostLegacyData_NumEntries = 512,
    kDqm_DtpToHostLegacyData_LowWater = 0,
    //@}
};


enum
{
    //@{ DtpDsidData
    ///
    /// These DQM queues are used by the DTP to send DSID-labelled tokens
    /// containing data packets to the host MIPS.  The _Queue is the base
    /// queue number.  We support 16 DSIDs, so 16 queues will be allocated.
    //kDqm_DtpDsidData_Queue = 0,  Not used, always 0.
    kDqm_DtpDsidData_EntrySizeWords = 1,
    kDqm_DtpDsidData_NumEntries = 256,
    kDqm_DtpDsidData_LowWater = 0,
    //@}
};


//********************** Global Variables ************************************

//********************** Forward Declarations ********************************

//********************** Function Prototypes *********************************

//********************** Inline Method Implementations ***********************

#endif


