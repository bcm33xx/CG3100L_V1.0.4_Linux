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
//
//  Filename:       ScdmaHelperConstants.h
//  Author:         David Pullen
//  Creation Date:  Sept 4, 2008
//
//****************************************************************************

#ifndef ScdmaHelperConstants_H
#define ScdmaHelperConstants_H

//********************** Include Files ***************************************

//********************** Global Types ****************************************

//********************** Global Constants ************************************

/// These are values for the DQM queues for communications between the IOP and
/// host MIPS.  The _Queue constant gives the queue # (0..31); the
/// _EntrySizeWords constant (1..4) gives the size of each message in the queue;
/// the _NumEntries constant (1..N) determines the number of entries in the
/// queue; and the _LowWater constant (0..N) determines the point at which the
/// low water mark interrupt will be asserted.
/// 
/// See this wiki page for documentation on the commands:
/// 
/// {TBD}

enum
{
    //@{ HostToIopCmd
    ///
    /// This DQM queue is used by the host MIPS to send commands to the IOP.
    ///
    /// See this page for detailed docs on the commands and parameters:
    /// {TBD}
    kDqm_HostToIopCmd_Queue = 30,
    kDqm_HostToIopCmd_EntrySizeWords = 4,
    kDqm_HostToIopCmd_NumEntries = 8,
    kDqm_HostToIopCmd_LowWater = 0,

    /// Values 0 and 1 deliberately skipped...
    kDqm_HostToIopCmd_Reserved0 = 0,
    kDqm_HostToIopCmd_Reserved1 = 1,

    /// These should be common across all IOPs.
    kDqm_HostToIopCmd_PingRequest = 2,
    kDqm_HostToIopCmd_ResetRequest = 3,
    kDqm_HostToIopCmd_ShutdownRequest = 4,
    kDqm_HostToIopCmd_LowPowerRequest = 5,

    /// These are specific to this IOP application.
    kDqm_HostToIopCmd_StartScdmaHelperRequest = 100,
    kDqm_HostToIopCmd_StopScdmaHelperRequest = 101,

    //@}
};


enum
{
    //@{ IopToHostCmdStatus
    ///
    /// This DQM queue is used by the IOP to send cmd replies and status to the
    /// host MIPS.
    /// 
    /// See this page for detailed docs on the commands and parameters:
    /// {TBD}
    kDqm_IopToHostCmdStatus_Queue = 31,
    kDqm_IopToHostCmdStatus_EntrySizeWords = 2,
    kDqm_IopToHostCmdStatus_NumEntries = 8,
    kDqm_IopToHostCmdStatus_LowWater = 0,

    /// Value 0 deliberately skipped...
    kDqm_IopToHostCmdStatus_Reserved0 = 0,

    /// These should be common across all IOPs.
    kDqm_IopToHostCmdStatus_InitComplete = 1,
        kDqm_IopToHostCmdStatus_InitCompleteSuccess = 1,
        kDqm_IopToHostCmdStatus_InitCompleteFailure = 2,
    kDqm_IopToHostCmdStatus_PingReply = 2,
    kDqm_IopToHostCmdStatus_ResetReply = 3,
    kDqm_IopToHostCmdStatus_ShutdownReply = 4,
    kDqm_IopToHostCmdStatus_Error = 6,

        /// These are the IOP-specific error causes.
        kIopErrorCause_None = 0,
        kIopErrorCause_Unspecified = 1,

    /// These are specific to this IOP application.
    kDqm_IopToHostCmdStatus_StartScdmaHelperReply = 100,
    kDqm_IopToHostCmdStatus_StopScdmaHelperReply = 101,

};


//********************** Global Variables ************************************

//********************** Forward Declarations ********************************

//********************** Function Prototypes *********************************

//********************** Inline Method Implementations ***********************

#endif


