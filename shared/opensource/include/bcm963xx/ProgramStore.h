//**************************************************************************
//
// Copyright (c) 2000-2008 Broadcom Corporation
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
//**************************************************************************
//
//    Filename:      ProgramStore.h
//    Author:        Sean Nazareth
//    Creation Date: 2000
//
//**************************************************************************

#ifndef PROGRAMSTORE_H
#define PROGRAMSTORE_H

#include "bcmtypes.h"

#define MIPS_BOOT_ADDRESS       0xbfc00000

typedef struct _BcmProgramHeader {

    // The application signature.  It may match the chip ID or a customer product ID.
    uint16 usSignature;

    // Control flags.  The compression type is an integer from 0-7, with 0
    // meaning uncompressed.  For historical reasons, the number 3 was skipped.
    // The upper 8 bits can be used for other features, such as dual binary.
    uint16 usControl;
        #define  PGM_CONTROL_COMPRESSION_BITS              0x0007
        #define  PGM_CONTROL_COMPRESSED_IMAGE_LZRW1_KH     0x0001
        #define  PGM_CONTROL_COMPRESSED_IMAGE_MINILZO      0x0002
        #define  PGM_CONTROL_COMPRESSED_IMAGE_NRV2D99      0x0004
        #define  PGM_CONTROL_COMPRESSED_IMAGE_LZMA         0x0005
        #define  PGM_CONTROL_DUAL_IMAGES                   0x0100

    // Major and minor software revisions no longer mean anything in particular.
    // Customers may use them for their own purposes.
    uint16 usMajorRevision;
    uint16 usMinorRevision;

    // Time when image was ProgramStore'd, expressed as seconds since Jan 1 1970.
    uint32 ulcalendarTime;

    // Length of the file after compression, not including this header.
    uint32 ulfilelength;

    // Address where the program should be loaded.
    uint32 ulProgramLoadAddress;

    // Null-terminated file name, and padding reserved for future use.
    char cFilename[48];
    char pad[8];

    // When doing a dual binary, these are the lengths of the two parts.  The
    // sum of these should equal ulfilelength.
    unsigned long ulCompressedLength1;
    unsigned long ulCompressedLength2;

    // 16-bit crc Header checksum, not including the remaining fields.
    uint16 usHcs;

    // Reserved for future use.
    uint16 reserved;

    // CRC-32 of Program portion of file, not including this header.
    uint32 ulcrc;

} BcmProgramHeader;


// ---------------------------------------------------------------------------
// Return each possible location for an image in flash.
// ---------------------------------------------------------------------------
bool PSGetNextImageLocation( int imageNum, uint32 *Offset, int *MaxSize, int *MinSize );
bool PSFindImage( int  imageNum, int  *offset );

bool PSGetImageHeader( uint32 Offset, BcmProgramHeader **pHeader );
bool PSGetImage(       uint32 Offset, BcmProgramHeader **pHeader );

void PSClobberImage( int imageNum, int newOffset );
void PSWriteImage( int imageNum, uint8* buffer, uint32 size );
int  PSWriteBootloader( uint8* buffer, uint32 size );

#endif
