//**************************************************************************** 
// 
// Copyright (c) 2003-2008 Broadcom Corporation
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
//  $Id$
// 
//  Filename:       bcmtypes.h 
//  Author:         [author] 
//  Creation Date:  [date] 
// 
//**************************************************************************** 
//  Description: 
//       
//       
//       
//       
// 
//**************************************************************************** 
//
// bcmtypes.h - misc useful typedefs
//
#ifndef BCMMACRO_H
#define BCMMACRO_H

/* macros to protect against unaligned accesses */

/* first arg is an address, second is a value */
#define PUT16( a, d ) { 		\
  *((byte *)a) = (byte)((d)>>8); 	\
  *(((byte *)a)+1) = (byte)(d); 	\
}

#define PUT32( a, d ) { 		\
  *((byte *)a) = (byte)((d)>>24); 	\
  *(((byte *)a)+1) = (byte)((d)>>16); 	\
  *(((byte *)a)+2) = (byte)((d)>>8); 	\
  *(((byte *)a)+3) = (byte)(d); 	\
}

/* first arg is an address, returns a value */
#define GET16( a ) ( 			\
  (*((byte *)a) << 8) |			\
  (*(((byte *)a)+1))	 		\
)

#define GET32( a ) ( 			\
  (*((byte *)a) << 24)     |		\
  (*(((byte *)a)+1) << 16) | 		\
  (*(((byte *)a)+2) << 8)  | 		\
  (*(((byte *)a)+3))	 		\
)

#ifndef NULL
#define NULL            0
#endif

typedef unsigned long CM_STATUS;
#define CM_STATUS_SUCCESS (CM_STATUS)0
#define CM_STATUS_FAILURE (CM_STATUS)1

//#define CacheFlush      0x01        // flushes the cache (ie: does writeback)
//#define CacheInvalidate 0x02        // invalidates cache entries
//#define DCACHE_LINE_SIZE 16         // line size for dcache

#define	BCM_ALIGN(addr, boundary) ((addr + boundary - 1) & ~(boundary - 1))
#define K0_TO_K1(x)     ((unsigned int)(x)|0xA0000000)    /* kseg0 to kseg1 */
#define K1_TO_K0(x)     ((unsigned int)(x)&0x9FFFFFFF)    /* kseg1 to kseg0 */
#define K0_TO_PHYS(x)   ((unsigned int)(x)&0x1FFFFFFF)    /* kseg0 to physical */
#define K1_TO_PHYS(x)   ((unsigned int)(x)&0x1FFFFFFF)    /* kseg1 to physical */
/* PHYS_TO_K0 is defined in include/asm-mips/addrspace.h */
#undef PHYS_TO_K0
#define PHYS_TO_K0(x)   ((unsigned int)(x)|0x80000000)    /* physical to kseg0 */
#define PHYS_TO_K1(x)   ((unsigned int)(x)|0xA0000000)    /* physical to kseg1 */

#define  CacheToNonCacheVirtAddress    K0_TO_K1
#define  NonCacheToCacheVirtAddress    K1_TO_K0
#define  CacheVirtToPhyAddress         K0_TO_PHYS
#define  NonCacheVirtToPhyAddress      K1_TO_PHYS
#define  PhyToNonCacheVirtAddress      PHYS_TO_K1
#define  PhyToCacheVirtAddress         PHYS_TO_K0

/* Register Macros to handle shadow register writing for 
   bitfields.  This macro handles read modify writes.    */
#define ReadModWrField(reg, type, field, value) \
{                                               \
	type local_##type;				   	        \
	local_##type.Reg32 = reg.Reg32;             \
	local_##type.Bits.field = value;            \
	reg.Reg32 = local_##type.Reg32;             \
}


/* Register Macros to handle shadow register writing for 
   bitfields.  This macro handles write onlys.           */
#define WrField(reg, type, field, value)        \
{                                               \
	type local_##type;				   	        \
	local_##type.Bits.field = value;            \
	reg.Reg32 = local_##type.Reg32;             \
}


/* Register Macros to handle shadow register writing for 
   bitfields.  This macro handles read onlys.           */
#define ReadField(reg, type, field, value)      \
{                                               \
	type local_##type;				   	        \
	local_##type.Reg32 = reg.Reg32;             \
	value = local_##type.Bits.field;            \
}


/* Register Macros to handle shadow register writing for 
   8 bit bitfields.  This macro handles write onlys.           */
#define WrField8(reg, type, field, value)        \
{                                               \
	type local_##type;				   	        \
	local_##type.Bits.field = value;            \
	reg.Reg8 = local_##type.Reg8;             \
}


/* Register Macros to handle shadow register writing for 
   bitfields.  This macro handles read onlys.           */
#define ReadField8(reg, type, field, value)      \
{                                               \
	type local_##type;				   	        \
	local_##type.Reg8 = reg.Reg8;             \
	value = local_##type.Bits.field;            \
}



/* General bit enable */
enum
{
    ONE  = 1,
    ZERO = 0
};

#ifndef YES
#define YES 1
#endif

#ifndef NO
#define NO  0
#endif

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#endif
