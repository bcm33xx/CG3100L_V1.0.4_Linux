//****************************************************************************
//
//  Copyright (c) 2003  Broadcom Corporation
//  All Rights Reserved
//  No portions of this material may be reproduced in any form without the
//  written permission of:
//          Broadcom Corporation
//          16215 Alton Parkway
//          Irvine, California 92619
//  All information contained in this document is Broadcom Corporation
//  company private, proprietary, and trade secret.
//
//****************************************************************************
//
//  $Id$
//
//  Filename:       types.h
//  Author:         Mike Sieweke
//  Creation Date:  24-Apr-2002
//
//**************************************************************************
//    Description:
//
//	  types.h includes sys/types.h and defines a few types that are
//    defined elsewhere in pSOS and vxWorks.
//
//**************************************************************************

#ifndef types_h
#define types_h

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned char   UCHAR;
typedef unsigned short  USHORT;
typedef unsigned int    UINT;
typedef unsigned long   ULONG;

#define TRUE 1
#define FALSE 0

#define VIRT_TO_PHY(a)  (((unsigned long)(a)) & 0x1fffffff)

#ifdef __cplusplus
}
#endif

#endif


