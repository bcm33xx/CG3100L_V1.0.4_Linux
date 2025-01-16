/*
<:copyright-gpl 
 Copyright 2007 Broadcom Corp. All Rights Reserved. 
 
 This program is free software; you can distribute it and/or modify it 
 under the terms of the GNU General Public License (Version 2) as 
 published by the Free Software Foundation. 
 
 This program is distributed in the hope it will be useful, but WITHOUT 
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License 
 for more details. 
 
 You should have received a copy of the GNU General Public License along 
 with this program; if not, write to the Free Software Foundation, Inc., 
 59 Temple Place - Suite 330, Boston MA 02111-1307, USA. 
:>
*/

#ifndef __3380_INTR_H
#define __3380_INTR_H

#ifdef __cplusplus
    extern "C" {
#endif

#define INTERRUPT_ID_SOFTWARE_0           0
#define INTERRUPT_ID_SOFTWARE_1           1

/*=====================================================================*/
/* BCM3380 Timer Interrupt Level Assignments                          */
/*=====================================================================*/
#define MIPS_TIMER_INT                  7

/*=====================================================================*/
/* Peripheral ISR Table Offset                                              */
/*=====================================================================*/
#define INTERNAL_ISR_TABLE_OFFSET       8

/*=====================================================================*/
/* Logical Peripheral Interrupt IDs                                    */
/*=====================================================================*/

#define INTERRUPT_ID_TIMER               (INTERNAL_ISR_TABLE_OFFSET + 0)
#define INTERRUPT_ID_SPI                 (INTERNAL_ISR_TABLE_OFFSET + 1)
#define INTERRUPT_ID_UART                (INTERNAL_ISR_TABLE_OFFSET + 2)
#define INTERRUPT_ID_UART1               (INTERNAL_ISR_TABLE_OFFSET + 3)
#define INTERRUPT_ID_SIMCARD0            (INTERNAL_ISR_TABLE_OFFSET + 4)
#define INTERRUPT_ID_SIMCARD1            (INTERNAL_ISR_TABLE_OFFSET + 5)
#define INTERRUPT_ID_I2C                 (INTERNAL_ISR_TABLE_OFFSET + 6)
#define INTERRUPT_ID_HS_SPI              (INTERNAL_ISR_TABLE_OFFSET + 7)
#define INTERRUPT_ID_RING_OSC            (INTERNAL_ISR_TABLE_OFFSET + 8)
#define INTERRUPT_ID_PERIPH_ERR          (INTERNAL_ISR_TABLE_OFFSET + 9)
#define INTERRUPT_ID_RESERVED_10         (INTERNAL_ISR_TABLE_OFFSET + 10)
#define INTERRUPT_ID_RESERVED_11         (INTERNAL_ISR_TABLE_OFFSET + 11)
#define INTERRUPT_ID_RESERVED_12         (INTERNAL_ISR_TABLE_OFFSET + 12)
#define INTERRUPT_ID_RESERVED_13         (INTERNAL_ISR_TABLE_OFFSET + 13)
#define INTERRUPT_ID_RESERVED_14         (INTERNAL_ISR_TABLE_OFFSET + 14)
#define INTERRUPT_ID_PCIE_RC             (INTERNAL_ISR_TABLE_OFFSET + 15)
#define INTERRUPT_ID_PCIE_EP_LNK_RST     (INTERNAL_ISR_TABLE_OFFSET + 16)
#define INTERRUPT_ID_BRG_UBUS0           (INTERNAL_ISR_TABLE_OFFSET + 17)
#define INTERRUPT_ID_BRG_UBUS1           (INTERNAL_ISR_TABLE_OFFSET + 18)
#define INTERRUPT_ID_FPM                 (INTERNAL_ISR_TABLE_OFFSET + 19)
#define INTERRUPT_ID_USB0                (INTERNAL_ISR_TABLE_OFFSET + 20)
#define INTERRUPT_ID_USB1                (INTERNAL_ISR_TABLE_OFFSET + 21)
#define INTERRUPT_ID_APM                 (INTERNAL_ISR_TABLE_OFFSET + 22)
#define INTERRUPT_ID_APM_DMA             (INTERNAL_ISR_TABLE_OFFSET + 23)
#define INTERRUPT_ID_UNI_IRQ2            (INTERNAL_ISR_TABLE_OFFSET + 24)
#define INTERRUPT_ID_UNI_IRQ             (INTERNAL_ISR_TABLE_OFFSET + 25)
#define INTERRUPT_ID_GPHY_IRQB           (INTERNAL_ISR_TABLE_OFFSET + 26)
#define INTERRUPT_ID_DAVIC               (INTERNAL_ISR_TABLE_OFFSET + 27)
#define INTERRUPT_ID_OB                  (INTERNAL_ISR_TABLE_OFFSET + 28)
#define INTERRUPT_ID_RESERVED_29         (INTERNAL_ISR_TABLE_OFFSET + 29)
#define INTERRUPT_ID_RESERVED_30         (INTERNAL_ISR_TABLE_OFFSET + 30)
#define INTERRUPT_ID_EXT_IRQ             (INTERNAL_ISR_TABLE_OFFSET + 31)

#define INTERRUPT_ID_MEP_IRQ             (INTERNAL_ISR_TABLE_OFFSET + 2 + 32)
#define INTERRUPT_ID_MSP_IRQ             (INTERNAL_ISR_TABLE_OFFSET + 3 + 32)
#define INTERRUPT_ID_MSP_SW_IRQ          (INTERNAL_ISR_TABLE_OFFSET + 5 + 32)

#define INTERRUPT_ID_LAST                INTERRUPT_ID_MSP_SW_IRQ
#define INTERRUPT_ID_MPI                 INTERRUPT_ID_EXT_IRQ

#ifdef __cplusplus
    }
#endif                    

#endif  /* __BCM3380_H */


