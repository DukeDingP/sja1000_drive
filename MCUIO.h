/*--------------------------------------------------------------------
 * TITLE: m2 Hardware Defines
 * AUTHOR: John & Jack 
 * DATE CREATED: 2013/10/10
 * FILENAME: m2.h
 * PROJECT: m2
 * COPYRIGHT: Small World, Inc.
 * DESCRIPTION:
 *    m2 Hardware Defines
 *
 *    2014-03-17: added sd adc, opo, v2p; sys reg modified
 *    2014-01-11: added sd adc, opo, v2p; sys reg modified
 *    2013-12-18: misc edit
 *    2013-12-15: uart reg back to m1
 *    2012-10-16: modified base on m2 new design
 *    2012-10-10: modified base on s0.h
 *--------------------------------------------------------------------*/
#ifndef __MCUIO_H__
#define __MCUIO_H__
#include "mcu.h"
#include "SPI.h"


/*********** Hardware addesses ***********/
#define SYS_GDR_REG		  0x1f800703  // gdr register
#define SYS_IOCTL_REG     0x1f800704  // 0=in; 1-out (16-bit), was IO config
#define SYS_GPIO0_REG     0x1f800705  // GPIO (16-bit) to pad content 
#define SYS_GPIO1_REG     0x1f800706  // GPIO (16_bit) from pad read 

#define SET_GPIO0_BIT_ON(A) {MemoryOr32(SYS_IOCTL_REG, (1<<A)); MemoryOr32(SYS_GPIO0_REG, (1<<A)); }
#define SET_GPIO0_BIT_OFF(A) {MemoryOr32(SYS_IOCTL_REG, (1<<A)); MemoryAnd32(SYS_GPIO0_REG, ~(1<<A)); }
#define GET_GPIO1_BIT(A) MemoryBitAt(SYS_GPIO1_REG, A)

/*************** IO Setup***************/
#define RT_IOCTL_Set1(n, oe) 	MemoryOr32(SYS_IOCTL_REG, oe<<n);
						
#define RT_IOCTL_Set16(oe)		MemoryWrite32(SYS_IOCTL_REG, oe);

#define RT_IOCTL_And16(oe)		MemoryAnd32(SYS_IOCTL_REG, oe);

#define RT_IOCTL_Or16(oe)		MemoryOr32(SYS_IOCTL_REG, oe);

#define RT_GPIO_Write1(n, v)	{MemoryAnd32(SYS_GPIO0_REG, ~(1<<n)); MemoryOr32(SYS_GPIO0_REG, v<<n);}
						
#define RT_GPIO_Write16(v)		MemoryWrite32(SYS_GPIO0_REG, v);	

#define RT_GPIO_Read1(n)	((MemoryRead32(SYS_GPIO0_REG)&(1<<n))>>n)

#define RT_GPIO_Read16()	MemoryRead32(SYS_GPIO0_REG)
/**************** IO End****************/

/******************gdr out***************/
#define RT_GDR_Write32(v)		MemoryWrite32(SYS_GDR_REG, v)

#endif //__MCUIO_H__
