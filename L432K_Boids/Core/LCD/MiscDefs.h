/*
 * MiscDefs.h
 *
 *  Created on: Apr 12, 2022
 *      Author: donst
 */

#ifndef INC_MISCDEFS_H_
#define INC_MISCDEFS_H_

#ifndef NULL
#define NULL ((void *)0)
#endif

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE (!FALSE)
#endif

/* Macro to get the offset of a member of a structure in memory (ROM or RAM) */
#define _offsetof(s,m) ((unsigned short)((unsigned char *)(&s.m) - (unsigned char *)(&s)))

/* Macro to get the offset of a member of a structure, by structure type */
#define _offsetofT(t,m)		( (unsigned short) ( (unsigned char *)&(((t *)0)->m) - (unsigned char *)0 ) )

#define GetTickCount	HAL_GetTick


#define _min(a,b)	(((a) < (b)) ? (a) : (b))
#define _max(a,b)	((a) > (b) ? (a) : (b))

#define _FarPtrToNear(p)	((void *)(unsigned short)(unsigned long)(p))

#define _EI	__enable_irq
#define _DI	__disable_irq

// fatal error definitions
typedef enum
{
	_FE_Misc,
} _eFatalError;
void FatalError( _eFatalError type, unsigned long arg1, unsigned short arg2 );

typedef enum
{
	stx		= 0x02,
	etx		= 0x03,
	eot		= 0x04,
	enq		= 0x05,
	ack		= 0x06,
	nak		= 0x15,
	can		= 0x18,
	bel		= 0x07,
	dle		= 0x10
} _eASCII;




#endif /* INC_MISCDEFS_H_ */
