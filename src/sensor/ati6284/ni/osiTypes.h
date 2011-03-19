/* ! \file include/sensor/ati6284/ni/osiTypes.h
    * \brief plik nag��wkowy NI
    * Ostatnia modyfikacja: 04.2006 
    *   
    *  osiTypes.h contains the constants and macros
    *  needed for ChipObjects and the iBus implementation.
    *
    */
#ifndef __osiTypes_h__
#define __osiTypes_h__
#include <stdio.h>

//!< Note: these different typedef's may be different
//!< depending on you system.  On a 32 bit processor with a 32 bit
//!< OS, these will probably not need to be modified.
typedef char               i8;
typedef unsigned char      u8;
typedef short              i16;
typedef unsigned short     u16;
typedef float              f32;
typedef long               i32;
typedef unsigned long      u32;
typedef double             f64;
typedef char               tText;
typedef i32                tStatus;

typedef u32                tBoolean;
typedef char               tChar;

enum {
	kFalse = 0,
	kTrue = 1
};

#define  markAsUnused(type,variable)
#define  kStatusOffset                    -50000
#define  kStatusSuccess                   0
#define  kStatusBadWindowType             (-16  + kStatusOffset)

//#define  NULL                                0

#endif


