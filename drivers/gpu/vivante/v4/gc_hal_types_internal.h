/****************************************************************************
*
*    Copyright (C) 2005 - 2012 by Vivante Corp.
*
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation; either version 2 of the license, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not write to the Free Software
*    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*****************************************************************************/

#ifndef __gc_hal_types_internal_h_
#define __gc_hal_types_internal_h_

#include <linux/stddef.h>

/******************************************************************************\
********************************** Common Types ********************************
\******************************************************************************/

#define gcvFALSE                0
#define gcvTRUE                 1

#define gcvINFINITE             ((u32) ~0U)

/* Stringizing macro. */
#define gcmSTRING(Value)        #Value

#define gcmPRINTABLE(c)         ((((c) >= ' ') && ((c) <= '}')) ? ((c) != '%' ?  (c) : ' ') : ' ')

#define gcmCC_PRINT(cc) \
    gcmPRINTABLE((char) ( (cc)        & 0xFF)), \
    gcmPRINTABLE((char) (((cc) >>  8) & 0xFF)), \
    gcmPRINTABLE((char) (((cc) >> 16) & 0xFF)), \
    gcmPRINTABLE((char) (((cc) >> 24) & 0xFF))

/******************************************************************************\
********************************* Status Macros ********************************
\******************************************************************************/

#define gcmIS_ERROR(status)         (status < 0)
#define gcmIS_SUCCESS(status)       (status == gcvSTATUS_OK)

/******************************************************************************\
********************************* Field Macros *********************************
\******************************************************************************/

#define __gcmSTART(reg_field) \
    (0 ? reg_field)

#define __gcmEND(reg_field) \
    (1 ? reg_field)

#define __gcmGETSIZE(reg_field) \
    (__gcmEND(reg_field) - __gcmSTART(reg_field) + 1)

#define __gcmALIGN(data, reg_field) \
    (((u32) (data)) << __gcmSTART(reg_field))

#define __gcmMASK(reg_field) \
    ((u32) ((__gcmGETSIZE(reg_field) == 32) \
        ?  ~0 \
        : (~(~0 << __gcmGETSIZE(reg_field)))))

/*******************************************************************************
**
**  gcmFIELDMASK
**
**      Get aligned field mask.
**
**  ARGUMENTS:
**
**      reg     Name of register.
**      field   Name of field within register.
*/
#define gcmFIELDMASK(reg, field) \
( \
    __gcmALIGN(__gcmMASK(reg##_##field), reg##_##field) \
)

/*******************************************************************************
**
**  gcmSETFIELD
**
**      Set the value of a field within specified data.
**
**  ARGUMENTS:
**
**      data    Data value.
**      reg     Name of register.
**      field   Name of field within register.
**      value   Value for field.
*/
#define gcmSETFIELD(data, reg, field, value) \
( \
    (((u32) (data)) \
        & ~__gcmALIGN(__gcmMASK(reg##_##field), reg##_##field)) \
        |  __gcmALIGN((u32) (value) \
            & __gcmMASK(reg##_##field), reg##_##field) \
)

/*******************************************************************************
**
**  gcmVERIFYFIELDVALUE
**
**      Verify if the value of a field within specified data equals a
**      predefined value.
**
**  ARGUMENTS:
**
**      data    Data value.
**      reg     Name of register.
**      field   Name of field within register.
**      value   Name of the value within the field.
*/
#define gcmVERIFYFIELDVALUE(data, reg, field, value) \
( \
    (((u32) (data)) >> __gcmSTART(reg##_##field) & \
                             __gcmMASK(reg##_##field)) \
        == \
    (reg##_##field##_##value & __gcmMASK(reg##_##field)) \
)

/******************************************************************************\
******************************** Min/Max Macros ********************************
\******************************************************************************/

#define gcmMAX(x, y)            (((x) >= (y)) ?  (x) :  (y))

/*******************************************************************************
**
**  gcmPTR2INT
**
**      Convert a pointer to an integer value.
**
**  ARGUMENTS:
**
**      p       Pointer value.
*/
#if defined(__LP64__) && __LP64__
#   define gcmPTR2INT(p) \
    ( \
        (u32) (u64) (p) \
    )
#else
#   define gcmPTR2INT(p) \
    ( \
        (u32) (p) \
    )
#endif

/*******************************************************************************
**
**  gcmINT2PTR
**
**      Convert an integer value into a pointer.
**
**  ARGUMENTS:
**
**      v       Integer value.
*/
#ifdef __LP64__
#   define gcmINT2PTR(i) \
    ( \
        (void *) (s64) (i) \
    )
#else
#   define gcmINT2PTR(i) \
    ( \
        (void *) (i) \
    )
#endif

#endif /* __gc_hal_types_internal_h_ */
