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






#ifndef __gc_hal_vg_h_
#define __gc_hal_vg_h_

#include "gc_hal_enum.h"
#include "gc_hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
**	@ingroup gcoVG
**
**	@brief	Tiling mode for painting and imagig.
**
**	This enumeration defines the tiling modes supported by the HAL.  This is
**	in fact a one-to-one mapping og the OpenVG 1.1 tile modes.
*/
typedef enum _gceVG_TILE_MODE
{
	gcvVG_TILE_FILL,
	gcvVG_TILE_PAD,
	gcvVG_TILE_REPEAT,
	gcvVG_TILE_REFLECT,
}
gceVG_TILE_MODE;

/******************************************************************************/
/** @defgroup gcoPAINT gcoPAINT
**
**	The gcoPAINT object abstracts the painting required by the gcoVG object.
*/

/** Pointer to gcoPAINT object. */
typedef struct _gcoPAINT *		gcoPAINT;

/**
**	@ingroup	gcoPAINT
**
**	@brief		Definition of the color ramp used by the gradient paints.
**
**	The gcsCOLOR_RAMP structure defines the layout of one single color inside
**	a color ramp which is used by gradient paints.
*/
typedef struct _gcsCOLOR_RAMP
{
	/** Value for the color stop. */
	gctFLOAT		stop;

	/** Red color channel value for the color stop. */
	gctFLOAT		red;

	/** Green color channel value for the color stop. */
	gctFLOAT		green;

	/** Blue color channel value for the color stop. */
	gctFLOAT		blue;

	/** Alpha color channel value for the color stop. */
	gctFLOAT		alpha;
}
gcsCOLOR_RAMP, * gcsCOLOR_RAMP_PTR;

gceSTATUS
gcoPAINT_Construct(
	IN gcoVG Vg,
	OUT gcoPAINT * Paint
	);

gceSTATUS
gcoPAINT_Destroy(
	IN gcoPAINT Paint
	);

gceSTATUS
gcoPAINT_SolidColor(
	IN gcoPAINT Paint,
	IN gctFLOAT Red,
	IN gctFLOAT Green,
	IN gctFLOAT Blue,
	IN gctFLOAT Alpha
	);

gceSTATUS
gcoPAINT_Linear(
	IN gcoPAINT Paint,
	IN gctFLOAT ZeroX,
	IN gctFLOAT ZeroY,
	IN gctFLOAT OneX,
	IN gctFLOAT OneY
	);

gceSTATUS
gcoPAINT_Radial(
	IN gcoPAINT Paint,
	IN gctFLOAT CenterX,
	IN gctFLOAT CenterY,
	IN gctFLOAT FocalX,
	IN gctFLOAT FocalY,
	IN gctFLOAT Radius
	);

gceSTATUS
gcoPAINT_Pattern(
	IN gcoPAINT Paint,
	IN gcoSURF Pattern
	);

gceSTATUS
gcoPAINT_SetColorRamp(
	IN gcoPAINT Paint,
	IN gctSIZE_T Entries,
	IN gcsCOLOR_RAMP_PTR ColorRamp,
	IN gctBOOL PreMultiplied
	);

gceSTATUS
gcoPAINT_Transform(
	IN gcoPAINT Paint,
	IN gctFLOAT_PTR Matrix3x2
	);

gceSTATUS
gcoPAINT_SetFillMode(
	IN gcoPAINT Paint,
	IN gceVG_TILE_MODE Mode,
	IN gctUINT32 Color
	);

/******************************************************************************/
/** @defgroup gcoPATH gcoPATH
**
**	The gcoPAINT object abstracts the path required by the gcoVG object.
*/

/** Pointer to gcoPATH object. */
typedef struct _gcoPATH *		gcoPATH;

/** @ingroup gcoPATH
**
**	@brief Structure that defines a scanline.
**
**	The gcsSCANLINE structure defines the layout of a sigle scanline.
*/
#pragma pack(1)
typedef struct _gcsSCANLINE
{
	/** Left coverage value in 0.8 format. */
	gctUINT8 leftCoverage;

	/** Left coordinate of the scanline in short format. */
	gctUINT16 left;

	/** Right coverage value in 0.8 format. */
	gctUINT8 rightCoverage;

	/** Right exclusive coordinate in short format. */
	gctUINT16 right;

	/** Y coordinate of the scanline in short format. */
	gctUINT16 y;
}
gcsSCANLINE, * gcsSCANLINE_PTR;
#pragma pack()

gceSTATUS
gcoPATH_Construct(
	IN gcoVG Vg,
	OUT gcoPATH * Path
	);

gceSTATUS
gcoPATH_Destroy(
	IN gcoPATH Path
	);

gceSTATUS
gcoPATH_Clear(
	IN gcoPATH Path
	);

gceSTATUS
gcoPATH_Reserve(
	IN gcoPATH Path,
	IN gctSIZE_T Scanlines,
	OUT gcsSCANLINE_PTR * Buffer,
	OUT gctSIZE_T * BufferCount
	);

gceSTATUS
gcoPATH_AddScanlines(
	IN gcoPATH Path,
	IN gctSIZE_T Scanlines,
	IN gcsSCANLINE_PTR Buffer
	);

/******************************************************************************/
/** @defgroup gcoVG gcoVG
**
**	The gcoVG object abstracts the VG hardware pipe.
*/

/**
**	@ingroup gcoVG
**
**	@brief	Blending modes supported by the HAL.
**
**	This enumeration defines the blending modes supported by the HAL.  This is
**	in fact a one-to-one mapping og the OpenVG 1.1 blending modes.
*/
typedef enum _gceVG_BLEND
{
	gcvVG_BLEND_SRC,
	gcvVG_BLEND_SRC_OVER,
	gcvVG_BLEND_DST_OVER,
	gcvVG_BLEND_SRC_IN,
	gcvVG_BLEND_DST_IN,
	gcvVG_BLEND_MULTIPLY,
	gcvVG_BLEND_SCREEN,
	gcvVG_BLEND_DARKEN,
	gcvVG_BLEND_LIGHTEN,
	gcvVG_BLEND_ADDITIVE,
}
gceVG_BLEND;

/**
**	@ingroup gcoVG
**
**	@brief	Image modes supported by the HAL.
**
**	This enumeration defines the image modes supported by the HAL.  This is
**	in fact a one-to-one mapping og the OpenVG 1.1 image modes with the addition
**	of NO IMAGE.
*/
typedef enum _gceVG_IMAGE
{
	gcvVG_IMAGE_NONE,
	gcvVG_IMAGE_NORMAL,
	gcvVG_IMAGE_MULTIPLY,
	gcvVG_IMAGE_STENCIL,
}
gceVG_IMAGE;

/**
**	@ingroup gcoVG
**
**	@brief	Filter mode patterns and imaging.
**
**	This enumeration defines the filter modes supported by the HAL.
*/
typedef enum _gceVG_FILTER
{
	gcvVG_FILTER_POINT,
	gcvVG_FILTER_BI_LINEAR,
}
gceVG_FILTER;

/**
**	@ingroup gcoVG
**
**	@brief	Rectangle structure used by the gcoVG object.
**
**	This structure defines the layout of a rectangle.  Make sure width and
**	height are larger than 0.
*/
typedef struct _gcsVG_RECT
{
	/** Left location of the rectangle. */
	gctINT		x;

	/** Top location of the rectangle. */
	gctINT		y;

	/** Width of the rectangle. */
	gctINT		width;

	/** Height of the rectangle. */
	gctINT		height;
}
gcsVG_RECT, * gcsVG_RECT_PTR;

gceSTATUS
gcoVG_Construct(
	IN gcoHAL Hal,
	OUT gcoVG * Vg
	);

gceSTATUS
gcoVG_Destroy(
	IN gcoVG Vg
	);

gceSTATUS
gcoVG_SetTarget(
	IN gcoVG Vg,
	IN gcoSURF Target
	);

gceSTATUS
gcoVG_EnableMask(
	IN gcoVG Vg,
	IN gctBOOL Enable
	);

gceSTATUS
gcoVG_SetMask(
	IN gcoVG Vg,
	IN gcoSURF Mask
	);

gceSTATUS
gcoVG_EnableScissor(
	IN gcoVG Vg,
	IN gctBOOL Enable
	);

gceSTATUS
gcoVG_SetScissor(
	IN gcoVG Vg,
	IN gctSIZE_T RectangleCount,
	IN gcsVG_RECT_PTR Rectangles
	);

gceSTATUS
gcoVG_EnableColorTransform(
	IN gcoVG Vg,
	IN gctBOOL Enable
	);

gceSTATUS
gcoVG_SetColorTransform(
	IN gcoVG Vg,
	IN gctFLOAT Scale[4],
	IN gctFLOAT Offset[4]
	);

gceSTATUS
gcoVG_SetPaint(
	IN gcoVG Vg,
	IN gcoPAINT Paint
	);

gceSTATUS
gcoVG_SetImageMode(
	IN gcoVG Vg,
	IN gceVG_IMAGE Mode
	);

gceSTATUS
gcoVG_SetBlendMode(
	IN gcoVG Vg,
	IN gceVG_BLEND Mode
	);

gceSTATUS
gcoVG_Clear(
	IN gcoVG Vg,
	IN gctFLOAT Red,
	IN gctFLOAT Green,
	IN gctFLOAT Blue,
	IN gctFLOAT Alpha
	);

gceSTATUS
gcoVG_DrawPath(
	IN gcoVG Vg,
	IN gcoPATH Path,
	IN gctINT OffsetX,
	IN gctINT OffsetY
	);

#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_vg_h_ */
