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




#ifndef __gc_hal_enum_h_
#define __gc_hal_enum_h_

#include "gc_hal_options.h"


/* Chip models. */
typedef enum _gceCHIPMODEL
{
    gcv300  = 0x0300,
    gcv320  = 0x0320,
    gcv350  = 0x0350,
    gcv355  = 0x0355,
    gcv400  = 0x0400,
    gcv410  = 0x0410,
    gcv420  = 0x0420,
    gcv450  = 0x0450,
    gcv500  = 0x0500,
    gcv530  = 0x0530,
    gcv600  = 0x0600,
    gcv700  = 0x0700,
    gcv800  = 0x0800,
    gcv860  = 0x0860,
    gcv880  = 0x0880,
    gcv1000 = 0x1000,
    gcv2000 = 0x2000,
    gcv2100 = 0x2100,
    gcv4000 = 0x4000,
}
gceCHIPMODEL;

/* Chip features. */
typedef enum _gceFEATURE
{
    gcvFEATURE_PIPE_2D = 0,
    gcvFEATURE_PIPE_3D,
    gcvFEATURE_PIPE_VG,
    gcvFEATURE_DC,
    gcvFEATURE_HIGH_DYNAMIC_RANGE,
    gcvFEATURE_MODULE_CG,
    gcvFEATURE_MIN_AREA,
    gcvFEATURE_BUFFER_INTERLEAVING,
    gcvFEATURE_BYTE_WRITE_2D,
    gcvFEATURE_ENDIANNESS_CONFIG,
    gcvFEATURE_DUAL_RETURN_BUS,
    gcvFEATURE_DEBUG_MODE,
    gcvFEATURE_YUY2_RENDER_TARGET,
    gcvFEATURE_FRAGMENT_PROCESSOR,
    gcvFEATURE_2DPE20,
    gcvFEATURE_FAST_CLEAR,
    gcvFEATURE_YUV420_TILER,
    gcvFEATURE_YUY2_AVERAGING,
    gcvFEATURE_FLIP_Y,
    gcvFEATURE_EARLY_Z,
    gcvFEATURE_Z_COMPRESSION,
    gcvFEATURE_MSAA,
    gcvFEATURE_SPECIAL_ANTI_ALIASING,
    gcvFEATURE_SPECIAL_MSAA_LOD,
    gcvFEATURE_422_TEXTURE_COMPRESSION,
    gcvFEATURE_DXT_TEXTURE_COMPRESSION,
    gcvFEATURE_ETC1_TEXTURE_COMPRESSION,
    gcvFEATURE_CORRECT_TEXTURE_CONVERTER,
    gcvFEATURE_TEXTURE_8K,
    gcvFEATURE_SCALER,
    gcvFEATURE_YUV420_SCALER,
    gcvFEATURE_SHADER_HAS_W,
    gcvFEATURE_SHADER_HAS_SIGN,
    gcvFEATURE_SHADER_HAS_FLOOR,
    gcvFEATURE_SHADER_HAS_CEIL,
    gcvFEATURE_SHADER_HAS_SQRT,
    gcvFEATURE_SHADER_HAS_TRIG,
    gcvFEATURE_VAA,
    gcvFEATURE_HZ,
    gcvFEATURE_CORRECT_STENCIL,
    gcvFEATURE_VG20,
    gcvFEATURE_VG_FILTER,
    gcvFEATURE_VG21,
    gcvFEATURE_VG_DOUBLE_BUFFER,
    gcvFEATURE_MC20,
    gcvFEATURE_SUPER_TILED,
    gcvFEATURE_2D_FILTERBLIT_PLUS_ALPHABLEND,
    gcvFEATURE_2D_DITHER,
    gcvFEATURE_2D_A8_TARGET,
    gcvFEATURE_2D_FILTERBLIT_FULLROTATION,
    gcvFEATURE_2D_BITBLIT_FULLROTATION,
    gcvFEATURE_WIDE_LINE,
    gcvFEATURE_FC_FLUSH_STALL,
    gcvFEATURE_FULL_DIRECTFB,
    gcvFEATURE_HALF_FLOAT_PIPE,
    gcvFEATURE_LINE_LOOP,
    gcvFEATURE_2D_YUV_BLIT,
    gcvFEATURE_2D_TILING,
    gcvFEATURE_NON_POWER_OF_TWO,
    gcvFEATURE_3D_TEXTURE,
    gcvFEATURE_TEXTURE_ARRAY,
    gcvFEATURE_TILE_FILLER,
    gcvFEATURE_LOGIC_OP,
    gcvFEATURE_COMPOSITION,
    gcvFEATURE_MIXED_STREAMS,
    gcvFEATURE_2D_MULTI_SOURCE_BLT,
    gcvFEATURE_END_EVENT,
    gcvFEATURE_VERTEX_10_10_10_2,
    gcvFEATURE_TEXTURE_10_10_10_2,
    gcvFEATURE_TEXTURE_ANISOTROPIC_FILTERING,
    gcvFEATURE_TEXTURE_FLOAT_HALF_FLOAT,
    gcvFEATURE_2D_ROTATION_STALL_FIX,
    gcvFEATURE_2D_MULTI_SOURCE_BLT_EX,
    gcvFEATURE_BUG_FIXES10,
    gcvFEATURE_2D_MINOR_TILING,
    /* Supertiled compressed textures are supported. */
    gcvFEATURE_TEX_COMPRRESSION_SUPERTILED,
    gcvFEATURE_FAST_MSAA,
    gcvFEATURE_BUG_FIXED_INDEXED_TRIANGLE_STRIP,
    gcvFEATURE_TEXTURE_TILED_READ,
    gcvFEATURE_DEPTH_BIAS_FIX,
    gcvFEATURE_RECT_PRIMITIVE,
    gcvFEATURE_BUG_FIXES11,
    gcvFEATURE_SUPERTILED_TEXTURE,
    gcvFEATURE_2D_NO_COLORBRUSH_INDEX8
}
gceFEATURE;

/* Chip Power Status. */
typedef enum _gceCHIPPOWERSTATE
{
    gcvPOWER_ON = 0,
    gcvPOWER_OFF,
    gcvPOWER_IDLE,
    gcvPOWER_SUSPEND,
    gcvPOWER_SUSPEND_ATPOWERON,
    gcvPOWER_OFF_ATPOWERON,
    gcvPOWER_IDLE_BROADCAST,
    gcvPOWER_SUSPEND_BROADCAST,
    gcvPOWER_OFF_BROADCAST,
    gcvPOWER_OFF_RECOVERY,
    gcvPOWER_OFF_TIMEOUT,
    gcvPOWER_ON_AUTO
}
gceCHIPPOWERSTATE;

/* CPU cache operations */
typedef enum _gceCACHEOPERATION
{
    gcvCACHE_CLEAN      = 0x01,
    gcvCACHE_INVALIDATE = 0x02,
    gcvCACHE_FLUSH      = gcvCACHE_CLEAN  | gcvCACHE_INVALIDATE,
    gcvCACHE_MEMORY_BARRIER = 0x04
}
gceCACHEOPERATION;

typedef enum _gceVIDMEM_NODE_SHARED_INFO_TYPE
{
    gcvVIDMEM_INFO_GENERIC,
    gcvVIDMEM_INFO_DIRTY_RECTANGLE
}
gceVIDMEM_NODE_SHARED_INFO_TYPE;

/* Surface types. */
typedef enum _gceSURF_TYPE
{
    gcvSURF_TYPE_UNKNOWN = 0,
    gcvSURF_INDEX,
    gcvSURF_VERTEX,
    gcvSURF_TEXTURE,
    gcvSURF_RENDER_TARGET,
    gcvSURF_DEPTH,
    gcvSURF_BITMAP,
    gcvSURF_TILE_STATUS,
    gcvSURF_IMAGE,
    gcvSURF_MASK,
    gcvSURF_SCISSOR,
    gcvSURF_HIERARCHICAL_DEPTH,
    gcvSURF_NUM_TYPES, /* Make sure this is the last one! */

    /* Combinations. */
    gcvSURF_NO_TILE_STATUS = 0x100,
    gcvSURF_NO_VIDMEM      = 0x200, /* Used to allocate surfaces with no underlying vidmem node.
                                       In Android, vidmem node is allocated by another process. */
    gcvSURF_CACHEABLE      = 0x400, /* Used to allocate a cacheable surface */

    gcvSURF_RENDER_TARGET_NO_TILE_STATUS = gcvSURF_RENDER_TARGET
                                         | gcvSURF_NO_TILE_STATUS,

    gcvSURF_DEPTH_NO_TILE_STATUS         = gcvSURF_DEPTH
                                         | gcvSURF_NO_TILE_STATUS,

    /* Supported surface types with no vidmem node. */
    gcvSURF_BITMAP_NO_VIDMEM             = gcvSURF_BITMAP
                                         | gcvSURF_NO_VIDMEM,

    gcvSURF_TEXTURE_NO_VIDMEM            = gcvSURF_TEXTURE
                                         | gcvSURF_NO_VIDMEM,

    /* Cacheable surface types with no vidmem node. */
    gcvSURF_CACHEABLE_BITMAP_NO_VIDMEM   = gcvSURF_BITMAP_NO_VIDMEM
                                         | gcvSURF_CACHEABLE,

    gcvSURF_CACHEABLE_BITMAP             = gcvSURF_BITMAP
                                         | gcvSURF_CACHEABLE,
}
gceSURF_TYPE;

/* Surface formats. */
typedef enum _gceSURF_FORMAT
{
    /* Unknown format. */
    gcvSURF_UNKNOWN             = 0,

    /* Palettized formats. */
    gcvSURF_INDEX1              = 100,
    gcvSURF_INDEX4,
    gcvSURF_INDEX8,

    /* RGB formats. */
    gcvSURF_A2R2G2B2            = 200,
    gcvSURF_R3G3B2,
    gcvSURF_A8R3G3B2,
    gcvSURF_X4R4G4B4,
    gcvSURF_A4R4G4B4,
    gcvSURF_R4G4B4A4,
    gcvSURF_X1R5G5B5,
    gcvSURF_A1R5G5B5,
    gcvSURF_R5G5B5A1,
    gcvSURF_R5G6B5,
    gcvSURF_R8G8B8,
    gcvSURF_X8R8G8B8,
    gcvSURF_A8R8G8B8,
    gcvSURF_R8G8B8A8,
    gcvSURF_G8R8G8B8,
    gcvSURF_R8G8B8G8,
    gcvSURF_X2R10G10B10,
    gcvSURF_A2R10G10B10,
    gcvSURF_X12R12G12B12,
    gcvSURF_A12R12G12B12,
    gcvSURF_X16R16G16B16,
    gcvSURF_A16R16G16B16,
    gcvSURF_A32R32G32B32,
    gcvSURF_R8G8B8X8,
    gcvSURF_R5G5B5X1,
    gcvSURF_R4G4B4X4,

    /* BGR formats. */
    gcvSURF_A4B4G4R4            = 300,
    gcvSURF_A1B5G5R5,
    gcvSURF_B5G6R5,
    gcvSURF_B8G8R8,
    gcvSURF_B16G16R16,
    gcvSURF_X8B8G8R8,
    gcvSURF_A8B8G8R8,
    gcvSURF_A2B10G10R10,
    gcvSURF_X16B16G16R16,
    gcvSURF_A16B16G16R16,
    gcvSURF_B32G32R32,
    gcvSURF_X32B32G32R32,
    gcvSURF_A32B32G32R32,
    gcvSURF_B4G4R4A4,
    gcvSURF_B5G5R5A1,
    gcvSURF_B8G8R8X8,
    gcvSURF_B8G8R8A8,
    gcvSURF_X4B4G4R4,
    gcvSURF_X1B5G5R5,
    gcvSURF_B4G4R4X4,
    gcvSURF_B5G5R5X1,
    gcvSURF_X2B10G10R10,

    /* Compressed formats. */
    gcvSURF_DXT1                = 400,
    gcvSURF_DXT2,
    gcvSURF_DXT3,
    gcvSURF_DXT4,
    gcvSURF_DXT5,
    gcvSURF_CXV8U8,
    gcvSURF_ETC1,

    /* YUV formats. */
    gcvSURF_YUY2                = 500,
    gcvSURF_UYVY,
    gcvSURF_YV12,
    gcvSURF_I420,
    gcvSURF_NV12,
    gcvSURF_NV21,
    gcvSURF_NV16,
    gcvSURF_NV61,
    gcvSURF_YVYU,
    gcvSURF_VYUY,

    /* Depth formats. */
    gcvSURF_D16                 = 600,
    gcvSURF_D24S8,
    gcvSURF_D32,
    gcvSURF_D24X8,

    /* Alpha formats. */
    gcvSURF_A4                  = 700,
    gcvSURF_A8,
    gcvSURF_A12,
    gcvSURF_A16,
    gcvSURF_A32,
    gcvSURF_A1,

    /* Luminance formats. */
    gcvSURF_L4                  = 800,
    gcvSURF_L8,
    gcvSURF_L12,
    gcvSURF_L16,
    gcvSURF_L32,
    gcvSURF_L1,

    /* Alpha/Luminance formats. */
    gcvSURF_A4L4                = 900,
    gcvSURF_A2L6,
    gcvSURF_A8L8,
    gcvSURF_A4L12,
    gcvSURF_A12L12,
    gcvSURF_A16L16,

    /* Bump formats. */
    gcvSURF_L6V5U5              = 1000,
    gcvSURF_V8U8,
    gcvSURF_X8L8V8U8,
    gcvSURF_Q8W8V8U8,
    gcvSURF_A2W10V10U10,
    gcvSURF_V16U16,
    gcvSURF_Q16W16V16U16,

    /* R/RG/RA formats. */
    gcvSURF_R8                  = 1100,
    gcvSURF_X8R8,
    gcvSURF_G8R8,
    gcvSURF_X8G8R8,
    gcvSURF_A8R8,
    gcvSURF_R16,
    gcvSURF_X16R16,
    gcvSURF_G16R16,
    gcvSURF_X16G16R16,
    gcvSURF_A16R16,
    gcvSURF_R32,
    gcvSURF_X32R32,
    gcvSURF_G32R32,
    gcvSURF_X32G32R32,
    gcvSURF_A32R32,
    gcvSURF_RG16,

    /* Floating point formats. */
    gcvSURF_R16F                = 1200,
    gcvSURF_X16R16F,
    gcvSURF_G16R16F,
    gcvSURF_X16G16R16F,
    gcvSURF_B16G16R16F,
    gcvSURF_X16B16G16R16F,
    gcvSURF_A16B16G16R16F,
    gcvSURF_R32F,
    gcvSURF_X32R32F,
    gcvSURF_G32R32F,
    gcvSURF_X32G32R32F,
    gcvSURF_B32G32R32F,
    gcvSURF_X32B32G32R32F,
    gcvSURF_A32B32G32R32F,
    gcvSURF_A16F,
    gcvSURF_L16F,
    gcvSURF_A16L16F,
    gcvSURF_A16R16F,
    gcvSURF_A32F,
    gcvSURF_L32F,
    gcvSURF_A32L32F,
    gcvSURF_A32R32F,

}
gceSURF_FORMAT;

/* Pipes. */
typedef enum _gcePIPE_SELECT
{
    gcvPIPE_INVALID = ~0,
    gcvPIPE_3D      =  0,
    gcvPIPE_2D
}
gcePIPE_SELECT;

/* Hardware type. */
typedef enum _gceHARDWARE_TYPE
{
    gcvHARDWARE_INVALID = 0x00,
    gcvHARDWARE_3D      = 0x01,
    gcvHARDWARE_2D      = 0x02,
    gcvHARDWARE_VG      = 0x04,

    gcvHARDWARE_3D2D    = gcvHARDWARE_3D | gcvHARDWARE_2D
}
gceHARDWARE_TYPE;

#define gcdCHIP_COUNT               3

typedef enum _gceMMU_MODE
{
    gcvMMU_MODE_1K,
    gcvMMU_MODE_4K,
} gceMMU_MODE;

/* User signal command codes. */
typedef enum _gceUSER_SIGNAL_COMMAND_CODES
{
    gcvUSER_SIGNAL_CREATE,
    gcvUSER_SIGNAL_DESTROY,
    gcvUSER_SIGNAL_SIGNAL,
    gcvUSER_SIGNAL_WAIT,
    gcvUSER_SIGNAL_MAP,
    gcvUSER_SIGNAL_UNMAP,
}
gceUSER_SIGNAL_COMMAND_CODES;

/* Event locations. */
typedef enum _gceKERNEL_WHERE
{
    gcvKERNEL_COMMAND,
    gcvKERNEL_VERTEX,
    gcvKERNEL_TRIANGLE,
    gcvKERNEL_TEXTURE,
    gcvKERNEL_PIXEL,
}
gceKERNEL_WHERE;

/* gcdDUMP message type. */
typedef enum _gceDEBUG_MESSAGE_TYPE
{
    gcvMESSAGE_TEXT,
    gcvMESSAGE_DUMP
}
gceDEBUG_MESSAGE_TYPE;

/******************************************************************************\
****************************** Object Declarations *****************************
\******************************************************************************/

typedef struct _gckCONTEXT          * gckCONTEXT;
typedef struct _gcoCMDBUF           * gcoCMDBUF;
typedef struct _gcsSTATE_DELTA      * gcsSTATE_DELTA_PTR;
typedef struct _gcsQUEUE            * gcsQUEUE_PTR;
typedef struct _gcoQUEUE            * gcoQUEUE;
typedef struct _gcsHAL_INTERFACE    * gcsHAL_INTERFACE_PTR;
typedef struct _gcs2D_PROFILE       * gcs2D_PROFILE_PTR;

#endif /* __gc_hal_enum_h_ */
