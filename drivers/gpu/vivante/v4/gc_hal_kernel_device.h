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

#ifndef __gc_hal_kernel_device_h_
#define __gc_hal_kernel_device_h_

/******************************************************************************\
******************************* gckGALDEVICE Structure *******************************
\******************************************************************************/

typedef struct _gckGALDEVICE
{
    /* Objects. */
    gckOS               os;
    gckKERNEL           kernels[gcdCORE_COUNT];

    /* Attributes. */
    size_t              internalSize;
    gctPHYS_ADDR        internalPhysical;
    void *              internalLogical;
    gckVIDMEM           internalVidMem;
    size_t              externalSize;
    gctPHYS_ADDR        externalPhysical;
    void *              externalLogical;
    gckVIDMEM           externalVidMem;
    gckVIDMEM           contiguousVidMem;
    void *              contiguousBase;
    gctPHYS_ADDR        contiguousPhysical;
    size_t              contiguousSize;
    gctBOOL             contiguousMapped;
    void *              contiguousMappedUser;
    size_t              systemMemorySize;
    gctUINT32           systemMemoryBaseAddress;
    void *              registerBases[gcdCORE_COUNT];
    size_t              registerSizes[gcdCORE_COUNT];
    gctUINT32           baseAddress;
    gctUINT32           requestedRegisterMemBases[gcdCORE_COUNT];
    size_t              requestedRegisterMemSizes[gcdCORE_COUNT];
    gctUINT32           requestedContiguousBase;
    size_t              requestedContiguousSize;

    /* IRQ management. */
    gctINT              irqLines[gcdCORE_COUNT];
    gctBOOL             isrInitializeds[gcdCORE_COUNT];
    gctBOOL             dataReadys[gcdCORE_COUNT];

    /* Thread management. */
    struct task_struct  *threadCtxts[gcdCORE_COUNT];
    struct semaphore    semas[gcdCORE_COUNT];
    gctBOOL             threadInitializeds[gcdCORE_COUNT];
    gctBOOL             killThread;

    /* Signal management. */
    gctINT              signal;

    /* Core mapping */
    gceCORE             coreMapping[8];

    /* States before suspend. */
    gceCHIPPOWERSTATE   statesStored[gcdCORE_COUNT];

    /* Clock management. */
    struct clk          *clk;
    int                 clk_enabled;

    /* Device pointer for dma_alloc_coherent */
    struct device       *dev;
}
* gckGALDEVICE;

typedef struct _gcsHAL_PRIVATE_DATA
{
    gckGALDEVICE        device;
    void *              mappedMemory;
    void *              contiguousLogical;
    /* The process opening the device may not be the same as the one that closes it. */
    gctUINT32           pidOpen;
}
gcsHAL_PRIVATE_DATA, * gcsHAL_PRIVATE_DATA_PTR;

gceSTATUS gckGALDEVICE_Start(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Stop(
    gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Construct(
    IN gctINT IrqLine,
    IN gctUINT32 RegisterMemBase,
    IN size_t RegisterMemSize,
    IN gctINT IrqLine2D,
    IN gctUINT32 RegisterMemBase2D,
    IN size_t RegisterMemSize2D,
    IN gctUINT32 ContiguousBase,
    IN size_t ContiguousSize,
    IN size_t BankSize,
    IN gctINT FastClear,
    IN gctINT Compression,
    IN gctUINT32 PhysBaseAddr,
    IN gctUINT32 PhysSize,
    IN gctINT Signal,
    OUT gckGALDEVICE *Device
    );

gceSTATUS gckGALDEVICE_Destroy(
    IN gckGALDEVICE Device
    );

#endif /* __gc_hal_kernel_device_h_ */
