/*
 * MSP430F1_2_4xxx_masks.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                                                                                                                                                                                                                                                                         
 */

#ifndef TEMPLATE_DEVICE_DB_MSP430F1_2_4XXX_MASK_H
#define TEMPLATE_DEVICE_DB_MSP430F1_2_4XXX_MASK_H

#if _MSC_VER > 1000
#pragma once
#endif

#include <inttypes.h>
#include "Registration.h"

namespace TI { namespace DLL430 { namespace TemplateDeviceDb{
namespace Memory {

#define SFR_MASK_SIZE 6

extern const uint8_t sfrMaskData_33ff13ffffff[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_33ff1fffffff[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_f330d330c030[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_33801380ffff[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_33031303ff03[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_f380d380c0ff[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_f3b0d3b0c030[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_33801f80ffff[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_338f1f1fffff[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_338f1f8fffff[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_33bf13bfff30[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_330f1f0fffff[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_f3ffd3ffc0ff[SFR_MASK_SIZE];
extern const uint8_t sfrMaskData_f3ffdfffc0ff[SFR_MASK_SIZE];

typedef MemoryMask<sfrMaskData_33ff13ffffff, sizeof(sfrMaskData_33ff13ffffff)> sfrMask_33ff13ffffff;
typedef MemoryMask<sfrMaskData_33ff1fffffff, sizeof(sfrMaskData_33ff1fffffff)> sfrMask_33ff1fffffff;
typedef MemoryMask<sfrMaskData_f330d330c030, sizeof(sfrMaskData_f330d330c030)> sfrMask_f330d330c030;
typedef MemoryMask<sfrMaskData_33801380ffff, sizeof(sfrMaskData_33801380ffff)> sfrMask_33801380ffff;
typedef MemoryMask<sfrMaskData_33031303ff03, sizeof(sfrMaskData_33031303ff03)> sfrMask_33031303ff03;
typedef MemoryMask<sfrMaskData_f380d380c0ff, sizeof(sfrMaskData_f380d380c0ff)> sfrMask_f380d380c0ff;
typedef MemoryMask<sfrMaskData_f3b0d3b0c030, sizeof(sfrMaskData_f3b0d3b0c030)> sfrMask_f3b0d3b0c030;
typedef MemoryMask<sfrMaskData_33801f80ffff, sizeof(sfrMaskData_33801f80ffff)> sfrMask_33801f80ffff;
typedef MemoryMask<sfrMaskData_338f1f1fffff, sizeof(sfrMaskData_338f1f1fffff)> sfrMask_338f1f1fffff;
typedef MemoryMask<sfrMaskData_338f1f8fffff, sizeof(sfrMaskData_338f1f8fffff)> sfrMask_338f1f8fffff;
typedef MemoryMask<sfrMaskData_33bf13bfff30, sizeof(sfrMaskData_33bf13bfff30)> sfrMask_33bf13bfff30;
typedef MemoryMask<sfrMaskData_330f1f0fffff, sizeof(sfrMaskData_330f1f0fffff)> sfrMask_330f1f0fffff;
typedef MemoryMask<sfrMaskData_f3ffd3ffc0ff, sizeof(sfrMaskData_f3ffd3ffc0ff)> sfrMask_f3ffd3ffc0ff;
typedef MemoryMask<sfrMaskData_f3ffdfffc0ff, sizeof(sfrMaskData_f3ffdfffc0ff)> sfrMask_f3ffdfffc0ff;

} //namespace Memory
} //namespace TemplateDeviceDb
}//namespace DLL430
}//namespace TI
#endif //namespace TEMPLATE_DEVICE_DB_MSP430F1_2_4XXX_MASK_H
