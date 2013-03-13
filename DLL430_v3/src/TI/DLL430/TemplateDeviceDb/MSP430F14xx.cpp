/*
 * MSP430F14xx.cpp
 *
 * Definition MSP430F14xx devices.
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

#include "MSP430F1xxx.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

typedef ClockInfo<GCC_NONE, 0x60D7, EmptyEemTimer, EmptyEemClockNames> MSP430F14xx_ClockInfo;

template<
	const char* description,
	const unsigned int versionId,
	const unsigned int fuses,
	class FlashSizeType,
	class OffsetType,
	class RamSize
>
struct MSP430F14xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MSP430F1xxx_Match<versionId, fuses>, 
		MedEemMode,
		MSP430F1xxx_DefaultVoltageNoTestVpp,
		MSP430F14xx_ClockInfo,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryList<boost::tuple<
			MSP430F1xxx_MainFlashMemory<FlashSizeType, OffsetType>, 
			MSP430F1xxx_InfoFlashMemoryInfo, 
			MSP430F1xxx_BootFlashMemoryInfo, 
			MSP430F1xxx_SystemRamInfo<RamSize>, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_f330d330c030>,
			MSP430F1xxx_CPUMemoryInfo,
			MSP430F1xxx_EEMMemoryInfo
		> >, //until C++0x, the space between the brackets is important
		MSP430F1xxx_NoI2C
	> {};

extern const char MSP430F147[] = "MSP430F147";
extern const char MSP430F148[] = "MSP430F148";
extern const char MSP430F149[] = "MSP430F149";
extern const char MSP430F1471[] = "MSP430F1471";
extern const char MSP430F1481[] = "MSP430F1481";
extern const char MSP430F1491[] = "MSP430F1491";

//            description, versionId, fuses, flash size,  flash offset,  ram size
typedef MSP430F14xx<MSP430F147, 0x49F1, 0x2, Size<0x8000>, Offset<0x8000>, Size<0x400> > MSP430F147_type;
typedef MSP430F14xx<MSP430F148, 0x49F1, 0x1, Size<0xC000>, Offset<0x4000>, Size<0x800> > MSP430F148_type;
typedef MSP430F14xx<MSP430F149, 0x49F1, 0x0, Size<0xEF00>, Offset<0x1100>, Size<0x800> > MSP430F149_type;

typedef MSP430F14xx<MSP430F1471, 0x49F1, 0x3, Size<0x8000>, Offset<0x8000>, Size<0x1400> > MSP430F1471_type;
typedef MSP430F14xx<MSP430F1481, 0x49F1, 0x0, Size<0xC000>, Offset<0x4000>, Size<0x2800> > MSP430F1481_type;
typedef MSP430F14xx<MSP430F1491, 0x49F1, 0x6, Size<0xEF00>, Offset<0x1100>, Size<0x1400> > MSP430F1491_type;

static const DeviceRegistrator<MSP430F147_type> regMSP430F147;
static const DeviceRegistrator<MSP430F148_type> regMSP430F148;
static const DeviceRegistrator<MSP430F149_type> regMSP430F149;

static const DeviceRegistrator<MSP430F1471_type> regMSP430F1471;
static const DeviceRegistrator<MSP430F1481_type> regMSP430F1481;
static const DeviceRegistrator<MSP430F1491_type> regMSP430F1491;
