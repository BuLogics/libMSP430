/*
 * MSP430F13xx.cpp
 *
 * Definition MSP430F13xx devices.
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

typedef ClockInfo<GCC_NONE, 0x60D7, EmptyEemTimer, EmptyEemClockNames> MSP430F13xx_ClockInfo;

template<
	const char* description,
	const unsigned int versionId,
	const unsigned int fuses,
	class FlashSizeType,
	class OffsetType,
	class RamSize
>
struct MSP430F13xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MSP430F1xxx_Match<versionId, fuses>, 
		MedEemMode,
		MSP430F1xxx_DefaultVoltageNoTestVpp,
		MSP430F13xx_ClockInfo,
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

extern const char MSP430F133[] = "MSP430F133";
extern const char MSP430F135[] = "MSP430F135";

typedef MSP430F13xx<MSP430F133, 0x49F1, 0x5, Size<0x2000>, Offset<0xE000>, Size<0x100> > MSP430F133_type;
typedef MSP430F13xx<MSP430F135, 0x49F1, 0x4, Size<0x4000>, Offset<0xC000>, Size<0x200> > MSP430F135_type;

static const DeviceRegistrator<MSP430F133_type> regMSP430F133;
static const DeviceRegistrator<MSP430F135_type> regMSP430F135;
