/*
 * MSP430F12xx.cpp
 *
 * Definition MSP430F12xx devices.
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

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00> MSP430F12xIdMask;
typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00> MSP430F12x2IdMask;

typedef Match< IdCode<0x23F1, 0x0, 0,0,0,0,0>, MSP430F12xIdMask> MSP430F12x_Match;
typedef Match< IdCode<0x3212, 0x0, 0, 0, 0,0,0>, MSP430F12x2IdMask> MSP430F12x2_Match;

typedef ClockInfo<GCC_NONE, 0x60D7, EmptyEemTimer, EmptyEemClockNames> MSP430F12xx_ClockInfo;

typedef Features<BC_1xx, false, false, false, false, false, false> MSP430F12x_Features;
typedef ExtendedFeatures<false, true, false, false, false, false, false> MSP430F12x_ExtFeatures;

struct FunctionMappingF12x : public FunctionMappingNone
{
	FunctionMappingF12x() {
		ReplacePair(ID_ExecuteFunclet, ID_ExecuteFuncletJtag);
		ReplacePair(ID_GetDcoFrequency, ID_GetDcoFrequencyJtag);
	}
};

template<
	const char* description,
	class MatchType,
	class FunctionMappingType,
	class FlashSizeType,
	class OffsetType,
	class Features = MSP430F1xxx_NoI2C,
	class ExtendedFeatures = NoExtendedFeatures
>
struct MSP430F12xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MatchType, 
		LowEemMode,
		MSP430F1xxx_DefaultVoltageTestVpp,
		MSP430F12xx_ClockInfo,
		FunctionMappingType,
		FuncletMapping1_2xx,
		MemoryList<boost::tuple<
			MSP430F1xxx_MainFlashMemory<FlashSizeType, OffsetType>, 
			MSP430F1xxx_InfoFlashMemoryInfo, 
			MSP430F1xxx_BootFlashMemoryInfo, 
			MSP430F1xxx_SystemRamInfo< Size<0x100> >,
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_33031303ff03>,
			MSP430F1xxx_CPUMemoryInfo,
			MSP430F1xxx_EEMMemoryInfo
		> >, //until C++0x, the space between the brackets is important
		Features,
		ExtendedFeatures
	> {};

extern const char MSP430F123[] = "MSP430F12x";
extern const char MSP430F1232[] = "MSP430F12x2/F11x2";

typedef MSP430F12xx<
	MSP430F123, MSP430F12x_Match,
	FunctionMappingF12x,
	Size<0x2000>, Offset<0xE000>, 
	MSP430F12x_Features, MSP430F12x_ExtFeatures
> MSP430F123_type;

typedef MSP430F12xx<
	MSP430F1232, MSP430F12x2_Match,
	FunctionMappingNone,
	Size<0x2000>, Offset<0xE000> 
> MSP430F1232_type;

static const DeviceRegistrator<MSP430F123_type> regMSP430F123;
static const DeviceRegistrator<MSP430F1232_type> regMSP430F1232;
