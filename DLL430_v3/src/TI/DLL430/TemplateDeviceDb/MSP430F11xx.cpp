/*
 * MSP430F11xx.cpp
 *
 * Definition MSP430F11xx devices.
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

typedef ClockInfo<GCC_NONE, 0x60D7, EmptyEemTimer, EmptyEemClockNames> MSP430F11xx_ClockInfo;

typedef IdCode<0xFFFF, 0x00, 0xFF, 0xFF, 0xFFFF, 0x00, 0x00> MSP430F11x1DIdMask;
typedef IdCode<0xFFFF, 0x00, 0xFF, 0x00, 0xFFFF, 0x00, 0x00> MSP430F11x1AIdMask;
typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00> MSP430F11x2IdMask;

typedef Match< IdCode<0x12F1, 0x0, 0x10, 0x40, 0x0, 0,0>, MSP430F11x1DIdMask> MSP430F11x1D_Match;
typedef Match< IdCode<0x12F1, 0x0, 0x13, 0, 0x0, 0,0>, MSP430F11x1AIdMask> MSP430F11x1A_Match;
typedef Match< IdCode<0x3211, 0x0, 0,0,0,0,0>, MSP430F11x2IdMask> MSP430F11x2_Match;

template<
	const char* description,
	class MatchType,
	class FlashSizeType,
	class OffsetType,
	class RamSize,
	class InfoMemoryType = MSP430F1xxx_InfoFlashMemoryInfo
>
struct MSP430F11xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MatchType,
		LowEemMode,
		MSP430F1xxx_DefaultVoltageTestVpp,
		MSP430F11xx_ClockInfo,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryList<boost::tuple<
			MSP430F1xxx_MainFlashMemory<FlashSizeType, OffsetType>, 
			MSP430F1xxx_InfoFlashMemoryInfo, 
			MSP430F1xxx_BootFlashMemoryInfo, 
			MSP430F1xxx_SystemRamInfo<RamSize>,
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_33ff13ffffff>,
			MSP430F1xxx_CPUMemoryInfo,
			MSP430F1xxx_EEMMemoryInfo
		> >, //until C++0x, the space between the brackets is important
		Features<BC_1xx, false, false, false, false, false, false>
	> {};

extern const char MSP430F1121D[] = "MSP430F11x1";
extern const char MSP430F1121A[] = "MSP430F11x1A";
extern const char MSP430F1132[] = "MSP430F11x2";

typedef MSP430F11xx<MSP430F1121D, MSP430F11x1D_Match, Size<0x1000>, Offset<0xF000>, Size<0x100> > MSP430F1121D_type;
typedef MSP430F11xx<MSP430F1121A, MSP430F11x1A_Match, Size<0x1000>, Offset<0xF000>, Size<0x100> > MSP430F1121A_type;
typedef MSP430F11xx<MSP430F1132, MSP430F11x2_Match, Size<0x2000>, Offset<0xE000>, Size<0x100> > MSP430F1132_type;

static const DeviceRegistrator<MSP430F1121D_type> regMSP430F1121D;
static const DeviceRegistrator<MSP430F1121A_type> regMSP430F1121A;
static const DeviceRegistrator<MSP430F1132_type> regMSP430F1132;
