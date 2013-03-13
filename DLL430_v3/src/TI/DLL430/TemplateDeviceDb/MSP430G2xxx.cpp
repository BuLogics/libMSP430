/*
 * MSP430G2xxxx.cpp
 *
 * Definition MSP430Gxxxx devices.
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

#include "MSP430F2xxx.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

typedef ClockInfo<GCC_STANDARD, 0x60D7, EmptyEemTimer, TAClkEemClockNames> MSP430G2xxx_ClockInfo;

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00> MSP430G2xx2_3IdMask;

typedef Match< IdCode<0x5224, 0,0,0,0,0,0>, MSP430G2xx2_3IdMask> MSP430G2xx2_Match;
typedef Match< IdCode<0x5325, 0,0,0,0,0,0>, MSP430G2xx2_3IdMask> MSP430G2xx3_Match;

typedef Features<BC_2xx, false, true, true, false, false, true> MSP430G24_5xx_Features;

template<
	const char* description,
	class MatchType,
	const unsigned int FlashSize,
	const unsigned int FlashOffset,
	const unsigned int RamSize,
	class Features = DefaultFeatures
>
struct MSP430G2xxx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MatchType,
		LowEemMode,
		MSP430F2xxx_DefaultVoltageTestVpp,
		MSP430G2xxx_ClockInfo,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryList<boost::tuple<
			MSP430F2xxx_MainFlashMemory< Size<FlashSize>, Offset<FlashOffset> >, 
			MSP430F2xxx_InfoFlashMemoryInfo, 
			MSP430F2xxx_SystemRamInfo< Size<RamSize> >, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_33ff1fffffff>,
			MSP430F2xxx_CPUMemoryInfo,
			MSP430F2xxx_EEMMemoryInfo
		> >, //until C++0x, the space between the brackets is important
		Features
	> {};


extern const char MSP430G2452[] = "MSP430G2xx2";
extern const char MSP430G2553[] = "MSP430G2xx3";

// G2xx1 covered in MSP430F20xx.cpp

static const DeviceRegistrator< 
	MSP430G2xxx<MSP430G2452, MSP430G2xx2_Match, 0x2000, 0xE000, 0x100, MSP430G24_5xx_Features> 
> regMSP430G2452;

static const DeviceRegistrator< 
	MSP430G2xxx<MSP430G2553, MSP430G2xx3_Match, 0x4000, 0xC000, 0x200, MSP430G24_5xx_Features> 
> regMSP430G2553;
