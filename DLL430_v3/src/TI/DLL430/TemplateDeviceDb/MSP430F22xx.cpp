/*
 * MSP430F22xx.cpp
 *
 * Definition MSP430F22xx devices.
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

typedef ClockInfo<GCC_STANDARD, 0x60D7, EmptyEemTimer, TAClkEemClockNames> MSP430F22xx_ClockInfo;

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07> MSP430F22xxIdMask;

template<const unsigned int fuses >
struct MSP430F22xx_Match : Match< IdCode<0x27F2, 0, 0, 0, 0, 0, fuses>, MSP430F22xxIdMask> {};


template<class FlashOffset, class FlashSize, class RamSize>
struct MemoryModel : MemoryList<boost::tuple<
			MSP430F2xxx_MainFlashMemory<FlashSize, FlashOffset>, 
			MSP430F2xxx_InfoFlashMemoryInfo, 
			MSP430F2xxx_BootFlashMemoryInfo, 
			MSP430F2xxx_SystemRamInfo< RamSize >, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_330f1f0fffff>,
			MSP430F2xxx_CPUMemoryInfo,
			MSP430F2xxx_EEMMemoryInfo
		> > {};

typedef MemoryModel< Offset<0xE000>, Size<0x2000>, Size<0x200> > MemoryModel3;
typedef MemoryModel< Offset<0xC000>, Size<0x4000>, Size<0x200> > MemoryModel5;
typedef MemoryModel< Offset<0x8000>, Size<0x8000>, Size<0x400> > MemoryModel7;

template<
	const char* description,
	const unsigned int fuses,
	class MemoryModel
>
struct MSP430F22xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		enhanced, 
		MSP430F22xx_Match<fuses>,
		LowEemMode,
		MSP430F2xxx_DefaultVoltageTestVpp,
		MSP430F22xx_ClockInfo,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryModel,
		DefaultFeatures,
		ExtendedFeatures<false, false, false, false, false, false, true>
	> {};

extern const char MSP430F2232[] = "MSP430F2232";
extern const char MSP430F2252[] = "MSP430F2252";
extern const char MSP430F2272[] = "MSP430F2272";

extern const char MSP430F2234[] = "MSP430F2234";
extern const char MSP430F2254[] = "MSP430F2254";
extern const char MSP430F2274[] = "MSP430F2274";

static const DeviceRegistrator< MSP430F22xx<MSP430F2232, 0x6, MemoryModel3> > regMSP430F2232;
static const DeviceRegistrator< MSP430F22xx<MSP430F2252, 0x5, MemoryModel5> > regMSP430F2252;
static const DeviceRegistrator< MSP430F22xx<MSP430F2272, 0x4, MemoryModel7> > regMSP430F2272;
static const DeviceRegistrator< MSP430F22xx<MSP430F2234, 0x2, MemoryModel3> > regMSP430F2234;
static const DeviceRegistrator< MSP430F22xx<MSP430F2254, 0x1, MemoryModel5> > regMSP430F2254;
static const DeviceRegistrator< MSP430F22xx<MSP430F2274, 0x0, MemoryModel7> > regMSP430F2274;
