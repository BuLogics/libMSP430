/*
 * MSP430F21xx.cpp
 *
 * Definition MSP430F21xx devices.
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

typedef ClockInfo<GCC_STANDARD, 0x60D7, EmptyEemTimer, TAClkEemClockNames> MSP430F21xx_ClockInfo;

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00> MSP430F21x1IdMask;
typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x03> MSP430F21x2IdMask;

typedef Match< IdCode<0x13F2, 0, 0, 0, 0, 0x1, 0>, MSP430F21x1IdMask> MSP430F21x1_Match;

template<const unsigned int fuses >
struct MSP430F21x2_Match : Match< IdCode<0x13F2, 0, 0, 0, 0, 0x2, fuses>, MSP430F21x2IdMask> {};


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

template<
	const char* description,
	const unsigned int FlashOffset,
	const unsigned int FlashSize,
	const unsigned int RamSize
>
struct MSP430F21x1 : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MSP430F21x1_Match,
		LowEemMode,
		MSP430F2xxx_DefaultVoltageTestVpp,
		MSP430F21xx_ClockInfo,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryModel< Offset<FlashOffset>, Size<FlashSize>, Size<RamSize> >
	> {};


template<
	const char* description,
	const unsigned int fuses,
	const unsigned int FlashOffset,
	const unsigned int FlashSize,
	const unsigned int RamSize
>
struct MSP430F21x2 : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		enhanced, 
		MSP430F21x2_Match<fuses>,
		LowEemMode,
		MSP430F2xxx_DefaultVoltageTestVpp,
		MSP430F21xx_ClockInfo,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryModel< Offset<FlashOffset>, Size<FlashSize>, Size<RamSize> >
	> {};

extern const char MSP430F2131[] = "MSP430F21x1";

extern const char MSP430F2112[] = "MSP430F2112";
extern const char MSP430F2122[] = "MSP430F2122";
extern const char MSP430F2132[] = "MSP430F2132";

static const DeviceRegistrator< MSP430F21x1<MSP430F2131, 0xE000, 0x2000, 0x100> > regMSP430F2131;

static const DeviceRegistrator< MSP430F21x2<MSP430F2112, 0x2, 0xF800, 0x800, 0x100> > regMSP430F2112;
static const DeviceRegistrator< MSP430F21x2<MSP430F2122, 0x1, 0xF000, 0x1000, 0x200> > regMSP430F2122;
static const DeviceRegistrator< MSP430F21x2<MSP430F2132, 0x0, 0xE000, 0x2000, 0x200> > regMSP430F2132;
