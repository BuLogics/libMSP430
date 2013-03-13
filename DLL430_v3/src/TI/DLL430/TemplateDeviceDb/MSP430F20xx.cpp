/*
 * MSP430F20xx.cpp
 *
 * Definition MSP430F20xx devices.
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

typedef ClockInfo<GCC_STANDARD, 0x60D7, EmptyEemTimer, TAClkEemClockNames> MSP430F20xx_ClockInfo;

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00> MSP430F20xxIdMask;

template<const char config >
struct MSP430F20xx_Match : Match< IdCode<0x01F2, 0x0, 0, 0, 0, config,0>, MSP430F20xxIdMask> {};

template<class FlashOffset, class FlashSize>
struct MemoryModel : MemoryList<boost::tuple<
			MSP430F2xxx_MainFlashMemory<FlashSize, FlashOffset>, 
			MSP430F2xxx_InfoFlashMemoryInfo, 
			MSP430F2xxx_SystemRamInfo< Size<0x80> >, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_33ff1fffffff>,
			MSP430F2xxx_CPUMemoryInfo,
			MSP430F2xxx_EEMMemoryInfo
		> > {};

typedef MemoryModel< Offset<0xFC00>, Size<0x400> > MemoryModel1;
typedef MemoryModel< Offset<0xF800>, Size<0x800> > MemoryModel2;


typedef Features<BC_2xx, false, true, true, false, false, true> MSP430F20xx_Features;


template<
	const char* description,
	const char config,
	class MemoryModelType
>
struct MSP430F20xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MSP430F20xx_Match<config>,
		LowEemMode,
		MSP430F2xxx_DefaultVoltageTestVpp,
		MSP430F20xx_ClockInfo,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryModelType,
		MSP430F20xx_Features
	> {};

extern const char MSP430F2011[] = "F20x1_G2x0x_G2x1x";
extern const char MSP430F2012[] = "F20x2_G2x2x_G2x3x";
extern const char MSP430F2013[] = "MSP430F20x3";

static const DeviceRegistrator< MSP430F20xx<MSP430F2011, 0x1, MemoryModel2> > regMSP430F2011;
static const DeviceRegistrator< MSP430F20xx<MSP430F2012, 0x2, MemoryModel2> > regMSP430F2012;
static const DeviceRegistrator< MSP430F20xx<MSP430F2013, 0x3, MemoryModel2> > regMSP430F2013;
