/*
 * MSP430F15x.cpp
 *
 * Definition MSP430F15x devices.
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

struct MSP430F15x_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F15x_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::MCLKpin, Eem::SMCLKpin, Eem::ACLKpin, 
		Eem::ADC12, Eem::FLASH_CTRL, Eem::Empty, Eem::USART0,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::TB, Eem::TA, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430F15x_EemTimer, StandardEemClockNames> MSP430F15x_ClockInfo;

template<
	const char* description,
	const unsigned int versionId,
	const unsigned int fuses,
	class FlashSizeType,
	class OffsetType,
	class RamSize
>
struct MSP430F15x : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MSP430F1xxx_Match<versionId, fuses>, 
		HighEemMode,
		MSP430F1xxx_DefaultVoltageNoTestVpp,
		MSP430F15x_ClockInfo,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryList<boost::tuple<
			MSP430F1xxx_MainFlashMemory<FlashSizeType, OffsetType>, 
			MSP430F1xxx_InfoFlashMemoryInfo, 
			MSP430F1xxx_BootFlashMemoryInfo, 
			MSP430F1xxx_SystemRamInfo<RamSize>, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_f3ffd3ffc0ff>,
			MSP430F1xxx_CPUMemoryInfo,
			MSP430F1xxx_EEMMemoryInfo
		> >, //until C++0x, the space between the brackets is important
		MSP430F1xxx_I2C
	> {};

extern const char MSP430F155[] = "MSP430F155";
extern const char MSP430F156[] = "MSP430F156";
extern const char MSP430F157[] = "MSP430F157";

typedef MSP430F15x<MSP430F155, 0x69F1, 0x6, Size<0x4000>, Offset<0xC000>, Size<0x200> > MSP430F155_type;
typedef MSP430F15x<MSP430F156, 0x69F1, 0x5, Size<0x6000>, Offset<0xA000>, Size<0x400> > MSP430F156_type;
typedef MSP430F15x<MSP430F157, 0x69F1, 0x4, Size<0x8000>, Offset<0x8000>, Size<0x400> > MSP430F157_type;

static const DeviceRegistrator<MSP430F155_type> regMSP430F155;
static const DeviceRegistrator<MSP430F156_type> regMSP430F156;
static const DeviceRegistrator<MSP430F157_type> regMSP430F157;
