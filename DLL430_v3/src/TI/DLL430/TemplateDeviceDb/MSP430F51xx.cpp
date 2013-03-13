/*
 * MSP430F51xx.cpp
 *
 * Definition MSP430F51xx devices.
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

#include "MSP430F5xxx.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

template<class FlashSizeType, class OffsetType>
struct MSP430F51xx_MainFlashMemory : MemoryInfo<
										Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
										FlashSizeType , OffsetType, SegmentSize<0x200>, 
										BankSize<0x8000>, Banks<1>, NoMask> {};

struct MSP430F51xx_LargeEemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F51xx_LargeEemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_B, 
		Eem::ADC10_A, Eem::RTC, Eem::Empty, Eem::Empty,
		Eem::USCI1, Eem::USCI0, Eem::Empty, Eem::TD3_1,
		Eem::TD3_0, Eem::TA3_0, Eem::Empty, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct MSP430F51xx_SmallEemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F51xx_SmallEemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_B, 
		Eem::Empty, Eem::RTC, Eem::Empty, Eem::Empty,
		Eem::USCI1, Eem::USCI0, Eem::Empty, Eem::TD3_1,
		Eem::TD3_0, Eem::TA3_0, Eem::Empty, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430F51xx_SmallEemTimer, EmptyEemClockNames> MSP430F51xx_SmallClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430F51xx_LargeEemTimer, EmptyEemClockNames> MSP430F51xx_LargeClockInfo;


template<
	const char* description,
	const unsigned int versionId,
	const unsigned int subVersionId,
	class ClockInfo,
	class FlashSizeType,
	class OffsetType,
	class RamSize
>
struct MSP430F51xx : Device<
						description, 
						ObjectId<0>,
						DefaultBits20Type, 
						regular, 
						MSP430F5xxx_Match<versionId,subVersionId>, 
						SmallEemMode, 
						MSP430F5xxx_DefaultVoltageTestVpp,
						ClockInfo,
						FunctionMappingXv2,
						FuncletMappingXv2,
						MemoryList<boost::tuple<
							MSP430F51xx_MainFlashMemory<FlashSizeType, OffsetType>, 
							MSP430F5xxx_InfoFlashMemoryInfo, 
							MSP430F5xxx_BootFlashMemoryInfo, 
							MSP430F5xxx_BootCodeMemoryInfo,
							MSP430F5xxx_SystemRamInfo<RamSize>, 
							MSP430F5xxx_peripherl16lbitMemoryInfo, 
							MSP430F5xxx_CPUMemoryInfo, 
							MSP430F5xxx_EEMMemoryInfo
						> >, //until C++0x, the space between the brackets is important
						MSP430F5xxx_Features,
						MSP430F5xxx_ExtFeatures
					> {};


extern const char MSP430F5132[] = "MSP430F5132";
extern const char MSP430F5152[] = "MSP430F5152";
extern const char MSP430F5172[] = "MSP430F5172";

extern const char MSP430F5131[] = "MSP430F5131";
extern const char MSP430F5151[] = "MSP430F5151";
extern const char MSP430F5171[] = "MSP430F5171";


//description, versionId, clock info, flash size, flash offset, ram size
typedef MSP430F51xx<MSP430F5132, 0x8028, 0x0000, MSP430F51xx_LargeClockInfo, Size<0x2000>, Offset<0xE000>, Size<0x400> > MSP430F5132_type;
typedef MSP430F51xx<MSP430F5152, 0x802C, 0x0000, MSP430F51xx_LargeClockInfo, Size<0x4000>, Offset<0xC000>, Size<0x800> > MSP430F5152_type;
typedef MSP430F51xx<MSP430F5172, 0x8030, 0x0000, MSP430F51xx_LargeClockInfo, Size<0x8000>, Offset<0x8000>, Size<0x800> > MSP430F5172_type;

//description, versionId, clock info, flash size, flash offset, ram size
typedef MSP430F51xx<MSP430F5131, 0x8026, 0x0000, MSP430F51xx_SmallClockInfo, Size<0x2000>, Offset<0xE000>, Size<0x400> > MSP430F5131_type;
typedef MSP430F51xx<MSP430F5151, 0x802A, 0x0000, MSP430F51xx_SmallClockInfo, Size<0x4000>, Offset<0xC000>, Size<0x800> > MSP430F5151_type;
typedef MSP430F51xx<MSP430F5171, 0x802E, 0x0000, MSP430F51xx_SmallClockInfo, Size<0x8000>, Offset<0x8000>, Size<0x800> > MSP430F5171_type;

static const DeviceRegistrator<MSP430F5132_type> regMSP430F5132;
static const DeviceRegistrator<MSP430F5152_type> regMSP430F5152;
static const DeviceRegistrator<MSP430F5172_type> regMSP430F5172;

static const DeviceRegistrator<MSP430F5131_type> regMSP430F5131;
static const DeviceRegistrator<MSP430F5151_type> regMSP430F5151;
static const DeviceRegistrator<MSP430F5171_type> regMSP430F5171;
