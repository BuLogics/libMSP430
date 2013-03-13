/*
 * MSP430F16xx.cpp
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

#include "MSP430F1xxx.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

template<class FlashSizeType, class OffsetType>
struct MSP430F16xx_MainFlashMemory : MemoryInfo<
						Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
						FlashSizeType , OffsetType, SegmentSize<0x200>, BankSize<0x10000>, Banks<1>, 
						NoMask> {};


struct MSP430F16xx_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F16xx_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::MCLKpin, Eem::SMCLKpin, Eem::ACLKpin, 
		Eem::ADC12, Eem::FLASH_CTRL, Eem::USART1, Eem::USART0,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::TB, Eem::TA, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430F16xx_EemTimer, StandardEemClockNames> MSP430F16xx_ClockInfo;

template<
	const char* description,
	class MatchType,
	class FlashSizeType,
	class OffsetType,
	class RamSize
>
struct MSP430F16x : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MatchType, 
		HighEemMode,
		MSP430F1xxx_DefaultVoltageNoTestVpp,
		MSP430F16xx_ClockInfo,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryList<boost::tuple<
			MSP430F16xx_MainFlashMemory<FlashSizeType, OffsetType>, 
			MSP430F1xxx_InfoFlashMemoryInfo, 
			MSP430F1xxx_BootFlashMemoryInfo, 
			MSP430F1xxx_SystemRamInfo<RamSize>, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_f330d330c030>,
			MSP430F1xxx_CPUMemoryInfo,
			MSP430F1xxx_EEMMemoryInfo
		> >, //until C++0x, the space between the brackets is important
		MSP430F1xxx_I2C
	> {};

typedef IdCode<0xFFFF, 0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x07> MSP430F161xIdMask;

template<const unsigned int versionId, const unsigned int fuses>
struct MSP430F161x_Match : Match< IdCode<versionId, 0x0, 0,0,0,0, fuses>, MSP430F161xIdMask> {};

template<
	const char* description,
	const unsigned int versionId,
	const unsigned int fuses,
	class FlashSizeType,
	class OffsetType,
	class Ram2Size
>
struct MSP430F161x : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MSP430F161x_Match<versionId, fuses>, 
		HighEemMode, 
		MSP430F1xxx_DefaultVoltageNoTestVpp,
		MSP430F16xx_ClockInfo,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryList<boost::tuple<
			MSP430F16xx_MainFlashMemory<FlashSizeType, OffsetType>, 
			MSP430F1xxx_InfoFlashMemoryInfo, 
			MSP430F1xxx_BootFlashMemoryInfo, 
			MSP430F1xxx_SystemRamInfo< Size<0x800> >,
			MSP430F1xxx_SystemRam2Info<Ram2Size>,
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_f330d330c030>,
			MSP430F1xxx_CPUMemoryInfo,
			MSP430F1xxx_EEMMemoryInfo
		> >, //until C++0x, the space between the brackets is important
		MSP430F1xxx_I2C
> {};

extern const char MSP430F167[] = "MSP430F167";
extern const char MSP430F168[] = "MSP430F168";
extern const char MSP430F169[] = "MSP430F169";
extern const char MSP430F1610[] = "MSP430F1610";
extern const char MSP430F1611[] = "MSP430F1611";
extern const char MSP430F1612[] = "MSP430F1612";

typedef MSP430F16x<MSP430F167, MSP430F1xxx_Match<0x69F1, 0x2>, Size<0x8000>, Offset<0x8000>, Size<0x400> > MSP430F167_type;
typedef MSP430F16x<MSP430F168, MSP430F1xxx_Match<0x69F1, 0x1>, Size<0xC000>, Offset<0x4000>, Size<0x800> > MSP430F168_type;
typedef MSP430F16x<MSP430F169, MSP430F1xxx_Match<0x69F1, 0x0>, Size<0xEF00>, Offset<0x1100>, Size<0x800> > MSP430F169_type;
typedef MSP430F16x<MSP430F169, MSP430F161x_Match<0x6CF1, 0x7>, Size<0xEF00>, Offset<0x1100>, Size<0x800> > MSP430F169_1611based_type;

typedef MSP430F161x<MSP430F1610, 0x6CF1, 0x3, Size<0x8000>, Offset<0x8000>, Size<0x1400> > MSP430F1610_type;
typedef MSP430F161x<MSP430F1611, 0x6CF1, 0x0, Size<0xC000>, Offset<0x4000>, Size<0x2800> > MSP430F1611_type;
typedef MSP430F161x<MSP430F1612, 0x6CF1, 0x6, Size<0xDB00>, Offset<0x2500>, Size<0x1400> > MSP430F1612_type;

static const DeviceRegistrator<MSP430F167_type> regMSP430F167;
static const DeviceRegistrator<MSP430F168_type> regMSP430F168;
static const DeviceRegistrator<MSP430F169_type> regMSP430F169;

static const DeviceRegistrator<MSP430F1610_type> regMSP430F1610;
static const DeviceRegistrator<MSP430F1611_type> regMSP430F1611;
static const DeviceRegistrator<MSP430F1612_type> regMSP430F1612;
static const DeviceRegistrator<MSP430F169_1611based_type> regMSP430F169_1611based;
