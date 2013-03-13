/*
 * MSP430F41xx.cpp
 *
 * Definition MSP430F41xx devices.
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

#include "MSP430F4xxx.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;


typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00> MSP430F41xIdMask;
typedef IdCode<0xFFFF, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00> MSP430F413IdMask;
typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01> MSP430F41x2IdMask;

typedef Match< IdCode<0x27F4, 0x0, 0,0,0, 'W', 0x0>, MSP430F41xIdMask> MSP430F41x_Match;
typedef Match< IdCode<0x13F4, 0x0, 0x02,0x40,0, 0x0, 0x0>, MSP430F413IdMask> MSP430F413_Match;

template<const unsigned int fuses>
struct MSP430F41x2_Match : Match< IdCode<0x5241, 0x0, 0,0,0,0, fuses>, MSP430F41x2IdMask> {};



struct MSP430F41x2_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F41x2_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::MCLKpin, Eem::SMCLKpin, Eem::ACLKpin, 
		Eem::Empty, Eem::FLASH_CTRLER, Eem::Empty, Eem::USCI0,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::BT_RTC, 
		Eem::Empty, Eem::TB3, Eem::TA3, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_STANDARD, 0x60D7, EmptyEemTimer, TAClkEemClockNames> MSP430F41x_ClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430F41x2_EemTimer, StandardEemClockNames> MSP430F41x2_ClockInfo;


typedef VoltageInfo<1800, 3600, 2200, 2700, 6000, 7000, true> MSP430F41x2_VoltageInfo;


typedef MemoryInfo<
	Name::information, FlashType, Mapped, Protectable, Bits16Type, 
	Size<0x100> , Offset<0x1000>, SegmentSize<0x40>, BankSize<0x40>, Banks<4>,
	NoMask, MemoryCreator<InformationFlashAccess>
> MSP430F41x2_InfoFlashMemoryInfo;

typedef Features<FLLPLUS, false, false, false, true, false, false> MSP430F413_Features;
typedef ExtendedFeatures<false, true, false, false, true, false, false> MSP430F413_ExtFeatures;


struct FunctionMappingF413 : public FunctionMappingNone
{
	FunctionMappingF413() {
		ReplacePair(ID_ExecuteFunclet, ID_ExecuteFuncletJtag);
		ReplacePair(ID_GetDcoFrequency, ID_GetFllFrequencyJtag);
	}
};


template<
	const char* description,
	const Psa PsaType,
	class MatchType,
	class VoltageType,
	class ClockInfoType,
	class FunctionMapping,
	class FlashOffset,
	class FlashSize,
	class RamSize,
	class LcdMemorySize,
	class InfoMemoryType,
	class SFRmaskType,
	class Features = MSP430F4xxx_DefaultFeatures,
	class ExtendedFeatures = NoExtendedFeatures
>
struct MSP430F41xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		PsaType, 
		MatchType, 
		LowEemMode,
		VoltageType,
		ClockInfoType,
		FunctionMapping,
		FuncletMapping4xx,
		MemoryList<boost::tuple<
			MSP430F4xxx_MainFlashMemory<FlashSize, FlashOffset>, 
			InfoMemoryType, 
			MSP430F4xxx_BootFlashMemoryInfo, 
			MSP430F4xxx_LcdMemoryInfo< LcdMemorySize >,
			MSP430F4xxx_SystemRamInfo<RamSize>, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<SFRmaskType>,
			MSP430F4xxx_CPUMemoryInfo,
			MSP430F4xxx_EEMMemoryInfo
		> >,
		Features,
		ExtendedFeatures
	> {};

template<const char* description, class FlashOffset, class FlashSize, class RamSize>
struct MSP430FW42x_F41x : MSP430F41xx<description,
								regular,
								MSP430F41x_Match, 
								MSP430F4xxx_DefaultVoltageNoTestVpp, 
								MSP430F41x_ClockInfo,
								FunctionMapping4xx,
								FlashOffset, FlashSize, RamSize, Size<0xD>,
								MSP430F4xxx_InfoFlashMemoryInfo,
								sfrMask_33801380ffff> {};

template<const char* description, class FlashOffset, class FlashSize, class RamSize>
struct MSP430F41x : MSP430F41xx<description,
								regular,
								MSP430F413_Match, 
								MSP430F4xxx_DefaultVoltageNoTestVpp, 
								MSP430F41x_ClockInfo,
								FunctionMappingF413,
								FlashOffset, FlashSize, RamSize, Size<0xD>,
								MSP430F4xxx_InfoFlashMemoryInfo,
								sfrMask_33801380ffff,
								MSP430F413_Features,
								MSP430F413_ExtFeatures> {};

template<const char* description, const unsigned int fuses, class FlashOffset, class FlashSize, class RamSize>
struct MSP430F41x2 : MSP430F41xx<description,
								enhanced,
								MSP430F41x2_Match<fuses>,
								MSP430F41x2_VoltageInfo,
								MSP430F41x2_ClockInfo,
								FunctionMapping4xx,
								FlashOffset, FlashSize, RamSize, Size<0x20>,
								MSP430F41x2_InfoFlashMemoryInfo,
								sfrMask_338f1f8fffff,
								MSP430F46_7xx_Features > {};

extern const char MSP430F413C[] = "MSP430F41x";
extern const char MSP430F417[] = "MSP430FW42x/F41x";
extern const char MSP430F4132[] = "MSP430F4132";
extern const char MSP430F4152[] = "MSP430F4152";

typedef MSP430F41x<MSP430F413C, Offset<0xE000>, Size<0x2000>, Size<0x100> > MSP430F413_Type;
typedef MSP430FW42x_F41x<MSP430F417, Offset<0x8000>, Size<0x8000>, Size<0x400> > MSP430F417_Type;

typedef MSP430F41x2<MSP430F4132, 0x1, Offset<0xE000>, Size<0x2000>, Size<0x200> > MSP430F4132_Type;
typedef MSP430F41x2<MSP430F4152, 0x0, Offset<0xC000>, Size<0x4000>, Size<0x200> > MSP430F4152_Type;

static const DeviceRegistrator<MSP430F413_Type> regMSP430F413;
static const DeviceRegistrator<MSP430F417_Type> regMSP430F417;
static const DeviceRegistrator<MSP430F4132_Type> regMSP430F4132;
static const DeviceRegistrator<MSP430F4152_Type> regMSP430F4152;
