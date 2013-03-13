/*
 * MSP430F64xx.cpp
 *
 * Definition MSP430F64xx devices.
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

#include "UsbTypes.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;


const ClockPair MSP430F64xxTimerTA3_1 = {"Timer1_A3", 57};
const ClockPair MSP430F64xxTimerTA3_2 = {"Timer2_A3", 58};


struct MSP430F64xx_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F64xx_EemTimer(const ClockPair& dac, const ClockPair& usci3, const ClockPair& usci2)
		  : EemTimerImpl(
			Eem::Empty, Eem::LCD_B,	dac,		Eem::COMP_B, 
			Eem::ADC12_A,Eem::RTC,	usci3,		usci2,
			Eem::USCI1, Eem::USCI0, Eem::Empty, Eem::TB7_0,
			MSP430F64xxTimerTA3_2, MSP430F64xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty)
	{}
};


struct MSP430F6433_35_EemTimer : MSP430F64xx_EemTimer
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F6433_35_EemTimer() : MSP430F64xx_EemTimer(Eem::Empty, Eem::USCI3, Eem::USCI2)	{}		
};

struct MSP430F6436_38_EemTimer : MSP430F64xx_EemTimer
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F6436_38_EemTimer() : MSP430F64xx_EemTimer(Eem::DAC12_0, Eem::USCI3, Eem::USCI2) {}		
};

struct MSP430F645x_EemTimer : MSP430F64xx_EemTimer
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F645x_EemTimer() : MSP430F64xx_EemTimer(Eem::Empty, Eem::Empty, Eem::Empty) {}		
};

typedef ClockInfo<GCC_EXTENDED, 0x040F, MSP430F6433_35_EemTimer, EmptyEemClockNames> MSP430F6433_35_ClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x040F, MSP430F6436_38_EemTimer, EmptyEemClockNames> MSP430F6436_38_ClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x040F, MSP430F645x_EemTimer, EmptyEemClockNames> MSP430F645x_ClockInfo;


template<const unsigned int nrBanks>
struct MSP430F64xx_MainFlashMemory : MemoryInfo<
									Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
									Size<nrBanks*0x10000> , Offset<0x8000>, SegmentSize<0x200>, 
									BankSize<0x10000>, Banks<nrBanks>, NoMask> {};


template<const unsigned int nrBanks>
struct MSP430F645x_MainFlashMemory : MemoryInfo<
									Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
									Size<nrBanks*0x20000> , Offset<0x8000>, SegmentSize<0x200>, 
									BankSize<0x20000>, Banks<nrBanks>, NoMask> {};



template<
	const char* description,
	const unsigned int versionId,
	typename ClockInfo,
	const unsigned int nrBanks,
	class RamSizeType
>
struct MSP430F64xx : Device<
							description, 
							ObjectId<0>,
							DefaultBits20Type,
							enhanced, 
							MSP430F5xxx_Match<versionId>, 
							LargeEemMode,
							MSP430F5xxx_DefaultVoltageTestVpp,
							ClockInfo,
							FunctionMappingXv2,
							FuncletMappingXv2WordMode,
							MemoryList<boost::tuple<
								MSP430F64xx_MainFlashMemory<nrBanks>,
								MSP430F5xxx_InfoFlashMemoryInfo, 
								MSP430F5xxx_BootFlashMemoryInfo, 
								MSP430F5xxx_BootCodeMemoryInfo,
								UsbTypeRamInfo,
								UsbTypeSystemRamInfo< RamSizeType, Banks<1> >,
								MSP430F5xxx_peripherl16lbitMemoryInfo, 
								MSP430F5xxx_CPUMemoryInfo, 
								MSP430F5xxx_EEMMemoryInfo
							> >, //until C++0x, the space between the brackets is important
							MSP430F5xxx_Features,
							MSP430F5xxx_ExtFeatures
						> {};


template<
	const char* description,
	const unsigned int versionId,
	typename ClockInfo,
	const unsigned int nrBanks,
	class RamSizeType,
	class Ram2OffsetType,
	class Ram2SizeType
>
struct MSP430F645x : Device<
							description, 
							ObjectId<0>,
							DefaultBits20Type,
							enhanced, 
							MSP430F5xxx_Match<versionId>, 
							LargeEemMode,
							MSP430F5xxx_DefaultVoltageTestVpp,
							ClockInfo,
							FunctionMappingXv2,
							FuncletMappingXv2WordMode,
							MemoryList<boost::tuple<
								MSP430F645x_MainFlashMemory<nrBanks>,
								MSP430F5xxx_InfoFlashMemoryInfo, 
								MSP430F5xxx_BootFlashMemoryInfo, 
								MSP430F5xxx_BootCodeMemoryInfo,
								UsbTypeRamInfo,
								UsbTypeSystemRamInfo< RamSizeType, Banks<1> >,
								MSP430F5xxx_SystemRam2Info<Ram2OffsetType, Ram2SizeType>,
								MSP430F5xxx_peripherl16lbitMemoryInfo, 
								MSP430F5xxx_CPUMemoryInfo, 
								MSP430F5xxx_EEMMemoryInfo
							> >, //until C++0x, the space between the brackets is important
							MSP430F5xxx_Features,
							MSP430F5xxx_ExtFeatures
						> {};




extern const char MSP430F6433[] = "MSP430F6433";
extern const char MSP430F6435[] = "MSP430F6435";
extern const char MSP430F6436[] = "MSP430F6436";
extern const char MSP430F6438[] = "MSP430F6438";
extern const char MSP430F6458[] = "MSP430F6458";
extern const char MSP430F6459[] = "MSP430F6459";


typedef MSP430F64xx<MSP430F6433, 0x811F, MSP430F6433_35_ClockInfo, 2, Size<0x2000> > MSP430F6433_type;
typedef MSP430F64xx<MSP430F6435, 0x8121, MSP430F6433_35_ClockInfo, 4, Size<0x4000> > MSP430F6435_type;
typedef MSP430F64xx<MSP430F6436, 0x8122, MSP430F6436_38_ClockInfo, 2, Size<0x4000> > MSP430F6436_type;
typedef MSP430F64xx<MSP430F6438, 0x8124, MSP430F6436_38_ClockInfo, 4, Size<0x4000> > MSP430F6438_type;

typedef MSP430F645x<MSP430F6458, 0x812E, MSP430F645x_ClockInfo, 3, Size<0x4000>, Offset<0xF8000>, Size<0x8000> > MSP430F6458_type;
typedef MSP430F645x<MSP430F6459, 0x812D, MSP430F645x_ClockInfo, 4, Size<0x4000>, Offset<0xF0000>, Size<0x10000> > MSP430F6459_type;


static const DeviceRegistrator<MSP430F6433_type> regMSP430F6433;
static const DeviceRegistrator<MSP430F6435_type> regMSP430F6435;
static const DeviceRegistrator<MSP430F6436_type> regMSP430F6436;
static const DeviceRegistrator<MSP430F6438_type> regMSP430F6438;
static const DeviceRegistrator<MSP430F6458_type> regMSP430F6458;
static const DeviceRegistrator<MSP430F6459_type> regMSP430F6459;

