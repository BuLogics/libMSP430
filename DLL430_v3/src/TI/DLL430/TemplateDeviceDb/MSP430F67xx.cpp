/*
 * MSP430F67xx.cpp
 *
 * Definition MSP430F67xx devices.
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

const ClockPair F67xxTimerTA2_3 = {"Timer3_A2", 11};

struct MSP430F67xx_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F67xx_EemTimer()
		  : EemTimerImpl(
			Eem::Empty, Eem::Empty, Eem::Empty,	Eem::SD24B, 
			Eem::ADC10_A,Eem::RTC,	Eem::eUSCIB0,Eem::eUSCIA1,
			Eem::eUSCIA0,Eem::Empty, Eem::Empty, Eem::TA3_0,
			Eem::TA2_1, Eem::TA2_2, F67xxTimerTA2_3, Eem::WDT_A,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty)
	{}
};


typedef ClockInfo<GCC_EXTENDED, 0x2407, MSP430F67xx_EemTimer, EmptyEemClockNames> F67xx_ClockInfo;


typedef MemoryInfo<
		Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
		Size<0x4000> , Offset<0xC000>, SegmentSize<0x200>, 
		BankSize<0x8000>, Banks<1>, NoMask
> MainFlash_16k;

typedef MemoryInfo<
		Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
		Size<0x8000> , Offset<0x8000>, SegmentSize<0x200>, 
		BankSize<0x8000>, Banks<1>, NoMask
> MainFlash_32k;

typedef MemoryInfo<
		Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
		Size<0xC000> , Offset<0x4000>, SegmentSize<0x200>, 
		BankSize<0x8000>, Banks<2>, NoMask
> MainFlash_48k;

typedef MemoryInfo<
		Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
		Size<0x10000> , Offset<0x4000>, SegmentSize<0x200>, 
		BankSize<0x8000>, Banks<2>, NoMask
> MainFlash_64k;

typedef MemoryInfo<
		Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
		Size<0x18000> , Offset<0x4000>, SegmentSize<0x200>, 
		BankSize<0x8000>, Banks<3>, NoMask
> MainFlash_96k;

typedef MemoryInfo<
		Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
		Size<0x20000> , Offset<0x4000>, SegmentSize<0x200>, 
		BankSize<0x8000>, Banks<4>, NoMask
> MainFlash_128k;


typedef MSP430F5xxx_SystemRamInfo< Size<0x400> > SystemRam_1k;
typedef MSP430F5xxx_SystemRamInfo< Size<0x800> > SystemRam_2k;
typedef MSP430F5xxx_SystemRamInfo< Size<0x1000> > SystemRam_4k;
typedef MSP430F5xxx_SystemRamInfo< Size<0x2000> > SystemRam_8k;


template<
	const char* description,
	const unsigned int versionId,
	typename ClockInfo,
	class MainFlashInfo,
	class RamInfo
>
struct MSP430F67xx : Device<
							description, 
							ObjectId<0>,
							DefaultBits20Type,
							enhanced, 
							MSP430F5xxx_Match<versionId>, 
							SmallEemMode,
							MSP430F5xxx_DefaultVoltageTestVpp,
							ClockInfo,
							FunctionMappingXv2,
							FuncletMappingXv2,
							MemoryList<boost::tuple<
								MainFlashInfo,
								MSP430F5xxx_InfoFlashMemoryInfo, 
								MSP430F5xxx_BootFlashMemoryInfo, 
								MSP430F5xxx_BootCodeMemoryInfo,
								RamInfo,
								MSP430F5xxx_peripherl16lbitMemoryInfo, 
								MSP430F5xxx_CPUMemoryInfo, 
								MSP430F5xxx_EEMMemoryInfo
							> >, //until C++0x, the space between the brackets is important
							MSP430F5xxx_Features,
							MSP430F5xxx_ExtFeatures,
							PowerSettings<
								0x0, // Test reg mask
								0x0, // Test reg value to enable LPMx.5 
								0x0, // Test reg value to disable LPMx.5 
								0x4020,	  // 3V Test reg mask 
								0x4020,	  // 3V Test reg value to enable LPMx.5 
								0x4020>	  // 3V Test reg value to disable LPMx.5 
						> {};


extern const char MSP430F6720[] = "MSP430F6720";
extern const char MSP430F6721[] = "MSP430F6721";
extern const char MSP430F6722[] = "MSP430F6722";
extern const char MSP430F6723[] = "MSP430F6723";
extern const char MSP430F6724[] = "MSP430F6724";
extern const char MSP430F6725[] = "MSP430F6725";
extern const char MSP430F6726[] = "MSP430F6726";

extern const char MSP430F6730[] = "MSP430F6730";
extern const char MSP430F6731[] = "MSP430F6731";
extern const char MSP430F6732[] = "MSP430F6732";
extern const char MSP430F6733[] = "MSP430F6733";
extern const char MSP430F6734[] = "MSP430F6734";
extern const char MSP430F6735[] = "MSP430F6735";
extern const char MSP430F6736[] = "MSP430F6736";


static const DeviceRegistrator<
	MSP430F67xx<MSP430F6734, 0x816A, F67xx_ClockInfo, MainFlash_96k, SystemRam_4k>
> regMSP430F6734;

static const DeviceRegistrator<
	MSP430F67xx<MSP430F6735, 0x816B, F67xx_ClockInfo, MainFlash_128k, SystemRam_4k>
> regMSP430F6735;

static const DeviceRegistrator<
	MSP430F67xx<MSP430F6736, 0x816C, F67xx_ClockInfo, MainFlash_128k, SystemRam_8k>
> regMSP430F6736;

static const DeviceRegistrator<
	MSP430F67xx<MSP430F6724, 0x816D, F67xx_ClockInfo, MainFlash_96k, SystemRam_4k>
> regMSP430F6724;

static const DeviceRegistrator<
	MSP430F67xx<MSP430F6725, 0x816E, F67xx_ClockInfo, MainFlash_128k, SystemRam_4k>
> regMSP430F6725;

static const DeviceRegistrator<
	MSP430F67xx<MSP430F6726, 0x816F, F67xx_ClockInfo, MainFlash_128k, SystemRam_8k>
> regMSP430F6726;


static const DeviceRegistrator<
	MSP430F67xx<MSP430F6720, 0x8058, F67xx_ClockInfo, MainFlash_16k, SystemRam_1k>
> regMSP430F6720;

static const DeviceRegistrator<
	MSP430F67xx<MSP430F6721, 0x8059, F67xx_ClockInfo, MainFlash_32k, SystemRam_2k>
> regMSP430F6721;

static const DeviceRegistrator<
	MSP430F67xx<MSP430F6722, 0x8060, F67xx_ClockInfo, MainFlash_48k, SystemRam_4k>
> regMSP430F6722;

static const DeviceRegistrator<
	MSP430F67xx<MSP430F6723, 0x8061, F67xx_ClockInfo, MainFlash_64k, SystemRam_4k>
> regMSP430F6723;

static const DeviceRegistrator<
	MSP430F67xx<MSP430F6730, 0x8062, F67xx_ClockInfo, MainFlash_16k, SystemRam_1k>
> regMSP430F6730;

static const DeviceRegistrator<
	MSP430F67xx<MSP430F6731, 0x8063, F67xx_ClockInfo, MainFlash_32k, SystemRam_2k>
> regMSP430F6731;

static const DeviceRegistrator<
	MSP430F67xx<MSP430F6732, 0x8064, F67xx_ClockInfo, MainFlash_48k, SystemRam_4k>
> regMSP430F6732;

static const DeviceRegistrator<
	MSP430F67xx<MSP430F6733, 0x8065, F67xx_ClockInfo, MainFlash_64k, SystemRam_4k>
> regMSP430F6733;



const ClockPair F6779eUSCIB1 = {"eUSCIB1", 59};

struct MSP430F6779_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F6779_EemTimer()
		  : EemTimerImpl(
			Eem::AES, Eem::eUSCIA3,Eem::eUSCIA2,Eem::SD24B, 
			Eem::ADC10_A,Eem::RTC,	Eem::eUSCIB0,Eem::eUSCIA1,
			Eem::eUSCIA0,F6779eUSCIB1,Eem::COMP_B, Eem::TA3_0,
			Eem::TA2_1, Eem::TA2_2, F67xxTimerTA2_3, Eem::WDT_A,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x2407, MSP430F6779_EemTimer, EmptyEemClockNames> F6779_ClockInfo;


struct MSP430F67791_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F67791_EemTimer()
		  : EemTimerImpl(
			Eem::Empty, Eem::eUSCIA3,Eem::eUSCIA2,Eem::SD24B, 
			Eem::ADC10_A,Eem::RTC,	Eem::eUSCIB0,Eem::eUSCIA1,
			Eem::eUSCIA0,F6779eUSCIB1,Eem::COMP_B, Eem::TA3_0,
			Eem::TA2_1, Eem::TA2_2, F67xxTimerTA2_3, Eem::WDT_A,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x2407, MSP430F67791_EemTimer, EmptyEemClockNames> F67791_ClockInfo;

typedef MemoryInfo<
		Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
		Size<0x80000> , Offset<0xC000>, SegmentSize<0x200>, 
		BankSize<0x20000>, Banks<4>, NoMask
> MainFlash6779_512k;

typedef MemoryInfo<
		Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
		Size<0x40000> , Offset<0xC000>, SegmentSize<0x200>, 
		BankSize<0x20000>, Banks<2>, NoMask
> MainFlash6779_256k;

typedef MemoryInfo<
		Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
		Size<0x20000> , Offset<0xC000>, SegmentSize<0x200>, 
		BankSize<0x20000>, Banks<1>, NoMask
> MainFlash6779_128k;
 
typedef MSP430F5xxx_SystemRamInfo< Size<0x8000> > SystemRam_32k;
typedef MSP430F5xxx_SystemRamInfo< Size<0x4000> > SystemRam_16k;


template<
	const char* description,
	const unsigned int versionId,
	class ClockInfo,
	class MainFlashInfo,
	class RamInfo
>
struct MSP430F6779x : Device<
							description, 
							ObjectId<0>,
							DefaultBits20Type,
							enhanced, 
							MSP430F5xxx_Match<versionId>, 
							LargeEemMode,
							MSP430F5xxx_DefaultVoltageTestVpp,
							ClockInfo,
							FunctionMappingXv2,
							FuncletMappingXv2,
							MemoryList<boost::tuple<
								MainFlashInfo,
								MSP430F5xxx_InfoFlashMemoryInfo, 
								MSP430F5xxx_BootFlashMemoryInfo, 
								MSP430F5xxx_BootCodeMemoryInfo,
								RamInfo,
								MSP430F5xxx_peripherl16lbitMemoryInfo, 
								MSP430F5xxx_CPUMemoryInfo, 
								MSP430F5xxx_EEMMemoryInfo
							> >, //until C++0x, the space between the brackets is important
							MSP430F5xxx_Features,
							MSP430F5xxx_ExtFeatures,
							PowerSettings<
								0x0, // Test reg mask
								0x0, // Test reg value to enable LPMx.5 
								0x0, // Test reg value to disable LPMx.5 
								0x4020,	  // 3V Test reg mask 
								0x4020,	  // 3V Test reg value to enable LPMx.5 
								0x4020>	  // 3V Test reg value to disable LPMx.5 
						> {};


extern const char MSP430F6779[] = "MSP430F6779";
extern const char MSP430F6778[] = "MSP430F6778";
extern const char MSP430F6777[] = "MSP430F6777";
extern const char MSP430F6776[] = "MSP430F6776";
extern const char MSP430F6775[] = "MSP430F6775";

extern const char MSP430F6769[] = "MSP430F6769";
extern const char MSP430F6768[] = "MSP430F6768";
extern const char MSP430F6767[] = "MSP430F6767";
extern const char MSP430F6766[] = "MSP430F6766";
extern const char MSP430F6765[] = "MSP430F6765";

extern const char MSP430F6749[] = "MSP430F6749";
extern const char MSP430F6748[] = "MSP430F6748";
extern const char MSP430F6747[] = "MSP430F6747";
extern const char MSP430F6746[] = "MSP430F6746";
extern const char MSP430F6745[] = "MSP430F6745";


extern const char MSP430F67791[] = "MSP430F67791";
extern const char MSP430F67781[] = "MSP430F67781";
extern const char MSP430F67771[] = "MSP430F67771";
extern const char MSP430F67761[] = "MSP430F67761";
extern const char MSP430F67751[] = "MSP430F67751";

extern const char MSP430F67691[] = "MSP430F67691";
extern const char MSP430F67681[] = "MSP430F67681";
extern const char MSP430F67671[] = "MSP430F67671";
extern const char MSP430F67661[] = "MSP430F67661";
extern const char MSP430F67651[] = "MSP430F67651";

extern const char MSP430F67491[] = "MSP430F67491";
extern const char MSP430F67481[] = "MSP430F67481";
extern const char MSP430F67471[] = "MSP430F67471";
extern const char MSP430F67461[] = "MSP430F67461";
extern const char MSP430F67451[] = "MSP430F67451";



static const DeviceRegistrator<
	MSP430F6779x<MSP430F6779, 0x8196, F6779_ClockInfo, MainFlash6779_512k, SystemRam_32k>
> regMSP430F6779;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6769, 0x8191, F6779_ClockInfo, MainFlash6779_512k, SystemRam_32k>
> regMSP430F6769;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6749, 0x818C, F6779_ClockInfo, MainFlash6779_512k, SystemRam_32k>
> regMSP430F6749;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6778, 0x8195, F6779_ClockInfo, MainFlash6779_512k, SystemRam_16k>
> regMSP430F6778;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6768, 0x8190, F6779_ClockInfo, MainFlash6779_512k, SystemRam_16k>
> regMSP430F6768;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6748, 0x818B, F6779_ClockInfo, MainFlash6779_512k, SystemRam_16k>
> regMSP430F6748;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6777, 0x8194, F6779_ClockInfo, MainFlash6779_256k, SystemRam_32k>
> regMSP430F6777;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6767, 0x818F, F6779_ClockInfo, MainFlash6779_256k, SystemRam_32k>
> regMSP430F6767;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6747, 0x818A, F6779_ClockInfo, MainFlash6779_256k, SystemRam_32k>
> regMSP430F6747;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6776, 0x8193, F6779_ClockInfo, MainFlash6779_256k, SystemRam_16k>
> regMSP430F6776;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6766, 0x818E, F6779_ClockInfo, MainFlash6779_256k, SystemRam_16k>
> regMSP430F6766;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6746, 0x8189, F6779_ClockInfo, MainFlash6779_256k, SystemRam_16k>
> regMSP430F6746;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6775, 0x8192, F6779_ClockInfo, MainFlash6779_128k, SystemRam_16k>
> regMSP430F6775;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6765, 0x818D, F6779_ClockInfo, MainFlash6779_128k, SystemRam_16k>
> regMSP430F6765;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F6745, 0x8188, F6779_ClockInfo, MainFlash6779_128k, SystemRam_16k>
> regMSP430F6745;



static const DeviceRegistrator<
	MSP430F6779x<MSP430F67791, 0x81A5, F67791_ClockInfo, MainFlash6779_512k, SystemRam_32k>
> regMSP430F67791;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67691, 0x81A0, F67791_ClockInfo, MainFlash6779_512k, SystemRam_32k>
> regMSP430F67691;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67491, 0x819B, F67791_ClockInfo, MainFlash6779_512k, SystemRam_32k>
> regMSP430F67491;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67781, 0x81A4, F67791_ClockInfo, MainFlash6779_512k, SystemRam_16k>
> regMSP430F67781;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67681, 0x819F, F67791_ClockInfo, MainFlash6779_512k, SystemRam_16k>
> regMSP430F67681;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67481, 0x819A, F67791_ClockInfo, MainFlash6779_512k, SystemRam_16k>
> regMSP430F67481;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67771, 0x81A3, F67791_ClockInfo, MainFlash6779_256k, SystemRam_32k>
> regMSP430F67771;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67671, 0x819E, F67791_ClockInfo, MainFlash6779_256k, SystemRam_32k>
> regMSP430F67671;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67471, 0x8199, F67791_ClockInfo, MainFlash6779_256k, SystemRam_32k>
> regMSP430F67471;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67761, 0x81A2, F67791_ClockInfo, MainFlash6779_256k, SystemRam_16k>
> regMSP430F67761;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67661, 0x819D, F67791_ClockInfo, MainFlash6779_256k, SystemRam_16k>
> regMSP430F67661;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67461, 0x8198, F67791_ClockInfo, MainFlash6779_256k, SystemRam_16k>
> regMSP430F67461;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67751, 0x81A1, F67791_ClockInfo, MainFlash6779_128k, SystemRam_16k>
> regMSP430F67751;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67651, 0x819C, F67791_ClockInfo, MainFlash6779_128k, SystemRam_16k>
> regMSP430F67651;

static const DeviceRegistrator<
	MSP430F6779x<MSP430F67451, 0x8197, F67791_ClockInfo, MainFlash6779_128k, SystemRam_16k>
> regMSP430F67451;
