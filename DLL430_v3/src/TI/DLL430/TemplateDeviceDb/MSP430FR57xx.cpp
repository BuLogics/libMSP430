/*
 * MSP430FR57xx.cpp
 *
 * Definition MSP430FR57xx.
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
#include "MpuFr5739.h"
#include "FramMemoryAccessBaseFR57.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;



struct MSP430FR57xx_EemTimerLarge : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FR57xx_EemTimerLarge() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_D, 
		Eem::ADC10_B, Eem::RTC, Eem::Empty, Eem::eUSCIB0,
		Eem::eUSCIA1, Eem::eUSCIA0, Eem::TB3_0, Eem::TB3_1,
		Eem::TB3_2, Eem::TA3_0, Eem::TA3_1, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct MSP430FR57xx_EemTimerLarge_NoADC : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FR57xx_EemTimerLarge_NoADC() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_D, 
		Eem::Empty, Eem::RTC, Eem::Empty, Eem::eUSCIB0,
		Eem::eUSCIA1, Eem::eUSCIA0, Eem::TB3_0, Eem::TB3_1,
		Eem::TB3_2, Eem::TA3_0, Eem::TA3_1, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct MSP430FR57xx_EemTimerSmall : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FR57xx_EemTimerSmall() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_D, 
		Eem::ADC10_B, Eem::RTC, Eem::Empty, Eem::eUSCIB0,
		Eem::Empty, Eem::eUSCIA0, Eem::TB3_0, Eem::Empty,
		Eem::Empty, Eem::TA3_0, Eem::TA3_1, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct MSP430FR57xx_EemTimerSmall_NoADC : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FR57xx_EemTimerSmall_NoADC() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_D, 
		Eem::Empty, Eem::RTC, Eem::Empty, Eem::eUSCIB0,
		Eem::Empty, Eem::eUSCIA0, Eem::TB3_0, Eem::Empty,
		Eem::Empty, Eem::TA3_0, Eem::TA3_1, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};


typedef ClockInfo<GCC_EXTENDED, 0x040F, MSP430FR57xx_EemTimerLarge, EmptyEemClockNames> FR57xx_LargeClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x040F, MSP430FR57xx_EemTimerLarge_NoADC, EmptyEemClockNames> FR57xx_LargeClockInfo_NoADC;
typedef ClockInfo<GCC_EXTENDED, 0x040F, MSP430FR57xx_EemTimerSmall, EmptyEemClockNames> FR57xx_SmallClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x040F, MSP430FR57xx_EemTimerSmall_NoADC, EmptyEemClockNames> FR57xx_SmallClockInfo_NoADC;


typedef Trigger<EMEX_SMALL_5XX, 0x03, 0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x01> FR57xxTrigger;


typedef EemInfo<0x00, 0x01, 0x01, FR57xxTrigger, SmallSequencer> FR57xxEemMode;


//protectable due to INFO A
typedef MemoryInfo<
	Name::information, RamType, Mapped, NotProtectable, Bits16Type, 
	Size<0x100> , Offset<0x1800>, SegmentSize<0x01>, BankSize<0x0>, Banks<1>,
	NoMask, MemoryCreator<FramMemoryAccessBaseFR57<MpuFr5739> >
> MSP430FR57xx_InfoFramMemoryInfo;


///bsl memory
typedef MemoryInfo<
	Name::boot, RomType, Mapped, NotProtectable, Bits16Type, 
	Size<0x800> , Offset<0x1000>, SegmentSize<0x01>, BankSize<0>, Banks<1>, 
	NoMask, MemoryCreator<ReadonlyMemoryAccess>
> MSP430FR57xx_BootFramMemoryInfo;


template<class SizeType, class OffsetType>
struct MSP430FR57xx_MainFramMemory : MemoryInfo<
										Name::main, RamType, Mapped, NotProtectable, Bits16Type, 
										SizeType , OffsetType, SegmentSize<0x01>, 
										BankSize<0x0>, Banks<1>, NoMask, MemoryCreator<FramMemoryAccessBaseFR57<MpuFr5739> >
									> {};

typedef Features<MOD_OSC, false, true, true, false, true, false> MSP430FR57xx_Features;

template<
	const char* description,
	const unsigned int versionId,
	class ClockInfoType,
	const unsigned int FramOffset,
	const unsigned int FramSize,
	const unsigned int SysRamSize
>
struct MSP430FR57xx_type : Device<
							description, 
							ObjectId<0>,
							DefaultBits20Type, 
							regular, 
							MSP430F5xxx_Match<versionId>, 
							FR57xxEemMode, 
							MSP430F5xxx_DefaultVoltageTestVpp,
							ClockInfoType,
							FunctionMappingXv2FRAM,
							FuncletMappingXv2FRAM,
							MemoryList<boost::tuple<
								MSP430FR57xx_MainFramMemory< Size<FramSize>, Offset<FramOffset> >,
								MSP430FR57xx_InfoFramMemoryInfo, 
								MSP430FR57xx_BootFramMemoryInfo,
								MSP430F5xxx_BootCodeMemoryInfo,
								MSP430F5xxx_SystemRamInfo< Size<SysRamSize> >,
								MSP430F5xxx_peripherl16lbitMemoryInfo, 
								MSP430F5xxx_CPUMemoryInfo, 
								MSP430F5xxx_EEMMemoryInfo
							> >, //until C++0x, the space between the brackets is important
							MSP430FR57xx_Features,
							NoExtendedFeatures,
							PowerSettings<0x00010018, // Test reg mask
									      0x00010010, // Test reg value to enable LPMx.5 
										  0x00000018, // Test reg value to disable LPMx.5 
										  0x4020,	  // 3V Test reg mask 
										  0x4020,	  // 3V Test reg value to enable LPMx.5 
										  0x4020>	  // 3V Test reg value to disable LPMx.5 
						> {};

extern const char MSP430FR5729[] = "MSP430FR5729";
extern const char MSP430FR5728[] = "MSP430FR5728";
extern const char MSP430FR5727[] = "MSP430FR5727";
extern const char MSP430FR5726[] = "MSP430FR5726";
extern const char MSP430FR5725[] = "MSP430FR5725";
extern const char MSP430FR5724[] = "MSP430FR5724";
extern const char MSP430FR5723[] = "MSP430FR5723";
extern const char MSP430FR5722[] = "MSP430FR5722";
extern const char MSP430FR5721[] = "MSP430FR5721";
extern const char MSP430FR5720[] = "MSP430FR5720";

extern const char MSP430FR5739[] = "MSP430FR5739";
extern const char MSP430FR5738[] = "MSP430FR5738";
extern const char MSP430FR5737[] = "MSP430FR5737";
extern const char MSP430FR5736[] = "MSP430FR5736";
extern const char MSP430FR5735[] = "MSP430FR5735";
extern const char MSP430FR5734[] = "MSP430FR5734";
extern const char MSP430FR5733[] = "MSP430FR5733";
extern const char MSP430FR5732[] = "MSP430FR5732";
extern const char MSP430FR5731[] = "MSP430FR5731";
extern const char MSP430FR5730[] = "MSP430FR5730";


static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5729, 0x807B, FR57xx_LargeClockInfo, 0xC200, 0x3E00, 0x400> 
> regMSP430FR5729;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5728, 0x807A, FR57xx_SmallClockInfo, 0xC200, 0x3E00, 0x400> 
> regMSP430FR5728;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5727, 0x8079, FR57xx_LargeClockInfo_NoADC, 0xC200, 0x3E00, 0x400> 
> regMSP430FR5727;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5726, 0x8174, FR57xx_SmallClockInfo_NoADC, 0xC200, 0x3E00, 0x400> 
> regMSP430FR5726;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5725, 0x8078, FR57xx_LargeClockInfo, 0xE000, 0x2000, 0x400> 
> regMSP430FR5725;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5724, 0x8173, FR57xx_SmallClockInfo, 0xE000, 0x2000, 0x400> 
> regMSP430FR5724;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5723, 0x8172, FR57xx_LargeClockInfo_NoADC, 0xE000, 0x2000, 0x400> 
> regMSP430FR5723;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5722, 0x8171, FR57xx_SmallClockInfo_NoADC, 0xE000, 0x2000, 0x400> 
> regMSP430FR5722;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5721, 0x8077, FR57xx_LargeClockInfo, 0xF000, 0x1000, 0x400> 
> regMSP430FR5721;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5720, 0x8170, FR57xx_SmallClockInfo, 0xF000, 0x1000, 0x400> 
> regMSP430FR5720;



static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5739, 0x8103, FR57xx_LargeClockInfo, 0xC200, 0x3E00, 0x400> 
> regMSP430FR5739;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5738, 0x8102, FR57xx_SmallClockInfo, 0xC200, 0x3E00, 0x400> 
> regMSP430FR5738;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5737, 0x8101, FR57xx_LargeClockInfo_NoADC, 0xC200, 0x3E00, 0x400> 
> regMSP430FR5737;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5736, 0x8177, FR57xx_SmallClockInfo_NoADC, 0xC200, 0x3E00, 0x400> 
> regMSP430FR5736;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5735, 0x8176, FR57xx_LargeClockInfo, 0xE000, 0x2000, 0x400> 
> regMSP430FR5735;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5734, 0x8100, FR57xx_SmallClockInfo, 0xE000, 0x2000, 0x400> 
> regMSP430FR5734;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5733, 0x807F, FR57xx_LargeClockInfo_NoADC, 0xE000, 0x2000, 0x400> 
> regMSP430FR5733;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5732, 0x8175, FR57xx_SmallClockInfo_NoADC, 0xE000, 0x2000, 0x400> 
> regMSP430FR5732;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5731, 0x807E, FR57xx_LargeClockInfo, 0xF000, 0x1000, 0x0400> 
> regMSP430FR5731;

static const DeviceRegistrator<
	MSP430FR57xx_type<MSP430FR5730, 0x807C, FR57xx_SmallClockInfo, 0xF000, 0x1000, 0x0400> 
> regMSP430FR5730;
