/*
 * MSP430F53xx.cpp
 *
 * Definition MSP430F53xx devices.
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

const ClockPair MSP430F53xxTimerTA3_1 = {"Timer1_A3", 57};
const ClockPair MSP430F53xxTimerTA3_2 = {"Timer2_A3", 58};

struct MSP430F53xx_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F53xx_EemTimer(
		const ClockPair& dac, const ClockPair& comp_b,
		const ClockPair& adc, const ClockPair& usci3, const ClockPair& usci2)
		  : EemTimerImpl(
			Eem::Empty, Eem::Empty, dac,		comp_b, 
			adc,		Eem::RTC,	usci3,		usci2,
			Eem::USCI1, Eem::USCI0, Eem::Empty, Eem::TB7_0,
			MSP430F53xxTimerTA3_2, MSP430F53xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty)
	{}
};

struct MSP430F5304_EemTimer : MSP430F53xx_EemTimer
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F5304_EemTimer() : MSP430F53xx_EemTimer(
		Eem::Empty, Eem::Empty, Eem::ADC10_A, Eem::Empty, Eem::Empty)
	{}		
};

struct MSP430F530x_EemTimer : MSP430F53xx_EemTimer
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F530x_EemTimer() : MSP430F53xx_EemTimer(
		Eem::Empty, Eem::COMP_B, Eem::ADC10_A, Eem::USCI3, Eem::USCI2)
	{}		
};

struct MSP430F532_4x_EemTimer : MSP430F53xx_EemTimer
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F532_4x_EemTimer() : MSP430F53xx_EemTimer(
		Eem::Empty, Eem::COMP_B, Eem::ADC12_A, Eem::USCI3, Eem::USCI2)
	{}		
};

struct MSP430F533x_EemTimer : MSP430F53xx_EemTimer
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F533x_EemTimer() : MSP430F53xx_EemTimer(
		Eem::DAC12_0, Eem::COMP_B, Eem::ADC12_A, Eem::USCI3, Eem::USCI2)
	{}		
};

struct MSP430F535x_EemTimer : MSP430F53xx_EemTimer
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F535x_EemTimer() : MSP430F53xx_EemTimer(
		Eem::DAC12_0, Eem::COMP_B, Eem::ADC12_A, Eem::Empty, Eem::Empty)
	{}		
};


typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430F5304_EemTimer, EmptyEemClockNames> MSP430F5304_ClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430F530x_EemTimer, EmptyEemClockNames> MSP430F530x_ClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430F532_4x_EemTimer, EmptyEemClockNames> MSP430F532_4x_ClockInfo;

typedef ClockInfo<GCC_EXTENDED, 0x040F, MSP430F532_4x_EemTimer, EmptyEemClockNames> MSP430F5333_35_ClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x040F, MSP430F533x_EemTimer, EmptyEemClockNames> MSP430F5336_38_ClockInfo;

typedef ClockInfo<GCC_EXTENDED, 0x040F, MSP430F535x_EemTimer, EmptyEemClockNames> MSP430F535x_ClockInfo;



template<class SizeType, class OffsetType>
struct MSP430F530x_MainFlashMemory : MemoryInfo<
											Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
											SizeType, OffsetType, SegmentSize<0x200>, BankSize<0x8000>, Banks<1>, 
											NoMask> {};


template<class SizeType, class BanksType>
struct MSP430F532_4x_MainFlashMemory : MemoryInfo<
										Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
										SizeType, Offset<0x4400>, SegmentSize<0x200>, BankSize<0x8000>, BanksType, 
										NoMask> {};


template<class SizeType, class BanksType>
struct MSP430F533x_MainFlashMemory : MemoryInfo<
										Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
										SizeType, Offset<0x8000>, SegmentSize<0x200>, BankSize<0x10000>, BanksType, 
										NoMask> {};

template<class SizeType, class BanksType>
struct MSP430F535x_MainFlashMemory : MemoryInfo<
										Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
										SizeType, Offset<0x8000>, SegmentSize<0x200>, BankSize<0x20000>, BanksType, 
										NoMask> {};


template<class FlashMemoryType, class RamSizeType>
struct MSP430F53xxMemoryList : MemoryList<boost::tuple<
										FlashMemoryType, 
										MSP430F5xxx_InfoFlashMemoryInfo, 
										MSP430F5xxx_BootFlashMemoryInfo, 
										MSP430F5xxx_BootCodeMemoryInfo,
										UsbTypeRamInfo,
										UsbTypeSystemRamInfo<RamSizeType, Banks<1> >,
										MSP430F5xxx_peripherl16lbitMemoryInfo,
										MSP430F5xxx_CPUMemoryInfo, 
										MSP430F5xxx_EEMMemoryInfo > > {};

template<class FlashMemoryType, typename Ram2OffsetType, typename Ram2SizeType>
struct MSP430F535xMemoryList : MemoryList<boost::tuple<
										FlashMemoryType, 
										MSP430F5xxx_InfoFlashMemoryInfo, 
										MSP430F5xxx_BootFlashMemoryInfo, 
										MSP430F5xxx_BootCodeMemoryInfo,
										UsbTypeRamInfo,
										UsbTypeSystemRamInfo< Size<0x4000>, Banks<1> >,
										MSP430F5xxx_SystemRam2Info<Ram2OffsetType, Ram2SizeType>,
										MSP430F5xxx_peripherl16lbitMemoryInfo,
										MSP430F5xxx_CPUMemoryInfo, 
										MSP430F5xxx_EEMMemoryInfo > > {};


typedef MSP430F53xxMemoryList< 
	MSP430F530x_MainFlashMemory< Size<0x2000>, Offset<0xE000> >, Size<0x1000> 
> MSP430F5304_MemoryInfo;

typedef MSP430F53xxMemoryList<
	MSP430F530x_MainFlashMemory< Size<0x4000>, Offset<0xC000> >, Size<0x1000> 
> MSP430F5308_MemoryInfo;

typedef MSP430F53xxMemoryList<
	MSP430F530x_MainFlashMemory< Size<0x6000>, Offset<0xA000> >, Size<0x1000> 
> MSP430F5309_MemoryInfo;

typedef MSP430F53xxMemoryList<
	MSP430F530x_MainFlashMemory< Size<0x8000>, Offset<0x8000> >, Size<0x1000> 
> MSP430F5310_MemoryInfo;

typedef MSP430F53xxMemoryList<
	MSP430F532_4x_MainFlashMemory< Size<0x10000>, Banks<2> >,	Size<0x1000> 
> MSP430F5340_24_25_MemoryInfo;

typedef MSP430F53xxMemoryList<
	MSP430F532_4x_MainFlashMemory< Size<0x18000>, Banks<3> >,	Size<0x1800> 
> MSP430F5341_26_27_MemoryInfo;

typedef MSP430F53xxMemoryList<
	MSP430F532_4x_MainFlashMemory< Size<0x20000>, Banks<4> >,	Size<0x2000> 
> MSP430F5342_28_29_MemoryInfo;

typedef MSP430F53xxMemoryList<
	MSP430F533x_MainFlashMemory< Size<0x20000>, Banks<2> >, Size<0x2000>
> MSP430F5333_MemoryInfo;

typedef MSP430F53xxMemoryList<
	MSP430F533x_MainFlashMemory< Size<0x20000>, Banks<2> >, Size<0x4000>
> MSP430F5336_MemoryInfo;

typedef MSP430F53xxMemoryList<
	MSP430F533x_MainFlashMemory< Size<0x40000>, Banks<4> >, Size<0x4000>
> MSP430F5335_38_MemoryInfo;

typedef MSP430F535xMemoryList<
	MSP430F535x_MainFlashMemory< Size<0x60000>, Banks<3> >, Offset<0xF8000>, Size<0x8000>
> MSP430F5358_MemoryInfo;

typedef MSP430F535xMemoryList<
	MSP430F535x_MainFlashMemory< Size<0x80000>, Banks<4> >, Offset<0xF0000>, Size<0x10000>
> MSP430F5359_MemoryInfo;




template<
	const char* description,
	const unsigned int versionId,
	typename EEMTrigger,
	typename ClockInfo,
	typename FuncletMapping,
	typename MemoryList,
	typename PowerSettings = NoPowerSettings
>
struct MSP430F53xx : Device<
						description, 
						ObjectId<0>,
						DefaultBits20Type,
						regular, 
						MSP430F5xxx_Match<versionId, 0x00>, 
						EEMTrigger, 
						MSP430F5xxx_DefaultVoltageTestVpp,
						ClockInfo,
						FunctionMappingXv2,
						FuncletMapping,
						MemoryList,
						MSP430F5xxx_Features,
						MSP430F5xxx_ExtFeatures,
						PowerSettings> {};


template<
	const char* description,
	const unsigned int versionId,
	typename MemoryList
>
struct MSP430F530x : MSP430F53xx<description, versionId,
								SmallEemMode, MSP430F530x_ClockInfo, FuncletMappingXv2, MemoryList> {};


template<
	const char* description,
	const unsigned int versionId,
	typename MemoryList
>
struct MSP430F532_4x : MSP430F53xx<description, versionId,
								LargeEemMode, MSP430F532_4x_ClockInfo, FuncletMappingXv2, MemoryList> {};


template<
	const char* description,
	const unsigned int versionId,
	typename ClockInfo,
	typename MemoryList
>
struct MSP430F533x : MSP430F53xx<description, versionId,
								LargeEemMode, ClockInfo, FuncletMappingXv2, MemoryList> {};


template<
	const char* description,
	const unsigned int versionId,
	typename MemoryList
>
struct MSP430F535x : MSP430F53xx<description, versionId,
								LargeEemMode, MSP430F535x_ClockInfo, FuncletMappingXv2WordMode, MemoryList,
								PowerSettings<
									0x0, // Test reg mask
									0x0, // Test reg value to enable LPMx.5 
									0x0, // Test reg value to disable LPMx.5 
									0x4020,	  // 3V Test reg mask 
									0x4020,	  // 3V Test reg value to enable LPMx.5 
									0x4020	  // 3V Test reg value to disable LPMx.5 
								> 
					 > {};




extern const char MSP430F5304[] = "MSP430F5304";
extern const char MSP430F5308[] = "MSP430F5308";
extern const char MSP430F5309[] = "MSP430F5309";
extern const char MSP430F5310[] = "MSP430F5310";

extern const char MSP430F5324[] = "MSP430F5324";
extern const char MSP430F5325[] = "MSP430F5325";
extern const char MSP430F5326[] = "MSP430F5326";
extern const char MSP430F5327[] = "MSP430F5327";
extern const char MSP430F5328[] = "MSP430F5328";
extern const char MSP430F5329[] = "MSP430F5329";

extern const char MSP430F5333[] = "MSP430F5333";
extern const char MSP430F5335[] = "MSP430F5335";
extern const char MSP430F5336[] = "MSP430F5336";
extern const char MSP430F5338[] = "MSP430F5338";

extern const char MSP430F5340[] = "MSP430F5340";
extern const char MSP430F5341[] = "MSP430F5341";
extern const char MSP430F5342[] = "MSP430F5342";

extern const char MSP430F5358[] = "MSP430F5358";
extern const char MSP430F5359[] = "MSP430F5359";


typedef MSP430F53xx<
		MSP430F5304, 0x8112, SmallEemMode, MSP430F5304_ClockInfo, FuncletMappingXv2, MSP430F5304_MemoryInfo
> MSP430F5304_type;

typedef MSP430F530x<MSP430F5308, 0x8113, MSP430F5308_MemoryInfo> MSP430F5308_type;
typedef MSP430F530x<MSP430F5309, 0x8114, MSP430F5309_MemoryInfo> MSP430F5309_type;
typedef MSP430F530x<MSP430F5310, 0x8115, MSP430F5310_MemoryInfo> MSP430F5310_type;

typedef MSP430F532_4x<MSP430F5324, 0x8116, MSP430F5340_24_25_MemoryInfo> MSP430F5324_type;
typedef MSP430F532_4x<MSP430F5325, 0x8117, MSP430F5340_24_25_MemoryInfo> MSP430F5325_type;
typedef MSP430F532_4x<MSP430F5326, 0x8118, MSP430F5341_26_27_MemoryInfo> MSP430F5326_type;
typedef MSP430F532_4x<MSP430F5327, 0x8119, MSP430F5341_26_27_MemoryInfo> MSP430F5327_type;
typedef MSP430F532_4x<MSP430F5328, 0x811a, MSP430F5342_28_29_MemoryInfo> MSP430F5328_type;
typedef MSP430F532_4x<MSP430F5329, 0x811b, MSP430F5342_28_29_MemoryInfo> MSP430F5329_type;

typedef MSP430F533x<MSP430F5333, 0x8125, MSP430F5333_35_ClockInfo, MSP430F5333_MemoryInfo> MSP430F5333_type;
typedef MSP430F533x<MSP430F5335, 0x8127, MSP430F5333_35_ClockInfo, MSP430F5335_38_MemoryInfo> MSP430F5335_type;
typedef MSP430F533x<MSP430F5336, 0x8128, MSP430F5336_38_ClockInfo, MSP430F5336_MemoryInfo> MSP430F5336_type;
typedef MSP430F533x<MSP430F5338, 0x812A, MSP430F5336_38_ClockInfo, MSP430F5335_38_MemoryInfo> MSP430F5338_type;

typedef MSP430F532_4x<MSP430F5340, 0x811c, MSP430F5340_24_25_MemoryInfo> MSP430F5340_type;
typedef MSP430F532_4x<MSP430F5341, 0x811d, MSP430F5341_26_27_MemoryInfo> MSP430F5341_type;
typedef MSP430F532_4x<MSP430F5342, 0x811e, MSP430F5342_28_29_MemoryInfo> MSP430F5342_type;

typedef MSP430F535x<MSP430F5358, 0x8133, MSP430F5358_MemoryInfo> MSP430F5358_type;
typedef MSP430F535x<MSP430F5359, 0x8132, MSP430F5359_MemoryInfo> MSP430F5359_type;



static const DeviceRegistrator<MSP430F5304_type> regMSP430F5304;
static const DeviceRegistrator<MSP430F5308_type> regMSP430F5308;
static const DeviceRegistrator<MSP430F5309_type> regMSP430F5309;
static const DeviceRegistrator<MSP430F5310_type> regMSP430F5310;

static const DeviceRegistrator<MSP430F5324_type> regMSP430F5324;
static const DeviceRegistrator<MSP430F5325_type> regMSP430F5325;
static const DeviceRegistrator<MSP430F5326_type> regMSP430F5326;
static const DeviceRegistrator<MSP430F5327_type> regMSP430F5327;
static const DeviceRegistrator<MSP430F5328_type> regMSP430F5328;
static const DeviceRegistrator<MSP430F5329_type> regMSP430F5329;

static const DeviceRegistrator<MSP430F5333_type> regMSP430F5333;
static const DeviceRegistrator<MSP430F5335_type> regMSP430F5335;
static const DeviceRegistrator<MSP430F5336_type> regMSP430F5336;
static const DeviceRegistrator<MSP430F5338_type> regMSP430F5338;

static const DeviceRegistrator<MSP430F5340_type> regMSP430F5340;
static const DeviceRegistrator<MSP430F5341_type> regMSP430F5341;
static const DeviceRegistrator<MSP430F5342_type> regMSP430F5342;

static const DeviceRegistrator<MSP430F5358_type> regMSP430F5358;
static const DeviceRegistrator<MSP430F5359_type> regMSP430F5359;
