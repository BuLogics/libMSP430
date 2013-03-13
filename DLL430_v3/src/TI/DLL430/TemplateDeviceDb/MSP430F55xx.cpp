/*
 * MSP430F55xx.cpp
 *
 * Definition MSP430F55xx USB and non USB devices.
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

const ClockPair MSP430F55xxTimerTA3_1 = {"Timer1_A3", 57};
const ClockPair MSP430F55xxTimerTA3_2 = {"Timer2_A3", 58};

struct MSP430F552x_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F552x_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_B, 
		Eem::ADC12_A, Eem::RTC, Eem::USCI3, Eem::USCI2,
		Eem::USCI1, Eem::USCI0, Eem::USB, Eem::TB7_0,
		MSP430F55xxTimerTA3_2, MSP430F55xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct MSP430F551x_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F551x_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_B, 
		Eem::Empty, Eem::RTC, Eem::USCI3, Eem::USCI2,
		Eem::USCI1, Eem::USCI0, Eem::USB, Eem::TB7_0,
		MSP430F55xxTimerTA3_2, MSP430F55xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430F552x_EemTimer, EmptyEemClockNames> MSP430_552x_LargeClockInfo;	
typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430F551x_EemTimer, EmptyEemClockNames> MSP430_551x_LargeClockInfo;



template<class SizeType, class OffsetType, class BanksType>
struct MSP430F55xx_MainFlashMemory : MemoryInfo<
										Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
										SizeType , OffsetType, SegmentSize<0x200>, 
										BankSize<0x10000>, BanksType, NoMask> {};


template<
	const char* description,
	const unsigned int versionId,
	class ClockInfoType,
	class FlashSizeType, class FlashOffsetType, class FlashBanksType, 
	class RamSizeType, class RamBanksType
>
struct MSP430F55xx_type : Device<
							description, 
							ObjectId<0>,
							DefaultBits20Type, 
							regular, 
							MSP430F5xxx_Match<versionId>, 
							LargeEemMode, 
							MSP430F5xxx_DefaultVoltageTestVpp,
							ClockInfoType,
							FunctionMappingXv2,
							FuncletMappingXv2,
							MemoryList<boost::tuple<
								MSP430F55xx_MainFlashMemory<FlashSizeType, FlashOffsetType, FlashBanksType>,
								MSP430F5xxx_InfoFlashMemoryInfo, 
								MSP430F5xxx_BootFlashMemoryInfo, 
								MSP430F5xxx_BootCodeMemoryInfo,
								UsbTypeRamInfo,
								UsbTypeSystemRamInfo<RamSizeType, RamBanksType>,
								MSP430F5xxx_peripherl16lbitMemoryInfo, 
								MSP430F5xxx_CPUMemoryInfo, 
								MSP430F5xxx_EEMMemoryInfo
							> >, //until C++0x, the space between the brackets is important
							MSP430F5xxx_Features,
							MSP430F5xxx_ExtFeatures
						> {};

typedef MSP430_552x_LargeClockInfo ClockInfo_552x;	
typedef MSP430_551x_LargeClockInfo ClockInfo_551x;

//128 KB types
template<
	const char* description,
	const unsigned int versionId,
	class clockInfo
>
struct MSP430F552x_128KB_type : MSP430F55xx_type<
									description, versionId, clockInfo,
									Size<0x20000>, Offset<0x4400>, Banks<4>, 
									Size<0x2000>,Banks<4> > {};

extern const char MSP430F5529[] = "MSP430F5529";
extern const char MSP430F5528[] = "MSP430F5528";
extern const char MSP430F5519[] = "MSP430F5519";
//description - flash (size, offset, nr banks) - ram info (size, nr banks)

typedef MSP430F552x_128KB_type<MSP430F5529, 0x2955, ClockInfo_552x> MSP430F5529_type;
typedef MSP430F552x_128KB_type<MSP430F5528, 0x2855, ClockInfo_552x> MSP430F5528_type;
typedef MSP430F552x_128KB_type<MSP430F5519, 0x1955, ClockInfo_551x> MSP430F5519_type;

static const DeviceRegistrator<MSP430F5529_type> regMSP430F5529;
static const DeviceRegistrator<MSP430F5528_type> regMSP430F5528;
static const DeviceRegistrator<MSP430F5519_type> regMSP430F5519;

//96 KB types
template<
	const char* description,
	const unsigned int versionId,
	class clockInfo
>
struct MSP430F552x_96KB_type : MSP430F55xx_type<
								description, versionId, clockInfo,
								Size<0x18000>, Offset<0x4400>, Banks<3>, 
								Size<0x1800>,Banks<3> > {};

extern const char MSP430F5527[] = "MSP430F5527";
extern const char MSP430F5526[] = "MSP430F5526";
extern const char MSP430F5517[] = "MSP430F5517";
//description - flash (size, offset, nr banks) - ram info (size, nr banks)

typedef MSP430F552x_96KB_type<MSP430F5527, 0x2755, ClockInfo_552x> MSP430F5527_type;
typedef MSP430F552x_96KB_type<MSP430F5526, 0x2655, ClockInfo_552x> MSP430F5526_type;
typedef MSP430F552x_96KB_type<MSP430F5517, 0x1755, ClockInfo_551x> MSP430F5517_type;

static const DeviceRegistrator<MSP430F5527_type> regMSP430F5527;
static const DeviceRegistrator<MSP430F5526_type> regMSP430F5526;
static const DeviceRegistrator<MSP430F5517_type> regMSP430F5517;

//64 KB types
template<
	const char* description,
	const unsigned int versionId,
	class clockInfo
>
struct MSP430F552x_64KB_type : MSP430F55xx_type<
									description, versionId, clockInfo,
									Size<0x10000>, Offset<0x4400>, Banks<2>, 
									Size<0x1000>,Banks<2> >{};

extern const char MSP430F5525[] = "MSP430F5525";
extern const char MSP430F5524[] = "MSP430F5524";
extern const char MSP430F5515[] = "MSP430F5515";
extern const char MSP430F5514[] = "MSP430F5514";

//description - flash (size, offset, nr banks) - ram info (size, nr banks)
typedef MSP430F552x_64KB_type<MSP430F5525, 0x2555, ClockInfo_552x> MSP430F5525_type;
typedef MSP430F552x_64KB_type<MSP430F5524, 0x2455, ClockInfo_552x> MSP430F5524_type;
typedef MSP430F552x_64KB_type<MSP430F5515, 0x1555, ClockInfo_551x> MSP430F5515_type;
typedef MSP430F552x_64KB_type<MSP430F5514, 0x1455, ClockInfo_551x> MSP430F5514_type;

static const DeviceRegistrator<MSP430F5525_type> regMSP430F5525;
static const DeviceRegistrator<MSP430F5524_type> regMSP430F5524;
static const DeviceRegistrator<MSP430F5515_type> regMSP430F5515;
static const DeviceRegistrator<MSP430F5514_type> regMSP430F5514;

//64 KB types
template<
	const char* description,
	const unsigned int versionId,
	class clockInfo
>
struct MSP430F552x_32KB_type : MSP430F55xx_type<
									description, versionId, clockInfo,
									Size<0x8000>, Offset<0x8000>, Banks<2>, 
									Size<0x2000>,Banks<4> > {};

extern const char MSP430F5522[] = "MSP430F5522";
extern const char MSP430F5521[] = "MSP430F5521";
extern const char MSP430F5513[] = "MSP430F5513";

//description - flash (size, offset, nr banks) - ram info (size, nr banks)
typedef MSP430F552x_32KB_type<MSP430F5522, 0x2255, ClockInfo_552x> MSP430F5522_type;
typedef MSP430F552x_32KB_type<MSP430F5521, 0x2155, ClockInfo_552x> MSP430F5521_type;
typedef MSP430F552x_32KB_type<MSP430F5513, 0x1355, ClockInfo_551x> MSP430F5513_type;

static const DeviceRegistrator<MSP430F5522_type> regMSP430F5522;
static const DeviceRegistrator<MSP430F5521_type> regMSP430F5521;
static const DeviceRegistrator<MSP430F5513_type> regMSP430F5513;

//tiny usb 5500 - 5510
typedef MemoryInfo<
	Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
	Size<0x1000> , Offset<0x2400>, SegmentSize<0x1>, BankSize<0x2>, Banks<2>, 
	NoMask
> MSP430F55xx_Tiny_USB_SystemRamInfo;

template<class SizeType, class OffsetType>
struct TinyUsb55xxFlashMemory : MemoryInfo<
									Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
									SizeType , OffsetType, SegmentSize<0x200>, 
									BankSize<0x10000>, Banks<1>, NoMask> {};

typedef TinyUsb55xxFlashMemory<
	Size<0x8000>, 
	Offset<0x8000> 
> MSP430F55xx_Tiny_USB_32KB_Type;

typedef TinyUsb55xxFlashMemory<
	Size<0x6000>, 
	Offset<0xA000> 
> MSP430F55xx_Tiny_USB_24KB_Type;

typedef TinyUsb55xxFlashMemory<
	Size<0x4000>, 
	Offset<0xC000> 
> MSP430F55xx_Tiny_USB_16KB_Type;

typedef TinyUsb55xxFlashMemory<
	Size<0x2000>, 
	Offset<0xE000> 
> MSP430F55xx_Tiny_USB_8KB_Type;

//terms small, medium, large are possibly not correct - better proposal?
struct MSP430F550x_LargeEemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F550x_LargeEemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_B, 
		Eem::ADC10_A, Eem::RTC, Eem::USCI3, Eem::USCI2,
		Eem::USCI1, Eem::USCI0, Eem::USB, Eem::TB7_0,
		MSP430F55xxTimerTA3_2, MSP430F55xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct MSP430F550x_MediumEemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F550x_MediumEemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_B, 
		Eem::ADC10_A, Eem::RTC, Eem::Empty, Eem::Empty,
		Eem::USCI1, Eem::USCI0, Eem::USB, Eem::TB7_0,
		MSP430F55xxTimerTA3_2, MSP430F55xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct MSP430F550x_SmallEemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F550x_SmallEemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::ADC10_A, Eem::RTC, Eem::Empty, Eem::Empty,
		Eem::USCI1, Eem::USCI0, Eem::USB, Eem::TB7_0,
		MSP430F55xxTimerTA3_2, MSP430F55xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430F550x_LargeEemTimer, EmptyEemClockNames> MSP430F550x_LargeClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430F550x_MediumEemTimer, EmptyEemClockNames> MSP430F550x_MediumClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430F550x_SmallEemTimer, EmptyEemClockNames> MSP430F550x_SmallClockInfo;

template<
	const char* description,
	const unsigned int versionId,
	class ClockInfoType,
	class FlashMemoryInfoType
>
struct MSP430F55xx_Tiny_USB_type : Device<
										description, 
										ObjectId<0>,
										DefaultBits<20>, 
										regular, 
										MSP430F5xxx_Match<versionId>, 
										SmallEemMode,
										MSP430F5xxx_DefaultVoltageTestVpp,
										ClockInfoType,
										FunctionMappingXv2,
										FuncletMappingXv2,
										MemoryList<boost::tuple<
											FlashMemoryInfoType,
											MSP430F5xxx_InfoFlashMemoryInfo, 
											MSP430F5xxx_BootFlashMemoryInfo, 
											MSP430F5xxx_BootCodeMemoryInfo,
											UsbTypeRamInfo,
											MSP430F55xx_Tiny_USB_SystemRamInfo,
											MSP430F5xxx_peripherl16lbitMemoryInfo, 
											MSP430F5xxx_CPUMemoryInfo, 
											MSP430F5xxx_EEMMemoryInfo
										> >, //until C++0x, the space between the brackets is important
										MSP430F5xxx_Features,
										MSP430F5xxx_ExtFeatures
									> {};

extern const char MSP430F5500[] = "MSP430F5500";
extern const char MSP430F5501[] = "MSP430F5501";
extern const char MSP430F5502[] = "MSP430F5502";
extern const char MSP430F5503[] = "MSP430F5503";
extern const char MSP430F5504[] = "MSP430F5504";
extern const char MSP430F5505[] = "MSP430F5505";
extern const char MSP430F5506[] = "MSP430F5506";
extern const char MSP430F5507[] = "MSP430F5507";
extern const char MSP430F5508[] = "MSP430F5508";
extern const char MSP430F5509[] = "MSP430F5509";
extern const char MSP430F5510[] = "MSP430F5510";

typedef MSP430F55xx_Tiny_USB_type<
	MSP430F5500, 0x803B, 
	MSP430F550x_MediumClockInfo, MSP430F55xx_Tiny_USB_8KB_Type
> MSP430F5500_type;

typedef MSP430F55xx_Tiny_USB_type<
	MSP430F5501, 0x8032, 
	MSP430F550x_MediumClockInfo, MSP430F55xx_Tiny_USB_16KB_Type
> MSP430F5501_type;

typedef MSP430F55xx_Tiny_USB_type<
	MSP430F5502, 0x8033, 
	MSP430F550x_MediumClockInfo, MSP430F55xx_Tiny_USB_24KB_Type
> MSP430F5502_type;

typedef MSP430F55xx_Tiny_USB_type<
	MSP430F5503, 0x8034, 
	MSP430F550x_MediumClockInfo, MSP430F55xx_Tiny_USB_32KB_Type
> MSP430F5503_type;

typedef MSP430F55xx_Tiny_USB_type<
	MSP430F5504, 0x8035, 
	MSP430F550x_SmallClockInfo, MSP430F55xx_Tiny_USB_8KB_Type
> MSP430F5504_type;

typedef MSP430F55xx_Tiny_USB_type<
	MSP430F5505, 0x8036, 
	MSP430F550x_SmallClockInfo, MSP430F55xx_Tiny_USB_16KB_Type
> MSP430F5505_type;

typedef MSP430F55xx_Tiny_USB_type<
	MSP430F5506, 0x8037, 
	MSP430F550x_SmallClockInfo, MSP430F55xx_Tiny_USB_24KB_Type
> MSP430F5506_type;

typedef MSP430F55xx_Tiny_USB_type<
	MSP430F5507, 0x8038, 
	MSP430F550x_SmallClockInfo, MSP430F55xx_Tiny_USB_32KB_Type
> MSP430F5507_type;

typedef MSP430F55xx_Tiny_USB_type<
	MSP430F5508, 0x8039, 
	MSP430F550x_LargeClockInfo, MSP430F55xx_Tiny_USB_16KB_Type
> MSP430F5508_type;

typedef MSP430F55xx_Tiny_USB_type<
	MSP430F5509, 0x803A, 
	MSP430F550x_LargeClockInfo, MSP430F55xx_Tiny_USB_24KB_Type
> MSP430F5509_type;

typedef MSP430F55xx_Tiny_USB_type<
	MSP430F5510, 0x8031, 
	MSP430F550x_LargeClockInfo, MSP430F55xx_Tiny_USB_32KB_Type
> MSP430F5510_type;

static const DeviceRegistrator<MSP430F5500_type> regMSP430F5500;
static const DeviceRegistrator<MSP430F5501_type> regMSP430F5501;
static const DeviceRegistrator<MSP430F5502_type> regMSP430F5502;
static const DeviceRegistrator<MSP430F5503_type> regMSP430F5503;
static const DeviceRegistrator<MSP430F5504_type> regMSP430F5504;
static const DeviceRegistrator<MSP430F5505_type> regMSP430F5505;
static const DeviceRegistrator<MSP430F5506_type> regMSP430F5506;
static const DeviceRegistrator<MSP430F5507_type> regMSP430F5507;
static const DeviceRegistrator<MSP430F5508_type> regMSP430F5508;
static const DeviceRegistrator<MSP430F5509_type> regMSP430F5509;
static const DeviceRegistrator<MSP430F5510_type> regMSP430F5510;
