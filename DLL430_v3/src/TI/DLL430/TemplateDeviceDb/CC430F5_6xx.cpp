/*
 * CC4305_6xx.cpp
 *
 * Definition CC430F5xx and CC430F5xx devices.
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

const ClockPair CC430F5_6xxTimerTA3_1 = {"Timer1_A3", 57};


struct CC430F6xx_EemTimer_ADC_LCD : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	CC430F6xx_EemTimer_ADC_LCD() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::AES, Eem::COMP_B, 
		Eem::ADC12_A, Eem::RTC, Eem::USCI3, Eem::USCI2,
		Eem::USCI1, Eem::USCI0, Eem::LCD_B, Eem::Empty,
		Eem::Empty, CC430F5_6xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct CC430F6xx_EemTimer_LCD : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	CC430F6xx_EemTimer_LCD() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::AES, Eem::COMP_B, 
		Eem::Empty, Eem::RTC, Eem::USCI3, Eem::USCI2,
		Eem::USCI1, Eem::USCI0, Eem::LCD_B, Eem::Empty,
		Eem::Empty, CC430F5_6xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct CC430F6xx_EemTimer_ADC : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	CC430F6xx_EemTimer_ADC() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::AES, Eem::COMP_B, 
		Eem::ADC12_A, Eem::RTC, Eem::USCI3, Eem::USCI2,
		Eem::USCI1, Eem::USCI0, Eem::Empty, Eem::Empty,
		Eem::Empty, CC430F5_6xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct CC430F6xx_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	CC430F6xx_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::AES, Eem::COMP_B, 
		Eem::Empty, Eem::RTC, Eem::USCI3, Eem::USCI2,
		Eem::USCI1, Eem::USCI0, Eem::Empty, Eem::Empty,
		Eem::Empty, CC430F5_6xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};


typedef ClockInfo<GCC_EXTENDED, 0x2407, CC430F6xx_EemTimer_ADC_LCD, EmptyEemClockNames> CC430F6xx_ClockInfo_ADC_LCD;
typedef ClockInfo<GCC_EXTENDED, 0x2407, CC430F6xx_EemTimer_LCD, EmptyEemClockNames> CC430F6xx_ClockInfo_LCD;
typedef ClockInfo<GCC_EXTENDED, 0x2407, CC430F6xx_EemTimer_ADC, EmptyEemClockNames> CC430F6xx_ClockInfo_ADC;
typedef ClockInfo<GCC_EXTENDED, 0x2407, CC430F6xx_EemTimer, EmptyEemClockNames> CC430F6xx_ClockInfo;



template<
	const char* description,
	const unsigned int versionId,
	class ClockInfoType,
	class MainFlashMemoryType,
	class SystemRamMemoryType
>
struct CC430F5_6xx : Device<
						description, 
						ObjectId<0>,
						DefaultBits20Type, 
						regular, 
						MSP430F5xxx_Match<versionId>, 
						SmallEemMode,
						MSP430F5xxx_DefaultVoltageTestVpp,
						ClockInfoType,
						FunctionMappingXv2,
						FuncletMappingXv2,
						MemoryList<boost::tuple<
							MainFlashMemoryType,
							MSP430F5xxx_InfoFlashMemoryInfo, 
							MSP430F5xxx_BootFlashMemoryInfo, 
							MSP430F5xxx_BootCodeMemoryInfo,
							SystemRamMemoryType,
							MSP430F5xxx_peripherl16lbitMemoryInfo, 
							MSP430F5xxx_CPUMemoryInfo, 
							MSP430F5xxx_EEMMemoryInfo 
						> >, //until C++0x, the space between the brackets is important
						MSP430F5xxx_Features
					> {};



//Backup ram is defined as second ram area, as this might not be accessible when connecting to device
struct CC430F614x_SystemRamInfo2 : MemoryInfo<
										Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
										Size<0x80> , Offset<0x1C00>, SegmentSize<0x1>, 
										BankSize<0x0>, Banks<1>, NoMask> {};


template<
	const char* description,
	const unsigned int versionId,
	class ClockInfoType,
	class MainFlashMemoryType,
	class SystemRamMemoryType
>
struct CC430F614x : Device<
						description, 
						ObjectId<0>,
						DefaultBits20Type, 
						regular, 
						MSP430F5xxx_Match<versionId>, 
						SmallEemMode,
						MSP430F5xxx_DefaultVoltageTestVpp,
						ClockInfoType,
						FunctionMappingXv2,
						FuncletMappingXv2,
						MemoryList<boost::tuple<
							MainFlashMemoryType,
							MSP430F5xxx_InfoFlashMemoryInfo, 
							MSP430F5xxx_BootFlashMemoryInfo, 
							MSP430F5xxx_BootCodeMemoryInfo,
							SystemRamMemoryType,
							CC430F614x_SystemRamInfo2,
							MSP430F5xxx_peripherl16lbitMemoryInfo, 
							MSP430F5xxx_CPUMemoryInfo, 
							MSP430F5xxx_EEMMemoryInfo 
						> >, //until C++0x, the space between the brackets is important
						MSP430F5xxx_Features,
						NoExtendedFeatures,
						PowerSettings<
							0x0, // Test reg mask
							0x0, // Test reg value to enable LPMx.5 
							0x0, // Test reg value to disable LPMx.5 
							0x4020,	  // 3V Test reg mask 
							0x4020,	  // 3V Test reg value to enable LPMx.5 
							0x4020>	  // 3V Test reg value to disable LPMx.5 
					> {};


template<class SizeType, class OffsetType>
struct CC430F5_6xx_MainFlashMemory : MemoryInfo<
										Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
										SizeType , OffsetType, SegmentSize<0x200>, 
										BankSize<0x10000>, Banks<2>, NoMask > {};

template<class SizeType, class BanksType>
struct CC430F5_6xx_SystemRamInfo : MemoryInfo<
										Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
										SizeType , Offset<0x1C00>, SegmentSize<0x1>, 
										BankSize<0x0>, BanksType, NoMask> {};

template<class SizeType, class BanksType>
struct CC430F614x_SystemRamInfo : MemoryInfo<
										Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
										SizeType , Offset<0x1C80>, SegmentSize<0x1>, 
										BankSize<0x0>, BanksType, NoMask> {};


typedef CC430F5_6xx_MainFlashMemory<Size<0x8000>, Offset<0x8000> > CC430F5_6xx_Main_32KB;
typedef CC430F5_6xx_MainFlashMemory<Size<0x4000>, Offset<0xC000> > CC430F5_6xx_Main_16KB;
typedef CC430F5_6xx_MainFlashMemory<Size<0x2000>, Offset<0xE000> > CC430F5_6xx_Main_8KB;

typedef CC430F5_6xx_SystemRamInfo<Size<0x1000>, Banks<2> > CC430F5_6xx_System_4KB;
typedef CC430F5_6xx_SystemRamInfo<Size<0x800>, Banks<1> > CC430F5_6xx_System_2KB;

typedef CC430F614x_SystemRamInfo<Size<0xF80>, Banks<2> > CC430F614x_System_4KB;
typedef CC430F614x_SystemRamInfo<Size<0x780>, Banks<1> > CC430F614x_System_2KB;


extern const char CC430F6137[] = "CC430F6137";
extern const char CC430F6135[] = "CC430F6135";
extern const char CC430F6127[] = "CC430F6127";
extern const char CC430F6126[] = "CC430F6126";
extern const char CC430F6125[] = "CC430F6125";
extern const char CC430F5137[] = "CC430F5137";
extern const char CC430F5135[] = "CC430F5135";
extern const char CC430F5133[] = "CC430F5133";

extern const char CC430F6147[] = "CC430F6147";
extern const char CC430F6145[] = "CC430F6145";
extern const char CC430F6143[] = "CC430F6143";
extern const char CC430F5147[] = "CC430F5147";
extern const char CC430F5145[] = "CC430F5145";
extern const char CC430F5143[] = "CC430F5143";
extern const char CC430F5125[] = "CC430F5125";
extern const char CC430F5123[] = "CC430F5123";


typedef CC430F5_6xx< 
	CC430F6137, 0x3761, CC430F6xx_ClockInfo_ADC_LCD, CC430F5_6xx_Main_32KB, CC430F5_6xx_System_4KB
> CC430F6137_type;

typedef CC430F5_6xx< 
	CC430F6135, 0x3561, CC430F6xx_ClockInfo_ADC_LCD, CC430F5_6xx_Main_16KB, CC430F5_6xx_System_2KB
> CC430F6135_type;

typedef CC430F5_6xx< 
	CC430F6127, 0x2761, CC430F6xx_ClockInfo_ADC_LCD, CC430F5_6xx_Main_32KB, CC430F5_6xx_System_4KB
> CC430F6127_type;

typedef CC430F5_6xx< 
	CC430F6126, 0x2661, CC430F6xx_ClockInfo_LCD, CC430F5_6xx_Main_32KB, CC430F5_6xx_System_2KB
> CC430F6126_type;

typedef CC430F5_6xx< 
	CC430F6125, 0x2561, CC430F6xx_ClockInfo_LCD, CC430F5_6xx_Main_16KB, CC430F5_6xx_System_2KB
> CC430F6125_type;

typedef CC430F5_6xx< 
	CC430F5137, 0x3751, CC430F6xx_ClockInfo_ADC, CC430F5_6xx_Main_32KB, CC430F5_6xx_System_4KB
> CC430F5137_type;

typedef CC430F5_6xx< 
	CC430F5135, 0x3551, CC430F6xx_ClockInfo_ADC, CC430F5_6xx_Main_16KB, CC430F5_6xx_System_2KB
> CC430F5135_type;

typedef CC430F5_6xx< 
	CC430F5133, 0x3351, CC430F6xx_ClockInfo_ADC, CC430F5_6xx_Main_8KB, CC430F5_6xx_System_2KB
> CC430F5133_type;




typedef CC430F614x< 
	CC430F6147, 0x8135, CC430F6xx_ClockInfo_ADC_LCD, CC430F5_6xx_Main_32KB, CC430F614x_System_4KB
> CC430F6147_type;

typedef CC430F614x< 
	CC430F6145, 0x8136, CC430F6xx_ClockInfo_ADC_LCD, CC430F5_6xx_Main_16KB, CC430F614x_System_2KB
> CC430F6145_type;

typedef CC430F614x< 
	CC430F6143, 0x8137, CC430F6xx_ClockInfo_ADC_LCD, CC430F5_6xx_Main_8KB, CC430F614x_System_2KB
> CC430F6143_type;

typedef CC430F614x< 
	CC430F5147, 0x8138, CC430F6xx_ClockInfo_ADC, CC430F5_6xx_Main_32KB, CC430F614x_System_4KB
> CC430F5147_type;

typedef CC430F614x< 
	CC430F5145, 0x8139, CC430F6xx_ClockInfo_ADC, CC430F5_6xx_Main_16KB, CC430F614x_System_2KB
> CC430F5145_type;

typedef CC430F614x< 
	CC430F5143, 0x813A, CC430F6xx_ClockInfo_ADC, CC430F5_6xx_Main_8KB, CC430F614x_System_2KB
> CC430F5143_type;

typedef CC430F614x< 
	CC430F5125, 0x813B, CC430F6xx_ClockInfo, CC430F5_6xx_Main_16KB, CC430F614x_System_2KB
> CC430F5125_type;

typedef CC430F614x< 
	CC430F5123, 0x813C, CC430F6xx_ClockInfo, CC430F5_6xx_Main_8KB, CC430F614x_System_2KB
> CC430F5123_type;


static const DeviceRegistrator<CC430F6137_type> regCC430F6137;
static const DeviceRegistrator<CC430F6127_type> regCC430F6127;
static const DeviceRegistrator<CC430F6135_type> regCC430F6135;
static const DeviceRegistrator<CC430F6126_type> regCC430F6126;
static const DeviceRegistrator<CC430F6125_type> regCC430F6125;
static const DeviceRegistrator<CC430F5137_type> regCC430F5137;
static const DeviceRegistrator<CC430F5135_type> regCC430F5135;
static const DeviceRegistrator<CC430F5133_type> regCC430F5133;

static const DeviceRegistrator<CC430F6147_type> regCC430F6147;
static const DeviceRegistrator<CC430F6145_type> regCC430F6145;
static const DeviceRegistrator<CC430F6143_type> regCC430F6143;
static const DeviceRegistrator<CC430F5147_type> regCC430F5147;
static const DeviceRegistrator<CC430F5145_type> regCC430F5145;
static const DeviceRegistrator<CC430F5143_type> regCC430F5143;
static const DeviceRegistrator<CC430F5125_type> regCC430F5125;
static const DeviceRegistrator<CC430F5123_type> regCC430F5123;
