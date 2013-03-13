/*
 * MSP430F56_66xx.cpp
 *
 * Definition MSP430F56xx/MSP430F56xx devices.
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
#include "FlashMemoryAccessBase.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

const ClockPair MSP430F56_66xxTimerTA3_1 = {"Timer1_A3", 57};
const ClockPair MSP430F56_66xxTimerTA3_2 = {"Timer2_A3", 58};

struct LcdDacAcdUsbTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	LcdDacAcdUsbTimer() : EemTimerImpl(
		Eem::Empty, Eem::LCD_B, Eem::DAC12_0, Eem::COMP_B, 
		Eem::ADC12_A, Eem::RTC, Eem::Empty, Eem::Empty,
		Eem::USCI1, Eem::USCI0, Eem::USB, Eem::TB7_0,
		MSP430F56_66xxTimerTA3_2, MSP430F56_66xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct LcdAcdUsbTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	LcdAcdUsbTimer() : EemTimerImpl(
		Eem::Empty, Eem::LCD_B, Eem::Empty, Eem::COMP_B, 
		Eem::ADC12_A, Eem::RTC, Eem::Empty, Eem::Empty,
		Eem::USCI1, Eem::USCI0, Eem::USB, Eem::TB7_0,
		MSP430F56_66xxTimerTA3_2, MSP430F56_66xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct LcdUsbTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	LcdUsbTimer() : EemTimerImpl(
		Eem::Empty, Eem::LCD_B, Eem::Empty, Eem::COMP_B, 
		Eem::Empty, Eem::RTC, Eem::Empty, Eem::Empty,
		Eem::USCI1, Eem::USCI0, Eem::USB, Eem::TB7_0,
		MSP430F56_66xxTimerTA3_2, MSP430F56_66xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct DacAcdUsbTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	DacAcdUsbTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::DAC12_0, Eem::COMP_B, 
		Eem::ADC12_A, Eem::RTC, Eem::Empty, Eem::Empty,
		Eem::USCI1, Eem::USCI0, Eem::USB, Eem::TB7_0,
		MSP430F56_66xxTimerTA3_2, MSP430F56_66xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct AcdUsbTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	AcdUsbTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_B, 
		Eem::ADC12_A, Eem::RTC, Eem::Empty, Eem::Empty,
		Eem::USCI1, Eem::USCI0, Eem::USB, Eem::TB7_0,
		MSP430F56_66xxTimerTA3_2, MSP430F56_66xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct UsbTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	UsbTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_B, 
		Eem::Empty, Eem::RTC, Eem::Empty, Eem::Empty,
		Eem::USCI1, Eem::USCI0, Eem::USB, Eem::TB7_0,
		MSP430F56_66xxTimerTA3_2, MSP430F56_66xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

template<const unsigned int nrBanks>
struct MSP430F56_66xx_MainFlashMemory : MemoryInfo<
											Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
											Size<nrBanks*0x10000> , Offset<0x8000>, SegmentSize<0x200>, 
											BankSize<0x10000>, Banks<nrBanks>, NoMask, 
											MemoryCreator<FlashMemoryAccess2ByteAligned> > {};


template<const unsigned int nrBanks>
struct MSP430Fx65x_MainFlashMemory : MemoryInfo<
											Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
											Size<nrBanks*0x20000> , Offset<0x8000>, SegmentSize<0x200>, 
											BankSize<0x20000>, Banks<nrBanks>, NoMask, 
											MemoryCreator<FlashMemoryAccess2ByteAligned> > {};

template<
	const char* description,
	const unsigned int versionId,
	typename ClockInfo,
	const unsigned int nrBanks
>
struct MSP430F56_66xx : Device<
							description, 
							ObjectId<0>,
							DefaultBits20Type,
							regular, 
							MSP430F5xxx_Match<versionId>, 
							LargeEemMode,
							MSP430F5xxx_DefaultVoltageTestVpp,
							ClockInfo,
							FunctionMappingXv2,
							FuncletMappingXv2WordMode,
							MemoryList<boost::tuple<
								MSP430F56_66xx_MainFlashMemory<nrBanks>,
								MSP430F5xxx_InfoFlashMemoryInfo, 
								MSP430F5xxx_BootFlashMemoryInfo, 
								MSP430F5xxx_BootCodeMemoryInfo,
								UsbTypeRamInfo,
								UsbTypeSystemRamInfo<Size<0x4000>, Banks<4> >,
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
	typename Ram2OffsetType,
	typename Ram2SizeType
>
struct MSP430F565_665x : Device<
							description, 
							ObjectId<0>,
							DefaultBits20Type,
							regular, 
							MSP430F5xxx_Match<versionId>, 
							LargeEemMode,
							MSP430F5xxx_DefaultVoltageTestVpp,
							ClockInfo,
							FunctionMappingXv2,
							FuncletMappingXv2WordMode,
							MemoryList<boost::tuple<
								MSP430Fx65x_MainFlashMemory<nrBanks>,
								MSP430F5xxx_InfoFlashMemoryInfo, 
								MSP430F5xxx_BootFlashMemoryInfo, 
								MSP430F5xxx_BootCodeMemoryInfo,
								UsbTypeRamInfo,
								UsbTypeSystemRamInfo< Size<0x4000>, Banks<1> >,
								MSP430F5xxx_SystemRam2Info< Ram2OffsetType, Ram2SizeType >,
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
								0x4020	  // 3V Test reg value to disable LPMx.5 
							>
						> {};

typedef ClockInfo<GCC_EXTENDED, 0x040F, LcdDacAcdUsbTimer, EmptyEemClockNames> LcdDacAcdUsbClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x040F, LcdAcdUsbTimer, EmptyEemClockNames> LcdAcdUsbClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x040F, LcdUsbTimer, EmptyEemClockNames> LcdUsbClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x040F, DacAcdUsbTimer, EmptyEemClockNames> DacAcdUsbClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x040F, AcdUsbTimer, EmptyEemClockNames> AcdUsbClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x040F, UsbTimer, EmptyEemClockNames> UsbClockInfo;


extern const char MSP430F5630[] = "MSP430F5630";
extern const char MSP430F5631[] = "MSP430F5631";
extern const char MSP430F5632[] = "MSP430F5632";
extern const char MSP430F5633[] = "MSP430F5633";
extern const char MSP430F5634[] = "MSP430F5634";
extern const char MSP430F5635[] = "MSP430F5635";
extern const char MSP430F5636[] = "MSP430F5636";
extern const char MSP430F5637[] = "MSP430F5637";
extern const char MSP430F5638[] = "MSP430F5638";

extern const char MSP430F6630[] = "MSP430F6630";
extern const char MSP430F6631[] = "MSP430F6631";
extern const char MSP430F6632[] = "MSP430F6632";
extern const char MSP430F6633[] = "MSP430F6633";
extern const char MSP430F6634[] = "MSP430F6634";
extern const char MSP430F6635[] = "MSP430F6635";
extern const char MSP430F6636[] = "MSP430F6636";
extern const char MSP430F6637[] = "MSP430F6637";
extern const char MSP430F6638[] = "MSP430F6638";

extern const char MSP430F5658[] = "MSP430F5658";
extern const char MSP430F5659[] = "MSP430F5659";

extern const char MSP430F6658[] = "MSP430F6658";
extern const char MSP430F6659[] = "MSP430F6659";

//description, versionId, clock info, number flash banks
typedef MSP430F56_66xx<MSP430F5630, 0x803C, UsbClockInfo, 2 > MSP430F5630_type;
typedef MSP430F56_66xx<MSP430F5631, 0x803E, UsbClockInfo, 3 > MSP430F5631_type;
typedef MSP430F56_66xx<MSP430F5632, 0x8040, UsbClockInfo, 4 > MSP430F5632_type;

typedef MSP430F56_66xx<MSP430F5633, 0x8042, AcdUsbClockInfo, 2 > MSP430F5633_type;
typedef MSP430F56_66xx<MSP430F5634, 0x8044, AcdUsbClockInfo, 3 > MSP430F5634_type;
typedef MSP430F56_66xx<MSP430F5635, 0x800E, AcdUsbClockInfo, 4 > MSP430F5635_type;

typedef MSP430F56_66xx<MSP430F5636, 0x8010, DacAcdUsbClockInfo, 2 > MSP430F5636_type;
typedef MSP430F56_66xx<MSP430F5637, 0x8012, DacAcdUsbClockInfo, 3 > MSP430F5637_type;
typedef MSP430F56_66xx<MSP430F5638, 0x8014, DacAcdUsbClockInfo, 4 > MSP430F5638_type;

typedef MSP430F56_66xx<MSP430F6630, 0x8046, LcdUsbClockInfo, 2 > MSP430F6630_type;
typedef MSP430F56_66xx<MSP430F6631, 0x8048, LcdUsbClockInfo, 3 > MSP430F6631_type;
typedef MSP430F56_66xx<MSP430F6632, 0x804A, LcdUsbClockInfo, 4 > MSP430F6632_type;

typedef MSP430F56_66xx<MSP430F6633, 0x804C, LcdAcdUsbClockInfo, 2 > MSP430F6633_type;
typedef MSP430F56_66xx<MSP430F6634, 0x804E, LcdAcdUsbClockInfo, 3 > MSP430F6634_type;
typedef MSP430F56_66xx<MSP430F6635, 0x8016, LcdAcdUsbClockInfo, 4 > MSP430F6635_type;

typedef MSP430F56_66xx<MSP430F6636, 0x8018, LcdDacAcdUsbClockInfo, 2 > MSP430F6636_type;
typedef MSP430F56_66xx<MSP430F6637, 0x801A, LcdDacAcdUsbClockInfo, 3 > MSP430F6637_type;
typedef MSP430F56_66xx<MSP430F6638, 0x801C, LcdDacAcdUsbClockInfo, 4 > MSP430F6638_type;

typedef MSP430F565_665x<MSP430F5658, 0x8131, DacAcdUsbClockInfo, 3, Offset<0xF8000>, Size<0x8000> > MSP430F5658_type;
typedef MSP430F565_665x<MSP430F5659, 0x8130, DacAcdUsbClockInfo, 4, Offset<0xF0000>, Size<0x10000> > MSP430F5659_type;

typedef MSP430F565_665x<MSP430F6658, 0x812C, LcdDacAcdUsbClockInfo, 3, Offset<0xF8000>, Size<0x8000> > MSP430F6658_type;
typedef MSP430F565_665x<MSP430F6659, 0x812B, LcdDacAcdUsbClockInfo, 4, Offset<0xF0000>, Size<0x10000> > MSP430F6659_type;

static const DeviceRegistrator<MSP430F5630_type> regMSP430F5630;
static const DeviceRegistrator<MSP430F5631_type> regMSP430F5631;
static const DeviceRegistrator<MSP430F5632_type> regMSP430F5632;
static const DeviceRegistrator<MSP430F5633_type> regMSP430F5633;
static const DeviceRegistrator<MSP430F5634_type> regMSP430F5634;
static const DeviceRegistrator<MSP430F5635_type> regMSP430F5635;
static const DeviceRegistrator<MSP430F5636_type> regMSP430F5636;
static const DeviceRegistrator<MSP430F5637_type> regMSP430F5637;
static const DeviceRegistrator<MSP430F5638_type> regMSP430F5638;

static const DeviceRegistrator<MSP430F5658_type> regMSP430F5658;
static const DeviceRegistrator<MSP430F5659_type> regMSP430F5659;

static const DeviceRegistrator<MSP430F6630_type> regMSP430F6630;
static const DeviceRegistrator<MSP430F6631_type> regMSP430F6631;
static const DeviceRegistrator<MSP430F6632_type> regMSP430F6632;
static const DeviceRegistrator<MSP430F6633_type> regMSP430F6633;
static const DeviceRegistrator<MSP430F6634_type> regMSP430F6634;
static const DeviceRegistrator<MSP430F6635_type> regMSP430F6635;
static const DeviceRegistrator<MSP430F6636_type> regMSP430F6636;
static const DeviceRegistrator<MSP430F6637_type> regMSP430F6637;
static const DeviceRegistrator<MSP430F6638_type> regMSP430F6638;

static const DeviceRegistrator<MSP430F6658_type> regMSP430F6658;
static const DeviceRegistrator<MSP430F6659_type> regMSP430F6659;
