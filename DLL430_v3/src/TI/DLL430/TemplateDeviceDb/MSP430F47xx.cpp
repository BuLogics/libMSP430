/*
 * MSP430F47xx.cpp
 *
 * Definition MSP430F47xx devices.
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

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x07> MSP430F47xxIdMask;

template<const unsigned int versionId, const char config, const unsigned int fuses>
struct MSP430F47xx_Match : Match< IdCode<versionId, 0x0, 0,0,0, config, fuses>, MSP430F47xxIdMask> {};



struct MSP430F47x_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F47x_EemTimer() : EemTimerImpl(
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

struct MSP430F47xx_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F47xx_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::SD16, Eem::FLASH_CTRL, Eem::USCI1, Eem::USCI0,
		Eem::Empty, Eem::Empty, Eem::LCD_FREQ, Eem::BT, 
		Eem::Empty, Eem::TB_MCLK, Eem::TA_SMCLK, Eem::WDT_ACLK,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430F47x_EemTimer, StandardEemClockNames> MSP430F47x_ClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430F47xx_EemTimer, StandardEemClockNames> MSP430F47xx_ClockInfo;


typedef VoltageInfo<1800, 3600, 2200, 2700, 6000, 7000, false> MSP430FX47xVoltage;
typedef VoltageInfo<1800, 3600, 2200, 2500, 6000, 7000, false> MSP430FX47xxVoltage;

typedef MemoryInfo<
	Name::information, FlashType, Mapped, Protectable, Bits16Type, 
	Size<0x100> , Offset<0x1000>, SegmentSize<0x40>, BankSize<0x40>, Banks<4>,
	NoMask, MemoryCreator<InformationFlashAccess>
> MSP430F47xx_InfoFlashMemoryInfo;


template<class FlashOffset, class FlashSize, 
		 class RamSize>
struct MemoryModel : MemoryList<boost::tuple<
			MSP430F4xxx_MainFlashMemory<FlashSize, FlashOffset>, 
			MSP430F47xx_InfoFlashMemoryInfo, 
			MSP430F4xxx_BootFlashMemoryInfo, 
			MSP430F4xxx_LcdMemoryInfo< Size<0x20> >,
			MSP430F4xxx_SystemRamInfo<RamSize>, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_338f1f8fffff>,
			MSP430F4xxx_CPUMemoryInfo,
			MSP430F4xxx_EEMMemoryInfo> 
		> {};

typedef MemoryModel< Offset<0x8000>, Size<0x8000>, Size<0x800> > MemoryModel477;
typedef MemoryModel< Offset<0x4000>, Size<0xC000>, Size<0x800> > MemoryModel478;
typedef MemoryModel< Offset<0x1100>, Size<0xEF00>, Size<0x800> > MemoryModel479;
typedef MemoryModel< Offset<0x1100>, Size<0xEF00>, Size<0xA00> > MemoryModel479x;

template<
	const char* description,
	const unsigned int versionId,
	const char config,
	const unsigned int fuses,
	class VoltageType,
	class ClockInfoType,
	class MemoryModelType,
	class ExtendedFeatures = NoExtendedFeatures
>
struct MSP430F47xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		enhanced, 
		MSP430F47xx_Match<versionId, config, fuses>, 
		LowEemMode,
		VoltageType,
		ClockInfoType,
		FunctionMapping4xx,
		FuncletMapping4xx,
		MemoryModelType,
		MSP430F46_7xx_Features,
		ExtendedFeatures
	> {};

extern const char MSP430F477[] = "MSP430F477";
extern const char MSP430F478[] = "MSP430F478";
extern const char MSP430F479[] = "MSP430F479";

extern const char MSP430FG477[] = "MSP430FG477";
extern const char MSP430FG478[] = "MSP430FG478";
extern const char MSP430FG479[] = "MSP430FG479";

extern const char MSP430F4783[] = "MSP430F4783";
extern const char MSP430F4784[] = "MSP430F4784";
extern const char MSP430F4793[] = "MSP430F4793";
extern const char MSP430F4794[] = "MSP430F4794";


typedef MSP430F47xx<MSP430F477, 0x79F4, 'G', 0x6, MSP430FX47xVoltage, MSP430F47x_ClockInfo, MemoryModel477> MSP430F477_Type;
typedef MSP430F47xx<MSP430F478, 0x79F4, 'G', 0x5, MSP430FX47xVoltage, MSP430F47x_ClockInfo, MemoryModel478> MSP430F478_Type;
typedef MSP430F47xx<MSP430F479, 0x79F4, 'G', 0x4, MSP430FX47xVoltage, MSP430F47x_ClockInfo, MemoryModel479> MSP430F479_Type;

typedef MSP430F47xx<MSP430FG477, 0x79F4, 'G', 0x2, MSP430FX47xVoltage, MSP430F47x_ClockInfo, MemoryModel477> MSP430FG477_Type;
typedef MSP430F47xx<MSP430FG478, 0x79F4, 'G', 0x1, MSP430FX47xVoltage, MSP430F47x_ClockInfo, MemoryModel478> MSP430FG478_Type;
typedef MSP430F47xx<MSP430FG479, 0x79F4, 'G', 0x0, MSP430FX47xVoltage, MSP430F47x_ClockInfo, MemoryModel479> MSP430FG479_Type;

typedef MSP430F47xx<MSP430F4783, 0x49F4, 0x2, 0x7, MSP430FX47xxVoltage, 
					MSP430F47xx_ClockInfo, MemoryModel478, MSP430F4xxx_SyncExtFeatures> MSP430F4783_Type;

typedef MSP430F47xx<MSP430F4793, 0x49F4, 0x2, 0x4, MSP430FX47xxVoltage, 
					MSP430F47xx_ClockInfo, MemoryModel479x, MSP430F4xxx_SyncExtFeatures> MSP430F4793_Type;

typedef MSP430F47xx<MSP430F4784, 0x49F4, 0x2, 0x3, MSP430FX47xxVoltage, 
					MSP430F47xx_ClockInfo, MemoryModel478, MSP430F4xxx_SyncExtFeatures> MSP430F4784_Type;

typedef MSP430F47xx<MSP430F4794, 0x49F4, 0x2, 0x0, MSP430FX47xxVoltage, 
					MSP430F47xx_ClockInfo, MemoryModel479x, MSP430F4xxx_SyncExtFeatures> MSP430F4794_Type;


static const DeviceRegistrator< MSP430F477_Type > regMSP430F477;
static const DeviceRegistrator< MSP430F478_Type > regMSP430F478;
static const DeviceRegistrator< MSP430F479_Type > regMSP430F479;

static const DeviceRegistrator< MSP430FG477_Type > regMSP430FG477;
static const DeviceRegistrator< MSP430FG478_Type > regMSP430FG478;
static const DeviceRegistrator< MSP430FG479_Type > regMSP430FG479;

static const DeviceRegistrator< MSP430F4783_Type > regMSP430F4783;
static const DeviceRegistrator< MSP430F4784_Type > regMSP430F4784;
static const DeviceRegistrator< MSP430F4793_Type > regMSP430F4793;
static const DeviceRegistrator< MSP430F4794_Type > regMSP430F4794;
