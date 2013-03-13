/*
 * MSP430F471xx.cpp
 *
 * Definition MSP430F46xx devices.
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

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F> MSP430F471xxIdMask;

template<const unsigned int fuses>
struct MSP430F471xx_Match : Match< IdCode<0x7FF4, 0x0, 0,0,0, 0, fuses>, MSP430F471xxIdMask > {};



struct MSP430F471xx_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F471xx_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::MCLKpin, Eem::SMCLKpin, Eem::ACLKpin, 
		Eem::Empty, Eem::FLASH_CTRL, Eem::USCI1, Eem::USCI0,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::BTRTC, 
		Eem::Empty, Eem::TB3, Eem::TA3, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};


typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430F471xx_EemTimer, StandardEemClockNames> MSP430F471xx_ClockInfo;



typedef Trigger<EMEX_HIGH, 0x8, 0x2, 0x8, 0x1, 0x0, 0x1, 0x1, 0x02, 0x00, 0x01> MSP430F471xx_HighTrigger;
typedef EemInfo<0x8, 0, 0, MSP430F471xx_HighTrigger, MediumSequencer> MSP430F471xx_HighEemMode;


typedef MemoryInfo<
			Name::information, FlashType, Mapped, Protectable, Bits16Type, 
			Size<0x100> , Offset<0x1000>, SegmentSize<0x40>, BankSize<0x40>, Banks<4>,
			NoMask, MemoryCreator<InformationFlashAccess>
		> MSP430F471xx_InfoFlashMemoryInfo;


template<const unsigned int FlashOffset, const unsigned int FlashSize, const unsigned int Ram2Size>
struct MSP430F471xx_MemoryModel : MemoryList<boost::tuple<
			MSP430F4xxx_MainFlashMemory< Size<FlashSize>, Offset<FlashOffset> >, 
			MSP430F471xx_InfoFlashMemoryInfo, 
			MSP430F4xxx_BootFlashMemoryInfo, 
			MSP430F4xxx_LcdMemoryInfo< Size<0x15> >,
			MSP430F4xxx_SystemRamInfo< Size<0x800> >,
			MSP430F4xxx_SystemRam2Info< Size<Ram2Size> >,
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_338f1f1fffff>,
			MSP430F4xxx_CPUMemoryInfo,
			MSP430F4xxx_EEMMemoryInfo
		> > {};

typedef MSP430F471xx_MemoryModel<0x2000, 0xE000,  0x1000> MemoryModel12;
typedef MSP430F471xx_MemoryModel<0x2100, 0x16F00, 0x1000> MemoryModel16;
typedef MSP430F471xx_MemoryModel<0x3100, 0x16F00, 0x2000> MemoryModel17;
typedef MSP430F471xx_MemoryModel<0x3100, 0x1CF00, 0x2000> MemoryModel18;
typedef MSP430F471xx_MemoryModel<0x2100, 0x1DF00, 0x1000> MemoryModel19;


typedef VoltageInfo<1800, 3600, 2200, 2500, 6000, 7000, false> MSP430FX471xxVoltage;


template<
	const char* description,
	const unsigned int fuses,
	class MemoryModelType
>
struct MSP430F471xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits20Type, 
		enhanced, 
		MSP430F471xx_Match<fuses>, 
		MSP430F471xx_HighEemMode,
		MSP430FX471xxVoltage,
		MSP430F471xx_ClockInfo,
		FunctionMappingX4xx,
		FuncletMappingX4xx,
		MemoryModelType,
		MSP430F46_7xx_Features
	> {};



extern const char MSP430F47163[] = "MSP430F47163";
extern const char MSP430F47173[] = "MSP430F47173";
extern const char MSP430F47183[] = "MSP430F47183";
extern const char MSP430F47193[] = "MSP430F47193";

extern const char MSP430F47126[] = "MSP430F47126";
extern const char MSP430F47166[] = "MSP430F47166";
extern const char MSP430F47176[] = "MSP430F47176";
extern const char MSP430F47186[] = "MSP430F47186";
extern const char MSP430F47196[] = "MSP430F47196";

extern const char MSP430F47127[] = "MSP430F47127";
extern const char MSP430F47167[] = "MSP430F47167";
extern const char MSP430F47177[] = "MSP430F47177";
extern const char MSP430F47187[] = "MSP430F47187";
extern const char MSP430F47197[] = "MSP430F47197";



typedef MSP430F471xx<MSP430F47163, 0xB, MemoryModel16> MSP430F47163_Type;
typedef MSP430F471xx<MSP430F47173, 0xA, MemoryModel17> MSP430F47173_Type;
typedef MSP430F471xx<MSP430F47183, 0x9, MemoryModel18> MSP430F47183_Type;
typedef MSP430F471xx<MSP430F47193, 0x8, MemoryModel19> MSP430F47193_Type;

typedef MSP430F471xx<MSP430F47126, 0xD, MemoryModel12> MSP430F47126_Type;
typedef MSP430F471xx<MSP430F47166, 0x7, MemoryModel16> MSP430F47166_Type;
typedef MSP430F471xx<MSP430F47176, 0x6, MemoryModel17> MSP430F47176_Type;
typedef MSP430F471xx<MSP430F47186, 0x5, MemoryModel18> MSP430F47186_Type;
typedef MSP430F471xx<MSP430F47196, 0x4, MemoryModel19> MSP430F47196_Type;

typedef MSP430F471xx<MSP430F47127, 0xC, MemoryModel12> MSP430F47127_Type;
typedef MSP430F471xx<MSP430F47167, 0x3, MemoryModel16> MSP430F47167_Type;
typedef MSP430F471xx<MSP430F47177, 0x2, MemoryModel17> MSP430F47177_Type;
typedef MSP430F471xx<MSP430F47187, 0x1, MemoryModel18> MSP430F47187_Type;
typedef MSP430F471xx<MSP430F47197, 0x0, MemoryModel19> MSP430F47197_Type;



static const DeviceRegistrator<MSP430F47163_Type> regMSP430F47163;
static const DeviceRegistrator<MSP430F47173_Type> regMSP430F47173;
static const DeviceRegistrator<MSP430F47183_Type> regMSP430F47183;
static const DeviceRegistrator<MSP430F47193_Type> regMSP430F47193;

static const DeviceRegistrator<MSP430F47126_Type> regMSP430F47126;
static const DeviceRegistrator<MSP430F47166_Type> regMSP430F47166;
static const DeviceRegistrator<MSP430F47176_Type> regMSP430F47176;
static const DeviceRegistrator<MSP430F47186_Type> regMSP430F47186;
static const DeviceRegistrator<MSP430F47196_Type> regMSP430F47196;

static const DeviceRegistrator<MSP430F47127_Type> regMSP430F47127;
static const DeviceRegistrator<MSP430F47167_Type> regMSP430F47167;
static const DeviceRegistrator<MSP430F47177_Type> regMSP430F47177;
static const DeviceRegistrator<MSP430F47187_Type> regMSP430F47187;
static const DeviceRegistrator<MSP430F47197_Type> regMSP430F47197;


