/*
 * MSP430F46xx.cpp
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

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x03> MSP430FG46xxIdMask;

template<const unsigned int fuses>
struct MSP430F46xx_Match : Match< IdCode<0x6FF4, 0x0, 0,0,0, 'G', fuses>, MSP430FG46xxIdMask > {};



struct MSP430FG46xx_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FG46xx_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::MCLKpin, Eem::SMCLKpin, Eem::ACLKpin, 
		Eem::ADC12, Eem::FLASH_CTRL, Eem::USCI1, Eem::USCI0,
		Eem::Empty, Eem::Empty, Eem::LCD_FREQ, Eem::BT, 
		Eem::Empty, Eem::TB, Eem::TA, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct MSP430FG461x1_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FG461x1_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::MCLKpin, Eem::SMCLKpin, Eem::ACLKpin, 
		Eem::Empty, Eem::FLASH_CTRL, Eem::USCI1, Eem::USCI0,
		Eem::Empty, Eem::Empty, Eem::LCD_FREQ, Eem::BT, 
		Eem::Empty, Eem::TB, Eem::TA, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430FG46xx_EemTimer, StandardEemClockNames> MSP430FG46xx_ClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430FG461x1_EemTimer, StandardEemClockNames> MSP430FG461x1_ClockInfo;

template<const unsigned int FlashOffset, const unsigned int FlashSize, const unsigned int Ram2Size>
struct MSP430F46xx_MemoryModel : MemoryList<boost::tuple<
			MSP430F4xxx_MainFlashMemory< Size<FlashSize>, Offset<FlashOffset> >, 
			MSP430F4xxx_InfoFlashMemoryInfo, 
			MSP430F4xxx_BootFlashMemoryInfo, 
			MSP430F4xxx_LcdMemoryInfo< Size<0xD> >,
			MSP430F4xxx_SystemRamInfo< Size<0x800> >,
			MSP430F4xxx_SystemRam2Info< Size<Ram2Size> >,
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_33bf13bfff30>,
			MSP430F4xxx_CPUMemoryInfo,
			MSP430F4xxx_EEMMemoryInfo
		> > {};

typedef MSP430F46xx_MemoryModel<0x2100, 0x16F00, 0x1000> MemoryModel16;
typedef MSP430F46xx_MemoryModel<0x3100, 0x16F00, 0x2000> MemoryModel17;
typedef MSP430F46xx_MemoryModel<0x3100, 0x1CF00, 0x2000> MemoryModel18;
typedef MSP430F46xx_MemoryModel<0x2100, 0x1DF00, 0x1000> MemoryModel19;


template<
	const char* description,
	const unsigned int fuses,
	class ClockInfoType,
	class MemoryModelType
>
struct MSP430F46xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits20Type, 
		enhanced, 
		MSP430F46xx_Match<fuses>, 
		HighEemMode,
		MSP430F4xxx_DefaultVoltageNoTestVpp,
		ClockInfoType,
		FunctionMappingX4xx,
		FuncletMappingX4xx,
		MemoryModelType,
		MSP430F46_7xx_Features
	> {};


extern const char MSP430FG4616[] = "MSP430FG4616";
extern const char MSP430FG4617[] = "MSP430FG4617";
extern const char MSP430FG4618[] = "MSP430FG4618";
extern const char MSP430FG4619[] = "MSP430FG4619";

extern const char MSP430F4616[] = "MSP430F4616";
extern const char MSP430F4617[] = "MSP430F4617";
extern const char MSP430F4618[] = "MSP430F4618";
extern const char MSP430F4619[] = "MSP430F4619";

extern const char MSP430F46161[] = "MSP430F46161";
extern const char MSP430F46171[] = "MSP430F46171";
extern const char MSP430F46181[] = "MSP430F46181";
extern const char MSP430F46191[] = "MSP430F46191";



typedef MSP430F46xx<MSP430FG4616, 0x3, MSP430FG46xx_ClockInfo, MemoryModel16> MSP430FG4616_Type;
typedef MSP430F46xx<MSP430FG4617, 0x2, MSP430FG46xx_ClockInfo, MemoryModel17> MSP430FG4617_Type;
typedef MSP430F46xx<MSP430FG4618, 0x1, MSP430FG46xx_ClockInfo, MemoryModel18> MSP430FG4618_Type;
typedef MSP430F46xx<MSP430FG4619, 0x0, MSP430FG46xx_ClockInfo, MemoryModel19> MSP430FG4619_Type;

typedef MSP430F46xx<MSP430F4616, 0x3, MSP430FG46xx_ClockInfo, MemoryModel16> MSP430F4616_Type;
typedef MSP430F46xx<MSP430F4617, 0x2, MSP430FG46xx_ClockInfo, MemoryModel17> MSP430F4617_Type;
typedef MSP430F46xx<MSP430F4618, 0x1, MSP430FG46xx_ClockInfo, MemoryModel18> MSP430F4618_Type;
typedef MSP430F46xx<MSP430F4619, 0x0, MSP430FG46xx_ClockInfo, MemoryModel19> MSP430F4619_Type;

typedef MSP430F46xx<MSP430F46161, 0x3, MSP430FG461x1_ClockInfo, MemoryModel16> MSP430F46161_Type;
typedef MSP430F46xx<MSP430F46171, 0x2, MSP430FG461x1_ClockInfo, MemoryModel17> MSP430F46171_Type;
typedef MSP430F46xx<MSP430F46181, 0x1, MSP430FG461x1_ClockInfo, MemoryModel18> MSP430F46181_Type;
typedef MSP430F46xx<MSP430F46191, 0x0, MSP430FG461x1_ClockInfo, MemoryModel19> MSP430F46191_Type;



static const DeviceRegistrator<MSP430FG4616_Type> regMSP430FG4616;
static const DeviceRegistrator<MSP430FG4617_Type> regMSP430FG4617;
static const DeviceRegistrator<MSP430FG4618_Type> regMSP430FG4618;
static const DeviceRegistrator<MSP430FG4619_Type> regMSP430FG4619;

static const DeviceRegistrator<MSP430F4616_Type> regMSP430F4616;
static const DeviceRegistrator<MSP430F4617_Type> regMSP430F4617;
static const DeviceRegistrator<MSP430F4618_Type> regMSP430F4618;
static const DeviceRegistrator<MSP430F4619_Type> regMSP430F4619;

static const DeviceRegistrator<MSP430F46161_Type> regMSP430F46161;
static const DeviceRegistrator<MSP430F46171_Type> regMSP430F46171;
static const DeviceRegistrator<MSP430F46181_Type> regMSP430F46181;
static const DeviceRegistrator<MSP430F46191_Type> regMSP430F46191;

