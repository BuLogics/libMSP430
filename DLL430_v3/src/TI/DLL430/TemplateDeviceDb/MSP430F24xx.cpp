/*
 * MSP430F24xx.cpp
 *
 * Definition MSP430F24xx devices.
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

#include "MSP430F2xxx.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

struct MSP430F24x_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F24x_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::MCLKpin, Eem::SMCLKpin, Eem::ACLKpin, 
		Eem::ADC12, Eem::Empty, Eem::USCI1, Eem::USCI0,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::TB, Eem::TA, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct MSP430F24x1_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F24x1_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::MCLKpin, Eem::SMCLKpin, Eem::ACLKpin, 
		Eem::Empty, Eem::Empty, Eem::USCI1, Eem::USCI0,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::TB, Eem::TA, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430F24x_EemTimer, StandardEemClockNames> MSP430F24x_ClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430F24x1_EemTimer, StandardEemClockNames> MSP430F24x1_ClockInfo;


typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F> MSP430F24xxIdMask;

template<const unsigned int fuses >
struct MSP430F24xx_Match : Match< IdCode<0x49F2, 0, 0, 0, 0, 0, fuses>, MSP430F24xxIdMask> {};


struct MemoryModel2kRam : MemoryList<boost::tuple<
			MSP430F2xxx_MainFlashMemory< Size<0xEF00>, Offset<0x1100> >, 
			MSP430F2xxx_InfoFlashMemoryInfo, 
			MSP430F2xxx_BootFlashMemoryInfo, 
			MSP430F2xxx_SystemRamInfo< Size<0x800> >, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_330f1f0fffff>,
			MSP430F2xxx_CPUMemoryInfo,
			MSP430F2xxx_EEMMemoryInfo
		> > {};

template<const unsigned int FlashOffset, const unsigned int FlashSize>
struct MemoryModel4kRam : MemoryList<boost::tuple<
			MSP430F2xxx_MainFlashMemory< Size<FlashSize>, Offset<FlashOffset> >, 
			MSP430F2xxx_InfoFlashMemoryInfo, 
			MSP430F2xxx_BootFlashMemoryInfo, 
			MSP430F2xxx_SystemRamInfo< Size<0x800> >,
			MSP430F2xxx_SystemRam2Info< Size<0x1000> >,
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_330f1f0fffff>,
			MSP430F2xxx_CPUMemoryInfo,
			MSP430F2xxx_EEMMemoryInfo
		> > {};


template<
	const char* description,
	const unsigned int fuses,
	class ClockInfoType,
	class MemoryModelType
>
struct MSP430F24xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		enhanced, 
		MSP430F24xx_Match<fuses>,
		MedEemMode,
		MSP430F2xxx_DefaultVoltageNoTestVpp,
		ClockInfoType,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryModelType
	> {};

extern const char MSP430F247[] = "MSP430F247";
extern const char MSP430F248[] = "MSP430F248";
extern const char MSP430F249[] = "MSP430F249";
extern const char MSP430F2410[] = "MSP430F2410";

extern const char MSP430F2471[] = "MSP430F2471";
extern const char MSP430F2481[] = "MSP430F2481";
extern const char MSP430F2491[] = "MSP430F2491";

typedef MSP430F24xx< MSP430F247, 0x2, MSP430F24x_ClockInfo, MemoryModel4kRam<0x8000, 0x8000> > MSP430F247_Type;
typedef MSP430F24xx< MSP430F248, 0x1, MSP430F24x_ClockInfo, MemoryModel4kRam<0x4000, 0xC000> > MSP430F248_Type;
typedef MSP430F24xx< MSP430F249, 0x0, MSP430F24x_ClockInfo, MemoryModel2kRam > MSP430F249_Type;
typedef MSP430F24xx< MSP430F2410, 0x8, MSP430F24x_ClockInfo, MemoryModel4kRam<0x2100, 0xDF00> > MSP430F2410_Type;

typedef MSP430F24xx< MSP430F2471, 0x6, MSP430F24x1_ClockInfo, MemoryModel4kRam<0x8000, 0x8000> > MSP430F2471_Type;
typedef MSP430F24xx< MSP430F2481, 0x5, MSP430F24x1_ClockInfo, MemoryModel4kRam<0x4000, 0xC000> > MSP430F2481_Type;
typedef MSP430F24xx< MSP430F2491, 0x4, MSP430F24x1_ClockInfo, MemoryModel2kRam > MSP430F2491_Type;

static const DeviceRegistrator<MSP430F247_Type> regMSP430F247;
static const DeviceRegistrator<MSP430F248_Type> regMSP430F248;
static const DeviceRegistrator<MSP430F249_Type> regMSP430F249;
static const DeviceRegistrator<MSP430F2410_Type> regMSP430F2410;

static const DeviceRegistrator<MSP430F2471_Type> regMSP430F2471;
static const DeviceRegistrator<MSP430F2481_Type> regMSP430F2481;
static const DeviceRegistrator<MSP430F2491_Type> regMSP430F2491;
