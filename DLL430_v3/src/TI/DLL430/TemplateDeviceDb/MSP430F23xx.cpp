/*
 * MSP430F23xx.cpp
 *
 * Definition MSP430F23xx devices.
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


struct MSP430F23x_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F23x_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::MCLKpin, Eem::SMCLKpin, Eem::ACLKpin, 
		Eem::ADC12, Eem::Empty, Eem::Empty, Eem::USCI0,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::TB, Eem::TA, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_STANDARD, 0x60D7, EmptyEemTimer, TAClkEemClockNames> MSP430F23x0_ClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430F23x_EemTimer, StandardEemClockNames> MSP430F23x_ClockInfo;

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03> MSP430F23x0IdMask;
typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F> MSP430F23xIdMask;

template<const unsigned int versionId, const unsigned int fuses >
struct MSP430F23x0_Match : Match< IdCode<versionId, 0, 0, 0, 0, 0, fuses>, MSP430F23x0IdMask> {};

template<const unsigned int versionId, const unsigned int fuses >
struct MSP430F23x_Match : Match< IdCode<versionId, 0, 0, 0, 0, 0, fuses>, MSP430F23xIdMask> {};


template<class FlashOffset, class FlashSize, class RamSize>
struct MemoryModel : MemoryList<boost::tuple<
			MSP430F2xxx_MainFlashMemory<FlashSize, FlashOffset>, 
			MSP430F2xxx_InfoFlashMemoryInfo, 
			MSP430F2xxx_BootFlashMemoryInfo, 
			MSP430F2xxx_SystemRamInfo< RamSize >, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_330f1f0fffff>,
			MSP430F2xxx_CPUMemoryInfo,
			MSP430F2xxx_EEMMemoryInfo
		> > {};

typedef MemoryModel< Offset<0xE000>, Size<0x2000>, Size<0x400> > MemoryModel3;
typedef MemoryModel< Offset<0xC000>, Size<0x4000>, Size<0x800> > MemoryModel5;
typedef MemoryModel< Offset<0x8000>, Size<0x8000>, Size<0x800> > MemoryModel7;

template<
	const char* description,
	class MatchType,
	class EemModeType,
	class ClockInfoType,
	class MemoryModelType
>
struct MSP430F23xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		enhanced, 
		MatchType,
		EemModeType,
		MSP430F2xxx_DefaultVoltageNoTestVpp,
		ClockInfoType,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryModelType
	> {};

extern const char MSP430F233[] = "MSP430F233";
extern const char MSP430F235[] = "MSP430F235";

extern const char MSP430F2330[] = "MSP430F2330";
extern const char MSP430F2350[] = "MSP430F2350";
extern const char MSP430F2370[] = "MSP430F2370";

typedef MSP430F23xx<MSP430F233, MSP430F23x_Match<0x49F2, 0x7>, MedEemMode, MSP430F23x_ClockInfo, MemoryModel3> MSP430F233_Type;
typedef MSP430F23xx<MSP430F235, MSP430F23x_Match<0x49F2, 0x3>, MedEemMode, MSP430F23x_ClockInfo, MemoryModel5> MSP430F235_Type;

typedef MSP430F23xx<MSP430F2330, MSP430F23x0_Match<0x37F2, 0x2>, LowEemMode, MSP430F23x0_ClockInfo, MemoryModel3> MSP430F2330_Type;
typedef MSP430F23xx<MSP430F2350, MSP430F23x0_Match<0x37F2, 0x1>, LowEemMode, MSP430F23x0_ClockInfo, MemoryModel5> MSP430F2350_Type;
typedef MSP430F23xx<MSP430F2370, MSP430F23x0_Match<0x37F2, 0x0>, LowEemMode, MSP430F23x0_ClockInfo, MemoryModel7> MSP430F2370_Type;

static const DeviceRegistrator<MSP430F233_Type> regMSP430F233;
static const DeviceRegistrator<MSP430F235_Type> regMSP430F235;
static const DeviceRegistrator<MSP430F2330_Type> regMSP430F2330;
static const DeviceRegistrator<MSP430F2350_Type> regMSP430F2350;
static const DeviceRegistrator<MSP430F2370_Type> regMSP430F2370;
