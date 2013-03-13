/*
 * MSP430F24_61x.cpp
 *
 * Definition MSP430F24_61x devices.
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

struct MSP430F24_61x_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F24_61x_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::MCLKpin, Eem::SMCLKpin, Eem::ACLKpin, 
		Eem::ADC12, Eem::FLASH_CTRL, Eem::USCI1, Eem::USCI0,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::BT, 
		Eem::Empty, Eem::TB, Eem::TA, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	) 
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430F24_61x_EemTimer, StandardEemClockNames> MSP430F24_61x_ClockInfo;


typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07> MSP430F24_61xIdMask;

template<const unsigned int fuses >
struct MSP430F24_61x_Match : Match< IdCode<0x6FF2, 0, 0, 0, 0, 0, fuses>, MSP430F24_61xIdMask> {};

template<const unsigned int FlashSize>
struct MemoryModel4kRam : MemoryList<boost::tuple<
			MSP430F2xxx_MainFlashMemory< Size<FlashSize>, Offset<0x2100> >, 
			MSP430F2xxx_InfoFlashMemoryInfo, 
			MSP430F2xxx_BootFlashMemoryInfo, 
			MSP430F2xxx_SystemRamInfo< Size<0x800> >,
			MSP430F2xxx_SystemRam2Info< Size<0x1000> >,
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_330f1f0fffff>,
			MSP430F2xxx_CPUMemoryInfo,
			MSP430F2xxx_EEMMemoryInfo
		> > {};

template<const unsigned int FlashSize>
struct MemoryModel8kRam : MemoryList<boost::tuple<
			MSP430F2xxx_MainFlashMemory< Size<FlashSize>, Offset<0x3100> >, 
			MSP430F2xxx_InfoFlashMemoryInfo, 
			MSP430F2xxx_BootFlashMemoryInfo, 
			MSP430F2xxx_SystemRamInfo< Size<0x800> >,
			MSP430F2xxx_SystemRam2Info< Size<0x2000> >,
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_330f1f0fffff>,
			MSP430F2xxx_CPUMemoryInfo,
			MSP430F2xxx_EEMMemoryInfo
		> > {};


template<
	const char* description,
	const unsigned int fuses,
	class MemoryModelType
>
struct MSP430F24_61x : Device<
		description, 
		ObjectId<0>,
		DefaultBits20Type, 
		enhanced, 
		MSP430F24_61x_Match<fuses>,
		HighEemMode,
		MSP430F2xxx_DefaultVoltageNoTestVpp,
		MSP430F24_61x_ClockInfo,
		FunctionMappingX,
		FuncletMappingX1_2xx,
		MemoryModelType
	> {};

extern const char MSP430F2416[] = "MSP430F2416";
extern const char MSP430F2417[] = "MSP430F2417";
extern const char MSP430F2418[] = "MSP430F2418";
extern const char MSP430F2419[] = "MSP430F2419";

extern const char MSP430F2616[] = "MSP430F2616";
extern const char MSP430F2617[] = "MSP430F2617";
extern const char MSP430F2618[] = "MSP430F2618";
extern const char MSP430F2619[] = "MSP430F2619";



typedef MSP430F24_61x< MSP430F2416, 0x7, MemoryModel4kRam<0x16F00> > MSP430F2416_Type;
typedef MSP430F24_61x< MSP430F2417, 0x6, MemoryModel8kRam<0x16F00> > MSP430F2417_Type;
typedef MSP430F24_61x< MSP430F2418, 0x5, MemoryModel8kRam<0x1CF00> > MSP430F2418_Type;
typedef MSP430F24_61x< MSP430F2419, 0x4, MemoryModel4kRam<0x1DF00> > MSP430F2419_Type;

typedef MSP430F24_61x< MSP430F2616, 0x3, MemoryModel4kRam<0x16F00> > MSP430F2616_Type;
typedef MSP430F24_61x< MSP430F2617, 0x2, MemoryModel8kRam<0x16F00> > MSP430F2617_Type;
typedef MSP430F24_61x< MSP430F2618, 0x1, MemoryModel8kRam<0x1CF00> > MSP430F2618_Type;
typedef MSP430F24_61x< MSP430F2619, 0x0, MemoryModel4kRam<0x1DF00> > MSP430F2619_Type;



static const DeviceRegistrator<MSP430F2416_Type> regMSP430F2416;
static const DeviceRegistrator<MSP430F2417_Type> regMSP430F2417;
static const DeviceRegistrator<MSP430F2418_Type> regMSP430F2418;
static const DeviceRegistrator<MSP430F2419_Type> regMSP430F2419;

static const DeviceRegistrator<MSP430F2616_Type> regMSP430F2616;
static const DeviceRegistrator<MSP430F2617_Type> regMSP430F2617;
static const DeviceRegistrator<MSP430F2618_Type> regMSP430F2618;
static const DeviceRegistrator<MSP430F2619_Type> regMSP430F2619;
