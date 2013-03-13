/*
 * MSP430F43xx.cpp
 *
 * Definition MSP430F43xx devices.
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

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F> MSP430F43xxp100IdMask;
typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00> MSP430F43xxp80IdMask;
typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00> MSP430FG43xxIdMask;

typedef Match< IdCode<0x49F4, 0,0,0,0,0, 0x02>, MSP430F43xxp100IdMask> F43xxp100_Match;
typedef Match< IdCode<0x37F4, 0,0,0,0,0,0>, MSP430F43xxp80IdMask> F43xxp80_Match;
typedef Match< IdCode<0x39F4, 0,0,0,0,'G',0>, MSP430FG43xxIdMask> FG43xx_Match;


struct MSP430F43xx_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F43xx_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::MCLKpin, Eem::SMCLKpin, Eem::ACLKpin, 
		Eem::ADC12, Eem::FLASH_CTRL, Eem::Empty, Eem::USART0,
		Eem::Empty, Eem::Empty, Eem::LCD_FREQ, Eem::BT, 
		Eem::Empty, Eem::TB, Eem::TA, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};


typedef ClockInfo<GCC_EXTENDED, 0x60D7, MSP430F43xx_EemTimer, StandardEemClockNames> MSP430F43xx_ClockInfo;
		

typedef Trigger<EMEX_LOW, 0x2, 0, 0x2, 0, 0x1, 0, 0, 0, 0, 0x01> MSP430F43xx_LowTrigger;
typedef EemInfo<0, 0, 0, MSP430F43xx_LowTrigger, MediumSequencer> FG43xx_EemMode;

typedef Trigger<EMEX_HIGH, 0x8, 0x2, 0x8, 0x1, 0, 0x1, 0x1, 0x02, 0x00, 0x01> MSP430F43xx_HighTrigger;
typedef EemInfo<0x8, 0, 0, MSP430F43xx_HighTrigger, MediumSequencer> F43xx_EemMode;



template<class FlashOffset, class FlashSize, class RamSize, class LcdMemorySize>
struct MemoryModel : MemoryList<boost::tuple<
			MSP430F4xxx_MainFlashMemory<FlashSize, FlashOffset>, 
			MSP430F4xxx_InfoFlashMemoryInfo, 
			MSP430F4xxx_BootFlashMemoryInfo, 
			MSP430F4xxx_LcdMemoryInfo<LcdMemorySize>,
			MSP430F4xxx_SystemRamInfo<RamSize>, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_f380d380c0ff>,
			MSP430F4xxx_CPUMemoryInfo,
			MSP430F4xxx_EEMMemoryInfo> 
		> {};


typedef MemoryModel< Offset<0x8000>, Size<0x8000>, Size<0x400>, Size<0x15> > MemoryModel7;
typedef MemoryModel< Offset<0x1100>, Size<0xEF00>, Size<0x800>, Size<0xD> > MemoryModelG9;


template<
	const char* description,
	class MatchType,
	class EemModeType,
	class MemoryModelType,
	class ExtendedFeatures = MSP430F4xxx_SyncExtFeatures
>
struct MSP430F43xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MatchType, 
		EemModeType,
		MSP430F4xxx_DefaultVoltageNoTestVpp,
		MSP430F43xx_ClockInfo,
		FunctionMapping4xx,
		FuncletMapping4xx,
		MemoryModelType,
		MSP430F43xx_Features,
		ExtendedFeatures
	> {};


extern const char MSP430F437[] = "MSP430F43x";
extern const char MSP430FG439[] = "MSP430FG43x_F43x";


static const DeviceRegistrator< MSP430F43xx<MSP430F437, F43xxp80_Match, F43xx_EemMode, MemoryModel7> > regMSP430F437p80;

static const DeviceRegistrator< MSP430F43xx<MSP430F437, F43xxp100_Match, F43xx_EemMode, MemoryModel7> > regMSP430F437p100;

static const DeviceRegistrator< 
		MSP430F43xx<MSP430FG439, FG43xx_Match, FG43xx_EemMode, MemoryModelG9, NoExtendedFeatures> 
> regMSP430FG439;
