/*
 * MSP430Defaults.h
 *
 * Default values to be used when devices are created, e.g. within MSP43054xx.cpp.
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

#ifndef TEMPLATE_DEVICE_DB_MSP430DEFAULTS_H
#define TEMPLATE_DEVICE_DB_MSP430DEFAULTS_H

#if _MSC_VER > 1000
#pragma once
#endif

#include "Registration.h"

namespace TI { namespace DLL430 { namespace TemplateDeviceDb {

struct FunctionMappingBasic;
struct FunctionMappingX;
struct FunctionMappingXv2;

enum EMEX_MODE
{
	/// Device has no Emex module.
	EMEX_NONE =0,
	/// Device Emex module has two breakpoints.
	EMEX_LOW = 1,
	/// Device Emex module has three breakpoints and range comparison.
	EMEX_MEDIUM = 2,
	/// Device Emex module has eight breakpoints, range comparison, state storage, and trigger sequencer,
	EMEX_HIGH = 3,
    EMEX_EXTRA_SMALL_5XX = 4,
    EMEX_SMALL_5XX = 5,
    EMEX_MEDIUM_5XX =6,
    EMEX_LARGE_5XX =7
};

typedef Trigger<EMEX_NONE, 0x2, 0, 0x2, 0, 0, 0, 0, 0, 0, 0> NoneTrigger;
typedef Trigger<EMEX_LOW, 0x2, 0, 0x2, 0, 0, 0, 0, 0, 0, 0> LowTrigger;
typedef Trigger<EMEX_MEDIUM, 0x3, 0, 0x3, 0, 0, 0, 0, 0, 0, 0> MedTrigger;
typedef Trigger<EMEX_HIGH, 0x8, 0x2, 0x8, 0x1, 0x1, 0x1, 0x1, 0x02, 0x00, 0x01> HighTrigger;

typedef Trigger<EMEX_EXTRA_SMALL_5XX, 0x02, 0x00, 0x02, 0x01, 0x01, 0x01, 0x00, 0x02, 0x00, 0x00> ExtraSmallTrigger;
typedef Trigger<EMEX_SMALL_5XX, 0x03, 0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00> SmallTrigger;
typedef Trigger<EMEX_MEDIUM_5XX, 0x05, 0x01, 0x06, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00> MediumTrigger;
typedef Trigger<EMEX_LARGE_5XX, 0x08, 0x02, 0x0a, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x01> LargeTrigger;

typedef Sequencer<0x00, 0x00, 0x00, 0x00, 0x00> SmallSequencer;
typedef Sequencer<0x01, 0x02, 0x00, 0x00, 0x00> MediumSequencer;
typedef Sequencer<0x03, 0x04, 0x00, 0x00, 0x00> LargeSequencer;

typedef EemInfo<0, 0, 0, NoneTrigger, SmallSequencer> NoneEemMode;
typedef EemInfo<0, 0, 0, LowTrigger, SmallSequencer> LowEemMode;
typedef EemInfo<0, 0, 0, MedTrigger, SmallSequencer> MedEemMode;
typedef EemInfo<0x8, 0, 0, HighTrigger, MediumSequencer> HighEemMode;

typedef EemInfo<0x00, 0x01, 0x00, ExtraSmallTrigger, SmallSequencer> ExtraSmallEemMode;
typedef EemInfo<0x00, 0x01, 0x00, SmallTrigger, SmallSequencer> SmallEemMode;
typedef EemInfo<0x00, 0x01, 0x00, MediumTrigger, MediumSequencer> MediumEemMode;
typedef EemInfo<0x08, 0x02, 0x01, LargeTrigger, LargeSequencer> LargeEemMode;

namespace Memory {
typedef MemoryInfo<
		Name::peripheral16bit, RegisterType, Mapped, NotProtectable, Bits16Type, 
		Size<0x100> , Offset<0x0100>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>, 
		NoMask
	> MSP430F1_2_4xxx_peripherl16lbitMemoryInfo;

template<class Mask>
struct MSP430F1_2_4xxx_peripherl8lbitMemoryInfo : MemoryInfo<
		Name::peripheral8bit, RegisterType, Mapped, NotProtectable, Bits8Type, 
		Size<0x100> , Offset<0x0>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>, Mask> {};
} //namespace Memory 

struct FunctionMappingBase : public FunctionMappingImpl
{	
};

struct FunctionMappingNone : public FunctionMappingBase
{
};

struct FunctionMappingX : public FunctionMappingBase
{
	FunctionMappingX(){
		FunctionMappingImpl::fcntMap_ = boost::assign::map_list_of
			(ID_SyncJtag_AssertPor_SaveContext, ID_SyncJtag_AssertPor_SaveContextX)
			(ID_SyncJtag_Conditional_SaveContext,ID_SyncJtag_Conditional_SaveContextX)
			(ID_RestoreContext_ReleaseJtag,ID_RestoreContext_ReleaseJtagX)
			(ID_ReadMemWords,ID_ReadMemWordsX)
			(ID_ReadMemQuick,ID_ReadMemQuickX)
			(ID_WriteMemWords,ID_WriteMemWordsX)
			(ID_EemDataExchange,ID_EemDataExchangeX)
			(ID_SingleStep,ID_SingleStepX)
			(ID_ReadAllCpuRegs,ID_ReadAllCpuRegsX)
			(ID_WriteAllCpuRegs,ID_WriteAllCpuRegsX)
			(ID_Psa,ID_PsaX)
			(ID_ExecuteFunclet,ID_ExecuteFuncletX)
			(ID_GetDcoFrequency,ID_GetDcoFrequencyX)
			(ID_WaitForStorage,ID_WaitForStorageX);
		}
};

struct FunctionMappingXv2 : public FunctionMappingBase
{
	FunctionMappingXv2(){
		FunctionMappingImpl::fcntMap_ = boost::assign::map_list_of
			(ID_BlowFuse, ID_BlowFuseXv2)
			(ID_SyncJtag_AssertPor_SaveContext, ID_SyncJtag_AssertPor_SaveContextXv2)
			(ID_SyncJtag_Conditional_SaveContext,ID_SyncJtag_Conditional_SaveContextXv2)
			(ID_RestoreContext_ReleaseJtag,ID_RestoreContext_ReleaseJtagXv2)
			(ID_ReadMemBytes,ID_ReadMemWordsXv2)
			(ID_ReadMemWords,ID_ReadMemWordsXv2)
			(ID_ReadMemQuick,ID_ReadMemQuickXv2)
			(ID_WriteMemBytes,ID_WriteMemWordsXv2)
			(ID_WriteMemWords,ID_WriteMemWordsXv2)
			(ID_EemDataExchange,ID_EemDataExchangeXv2)
			(ID_SingleStep,ID_SingleStepXv2)
			(ID_ReadAllCpuRegs,ID_ReadAllCpuRegsXv2)
			(ID_WriteAllCpuRegs,ID_WriteAllCpuRegsXv2)
			(ID_Psa,ID_PsaXv2)
			(ID_PollJStateReg,ID_PollJStateRegFR57xx)
			(ID_ExecuteFunclet,ID_ExecuteFuncletXv2)
			(ID_WaitForStorage,ID_WaitForStorageX);
		}
};


struct FunctionMappingXv2FRAM : public FunctionMappingBase
{
	FunctionMappingXv2FRAM(){
		FunctionMappingImpl::fcntMap_ = boost::assign::map_list_of
			(ID_BlowFuse, ID_BlowFuseFram)
			(ID_SyncJtag_AssertPor_SaveContext, ID_SyncJtag_AssertPor_SaveContextXv2)
			(ID_SyncJtag_Conditional_SaveContext,ID_SyncJtag_Conditional_SaveContextXv2)
			(ID_RestoreContext_ReleaseJtag,ID_RestoreContext_ReleaseJtagXv2)
			(ID_ReadMemBytes,ID_ReadMemWordsXv2)
			(ID_ReadMemWords,ID_ReadMemWordsXv2)
			(ID_ReadMemQuick,ID_ReadMemQuickXv2)
			(ID_WriteMemBytes,ID_WriteMemWordsXv2)
			(ID_WriteMemWords,ID_WriteMemWordsXv2)
			(ID_EemDataExchange,ID_EemDataExchangeXv2)
			(ID_SingleStep,ID_SingleStepXv2)
			(ID_ReadAllCpuRegs,ID_ReadAllCpuRegsXv2)
			(ID_WriteAllCpuRegs,ID_WriteAllCpuRegsXv2)
			(ID_Psa,ID_PsaXv2)
			(ID_PollJStateReg,ID_PollJStateRegFR57xx)
			(ID_ExecuteFunclet,ID_ExecuteFuncletXv2)
			(ID_WaitForStorage,ID_WaitForStorageX);
		}
};


struct FuncletMappingNone : FuncletMappingImpl 
{
	FuncletMappingNone() 
		: FuncletMappingImpl(FuncletCode(), FuncletCode()) {}
};


struct FuncletMapping1_2xx : FuncletMappingImpl
{
	FuncletMapping1_2xx() 
		: FuncletMappingImpl(
			FuncletCode( eraseFuncletCodeDCO, sizeof(eraseFuncletCodeDCO), 4 ),
			FuncletCode( writeFuncletCodeDCO, sizeof(writeFuncletCodeDCO), 128 ) ) 
	{}
};


struct FuncletMappingX1_2xx : FuncletMappingImpl
{
	FuncletMappingX1_2xx() 
		: FuncletMappingImpl(
			FuncletCode( eraseFuncletCodeXDCO, sizeof(eraseFuncletCodeXDCO), 4 ),
			FuncletCode( writeFuncletCodeXDCO, sizeof(writeFuncletCodeXDCO), 256 ) ) 
	{}
};


struct FuncletMappingXv2 : FuncletMappingImpl
{
	FuncletMappingXv2() 
		: FuncletMappingImpl(
			FuncletCode( eraseFuncletCodeXv2, sizeof(eraseFuncletCodeXv2) ),
			FuncletCode( writeFuncletCodeXv2, sizeof(writeFuncletCodeXv2) ),
			FuncletCode( UnlockBslFuncletCodeXv2, sizeof(UnlockBslFuncletCodeXv2))
		)
	{}
};

struct FuncletMappingXv2WordMode : FuncletMappingImpl
{
	FuncletMappingXv2WordMode() 
		: FuncletMappingImpl(
			FuncletCode( eraseFuncletCodeXv2, sizeof(eraseFuncletCodeXv2) ),
			FuncletCode( writeFuncletCodeXv2WordMode, sizeof(writeFuncletCodeXv2WordMode) ),
			FuncletCode( UnlockBslFuncletCodeXv2, sizeof(UnlockBslFuncletCodeXv2))
		)
	{}
};



struct FuncletMappingXv2FRAM : FuncletMappingImpl
{
	FuncletMappingXv2FRAM() 
		: FuncletMappingImpl(
			FuncletCode( eraseFuncletCodeXv2FRAM, sizeof(eraseFuncletCodeXv2FRAM) ),
			FuncletCode( writeFuncletCodeXv2FRAM, sizeof(writeFuncletCodeXv2FRAM) ) ) 
	{}
};


/// One of the following enumerations is returned in device.clockControl
enum DEVICE_CLOCK_CONTROL
{
	/// Device has no clock control. The system clock continue to function when the device is stopped by JTAG.
	GCC_NONE,
	/// Device has General Clock Control register.
	GCC_STANDARD,
	/// Device has Extended General Clock Control register and Module Clock Control register 0.
	GCC_EXTENDED,
};

struct EmptyEemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	EmptyEemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};


struct EmptyEemClockNames : EemClocksImpl
{
	typedef EemClocksImpl::Clocks Clock;
	EmptyEemClockNames() : EemClocksImpl(
		Clock::Empty, Clock::Empty, Clock::Empty, Clock::Empty, 
		Clock::Empty, Clock::Empty, Clock::Empty, Clock::Empty,
		Clock::Empty, Clock::Empty, Clock::Empty, Clock::Empty,
		Clock::Empty, Clock::Empty, Clock::Empty, Clock::Empty
	)
	{}
};

struct StandardEemClockNames : EemClocksImpl
{
	typedef EemClocksImpl::Clocks Clock;
	StandardEemClockNames() : EemClocksImpl(
		Clock::Empty, Clock::Empty, Clock::Empty, Clock::Empty, 
		Clock::Empty, Clock::Empty, Clock::Empty, Clock::Empty,
		Clock::Empty, Clock::Empty, Clock::Empty, Clock::Empty,
		Clock::Empty, Clock::SMCLK, Clock::ACLK,  Clock::Empty
	)
	{}
};

struct TAClkEemClockNames : EemClocksImpl
{
	typedef EemClocksImpl::Clocks Clock;
	TAClkEemClockNames() : EemClocksImpl(
		Clock::Empty, Clock::Empty, Clock::Empty, Clock::Empty, 
		Clock::Empty, Clock::Empty, Clock::Empty, Clock::Empty,
		Clock::Empty, Clock::Empty, Clock::TACLK, Clock::Empty,
		Clock::Empty, Clock::SMCLK, Clock::ACLK,  Clock::Empty
	)
	{}
};

///You can also overwrite parts of a function mapping. See this example code:
/*struct FunctionMappingTest : public FunctionMappingXv2
{
	FunctionMappingTest() :  FunctionMappingXv2() {
		ReplacePair(ID_Psa,ID_PsaMy);
	}
};*/

} //namespace TemplateDeviceDb
}//namespace DLL430
}//namespace TI
#endif //TEMPLATE_DEVICE_DB_MSP430DEFAULTS_H

