/*
 * MSP430FR59xx.cpp
 *
 * Definition MSP430FR59xx.
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
#include "MpuFr5969.h"
#include "FramMemoryAccessBase.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;



struct MSP430FR59xx_EemTimerLarge : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FR59xx_EemTimerLarge() : EemTimerImpl(
		Eem::TA2_1, Eem::Empty, Eem::AES, Eem::COMP_D, 
		Eem::ADC12_B, Eem::RTC, Eem::Empty, Eem::eUSCIB0,
		Eem::eUSCIA1, Eem::eUSCIA0, Eem::TB7_0, Eem::TA3_0,
		Eem::TA3_1, Eem::TA3_2, Eem::TA2_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0xBDFF, MSP430FR59xx_EemTimerLarge, EmptyEemClockNames> FR59xx_SmallClockInfo;	

typedef Trigger<EMEX_SMALL_5XX, 0x03, 0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00> FR59xxTrigger;

typedef EemInfo<0x00, 0x01, 0x01, FR59xxTrigger, SmallSequencer> FR59xxEemMode;


/* Need datasheet or other source */

//protectable due to INFO A
typedef MemoryInfo<
	Name::information, RamType, Mapped, NotProtectable, Bits16Type, 
	Size<0x200> , Offset<0x1800>, SegmentSize<0x01>, BankSize<0x0>, Banks<1>,
	NoMask, MemoryCreator<FramMemoryAccessBase<MpuFr5969> >
> MSP430FR59xx_InfoFramMemoryInfo;


///bsl memory
typedef MemoryInfo<
	Name::boot, RomType, Mapped, NotProtectable, Bits16Type, 
	Size<0x800> , Offset<0x1000>, SegmentSize<0x01>, BankSize<0>, Banks<1>, 
	NoMask, MemoryCreator<ReadonlyMemoryAccess>
> MSP430FR59xx_BootFramMemoryInfo;

typedef Features<MOD_OSC, false, true, true, false, true, false> MSP430FR59xx_Features;

struct FunctionMappingMSP430FR59xx : public FunctionMappingBase
{
	FunctionMappingMSP430FR59xx(){
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
			(ID_ExecuteFunclet,ID_ExecuteFuncletXv2)
			(ID_WaitForStorage,ID_WaitForStorageX);
		}
};

struct FunctionMappingMSP430FR69xx : public FunctionMappingBase
{
	FunctionMappingMSP430FR69xx(){
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
			(ID_SingleStep,ID_SingleStepJStateXv2) 
			(ID_ReadAllCpuRegs,ID_ReadAllCpuRegsXv2)
			(ID_WriteAllCpuRegs,ID_WriteAllCpuRegsXv2)
			(ID_Psa,ID_PsaXv2)
			(ID_ExecuteFunclet,ID_ExecuteFuncletXv2)
			(ID_WaitForEem,ID_PollJStateReg)
			(ID_WriteFramQuickXv2,ID_WriteMemWordsXv2) // Bug in Havok device
			(ID_WaitForStorage,ID_WaitForStorageX);
		}
};

template<class SizeType>
struct MSP430FR6xxx_SystemRam2Info : MemoryInfo<
				Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
				SizeType , Offset<0x3C00>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>, 
				NoMask> {};

template<class SizeType, class OffsetType>
struct MSP430FR59xx_MainFramMemory : MemoryInfo<
										Name::main, RamType, Mapped, NotProtectable, Bits16Type, 
										SizeType , OffsetType, SegmentSize<0x01>, 
										BankSize<0x0>, Banks<1>, NoMask, MemoryCreator<FramMemoryAccessBase<MpuFr5969> >
									> {};
template<
	const char* description,
	const unsigned int versionId,
	class ClockInfoType,
	const unsigned int FramOffset,
	const unsigned int FramSize,
	const unsigned int SysRamSize
>
struct MSP430FR59xx_type : Device<
							description, 
							ObjectId<0>,
							DefaultBits20Type, 
							regular, 
							MSP430F5xxx_Match<versionId>, 
							FR59xxEemMode, 
							MSP430F5xxx_DefaultVoltageTestVpp,
							ClockInfoType,
							FunctionMappingMSP430FR59xx,
							FuncletMappingXv2FRAM,
							MemoryList<boost::tuple<
								MSP430FR59xx_MainFramMemory< Size<FramSize>, Offset<FramOffset> >,
								MSP430FR59xx_InfoFramMemoryInfo, 
								MSP430FR59xx_BootFramMemoryInfo,
								MSP430F5xxx_BootCodeMemoryInfo,
								MSP430F5xxx_SystemRamInfo< Size<SysRamSize> >, /*MSP430F5xxx_SystemRamInfo< Size<0x800> >,*/
								MSP430F5xxx_peripherl16lbitMemoryInfo, 
								MSP430F5xxx_CPUMemoryInfo, 
								MSP430F5xxx_EEMMemoryInfo 
//								MSP430F5xxx_extendedMemoryInfo
							> >, //until C++0x, the space between the brackets is important
							MSP430FR59xx_Features,
							NoExtendedFeatures,
							PowerSettings<0x00010018, // Test reg mask
									      0x00010018, // Test reg value to enable LPMx.5 
										  0x00010000, // Test reg value to disable LPMx.5 
										  0x4020,	  // 3V Test reg mask 
										  0x4020,	  // 3V Test reg value to enable LPMx.5 
										  0x4020>	  // 3V Test reg value to disable LPMx.5 
						> {};

template<
	const char* description,
	const unsigned int versionId,
	class ClockInfoType,
	const unsigned int FramOffset,
	const unsigned int FramSize,
	const unsigned int SysRamSize,
	const unsigned int SysRam2Size
>
struct MSP430FR69xx_type : Device<
							description, 
							ObjectId<0>,
							DefaultBits20Type, 
							regular, 
							MSP430F5xxx_Match<versionId>, 
							FR59xxEemMode, 
							MSP430F5xxx_DefaultVoltageTestVpp,
							ClockInfoType,
							FunctionMappingMSP430FR69xx,
							FuncletMappingXv2FRAM,
							MemoryList<boost::tuple<
								MSP430FR59xx_MainFramMemory< Size<FramSize>, Offset<FramOffset> >,
								MSP430FR59xx_InfoFramMemoryInfo, 
								MSP430FR59xx_BootFramMemoryInfo,
								MSP430F5xxx_BootCodeMemoryInfo,
								MSP430FR6xxx_SystemRam2Info< Size<SysRam2Size> >,
								MSP430F5xxx_SystemRamInfo< Size<SysRamSize> >, /*MSP430F5xxx_SystemRamInfo< Size<0x800> >,*/
								MSP430F5xxx_peripherl16lbitMemoryInfo, 
								MSP430F5xxx_CPUMemoryInfo, 
								MSP430F5xxx_EEMMemoryInfo 
//								MSP430F5xxx_extendedMemoryInfo
							> >, //until C++0x, the space between the brackets is important
							MSP430FR59xx_Features,
							NoExtendedFeatures,
							PowerSettings<0x00010018, // Test reg mask
									      0x00010018, // Test reg value to enable LPMx.5 
										  0x00010000, // Test reg value to disable LPMx.5 
										  0xC020,	  // 3V Test reg mask 
										  0xC020,	  // 3V Test reg value to enable LPMx.5 
										  0x4020>	  // 3V Test reg value to disable LPMx.5 
						> {};

extern const char MSP430FR5969[] = "MSP430FR5969";
extern const char MSP430FR5949[] = "MSP430FR5949";
extern const char MSP430FR6989[] = "MSP430FR6989";

static const DeviceRegistrator<
	MSP430FR59xx_type<MSP430FR5969, 0x8169, FR59xx_SmallClockInfo, 0x4400, 0xFC00, 0x800> 
> regMSP430FR5969;

static const DeviceRegistrator<
	MSP430FR59xx_type<MSP430FR5949 , 0x8161, FR59xx_SmallClockInfo, 0x4400, 0xFC00, 0x800> 
> regMSP430FR5949 ;

static const DeviceRegistrator<
	MSP430FR69xx_type<MSP430FR6989 , 0x81A8 , FR59xx_SmallClockInfo, 0x4400, 0x1FC00, 0x800, 0x80> 
> regMSP430FR6989 ;


