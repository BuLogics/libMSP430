/*
 * MSP430L092.cpp
 *
 * Definition of MSP430L092 devices.
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
#include "LockableRamMemoryAccess.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

typedef IdCode<0xFFFF, 0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x0, 0xFFFFFFFF> MSP430L092IdMask;

template<const unsigned int versionId, const unsigned int activationKey>
struct MSP430L092_Match : Match<
								IdCode<versionId, 0x0, 0x00, 0x00, 0x00, 0x00, 0x0, activationKey>, 
								MSP430L092IdMask> {};

struct MSP430L0xx_Timer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430L0xx_Timer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::CCS, Eem::APOOL,
		Eem::Empty, Eem::TA3_0, Eem::TA3_1, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x0417, MSP430L0xx_Timer, EmptyEemClockNames> MSP430L0xx_Clock;

typedef MemoryInfo<
	Name::main, RamType, Mapped, NotProtectable, Bits16Type, 
	Size<0x780> , Offset<0x1c00>, SegmentSize<0x1>, 
	BankSize<0x0>, Banks<1>, NoMask,MemoryCreator<LockableRamMemoryAccess>
> MSP430L0xx_MainRamMemory;


typedef MemoryInfo<
	Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
	Size<0x80> , Offset<0x2380>, SegmentSize<0x1>, 
	BankSize<0x0>, Banks<1>, NoMask
> MSP430L0xx_SystemRamMemory;


typedef MemoryInfo<
			Name::bootCode, RomType, Mapped, NotProtectable, Bits16Type, 
			Size<0x7E0> , Offset<0xF800>, SegmentSize<0x1>, BankSize<0>, Banks<1>, 
			NoMask, MemoryCreator<BootcodeRomAccess>
		> MSP430L0xx_BootCodeMemoryInfo;

extern const char vectorTableName[] = "VectorTable";
typedef MemoryInfo<
			vectorTableName, RamType, Mapped, NotProtectable, Bits16Type, 
			Size<0x20> , Offset<0xFFE0>, SegmentSize<0x1>, BankSize<0>, Banks<1>, 
			NoMask, MemoryCreator<BootcodeRomAccess>
		> MSP430L0xx_VectorTableMemoryInfo;


typedef VoltageInfo<900, 1800, 0, 0, 0, 0, false> MSP430L092VoltageInfo;


typedef Features<FLLPLUS, false, false, true, false, false, false> MSP430L092_Features;

template<
	const char* description,
	const unsigned int versionId
>
struct MSP430L0xx : Device<
						description, 
						ObjectId<0>,
						DefaultBits20Type, 
						regular, 
						MSP430L092_Match<versionId, 0xA55AA55A>,
						ExtraSmallEemMode,
						MSP430L092VoltageInfo,
						MSP430L0xx_Clock,
						FunctionMappingXv2,
						FuncletMappingXv2,
						MemoryList<boost::tuple<
							MSP430L0xx_MainRamMemory,
							MSP430L0xx_SystemRamMemory,
							MSP430L0xx_BootCodeMemoryInfo,
							MSP430F5xxx_peripherl16lbitMemoryInfo, 
							MSP430F5xxx_CPUMemoryInfo, 
							MSP430F5xxx_EEMMemoryInfo,
							MSP430L0xx_VectorTableMemoryInfo
						> >, //until C++0x, the space between the brackets is important
						MSP430L092_Features
					> {};

extern const char MSP430L092[] = "MSP430L092";

static const DeviceRegistrator< MSP430L0xx<MSP430L092, 0xC092> > regMSP430L092_type;
