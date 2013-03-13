/*
 * MSP430C092.cpp
 *
 * Definition of MSP430C092 devices.
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

typedef IdCode<0xFFFF, 0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x0, 0xFFFFFFFF> MSP430C092IdMask;

template<const unsigned int versionId, const unsigned int activationKey>
struct MSP430C092_Match : Match<
								IdCode<versionId, 0x0, 0x00, 0x00, 0x00, 0x00, 0x0, activationKey>, 
								MSP430C092IdMask> {};

struct MSP430C0xx_Timer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430C0xx_Timer() : EemTimerImpl(
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

typedef ClockInfo<GCC_EXTENDED, 0x0417, MSP430C0xx_Timer, EmptyEemClockNames> MSP430C0xx_Clock;

typedef MemoryInfo<
	Name::main, RamType, Mapped, NotProtectable, Bits16Type, 
	Size<0x60> , Offset<0x1c00>, SegmentSize<0x1>, 
	BankSize<0x0>, Banks<1>, NoMask,MemoryCreator<LockableRamMemoryAccess>
> MSP430C0xx_MainRamMemory;


typedef MemoryInfo<
	Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
	Size<0x80> , Offset<0x2380>, SegmentSize<0x1>, 
	BankSize<0x0>, Banks<1>, NoMask
> MSP430C0xx_SystemRamMemory;


typedef MemoryInfo<
			Name::bootCode, RomType, Mapped, NotProtectable, Bits16Type, 
			Size<0x7E0> , Offset<0xF800>, SegmentSize<0x1>, BankSize<0>, Banks<1>, 
			NoMask, MemoryCreator<BootcodeRomAccess>
		> MSP430C0xx_BootCodeMemoryInfo;

extern const char vectorTableNameC092[] = "VectorTable";
typedef MemoryInfo<
			vectorTableNameC092, RomType, Mapped, NotProtectable, Bits16Type, 
			Size<0x20> , Offset<0xFFE0>, SegmentSize<0x1>, BankSize<0>, Banks<1>, 
			NoMask, MemoryCreator<BootcodeRomAccess>
		> MSP430C0xx_VectorTableMemoryInfo;

typedef VoltageInfo<900, 1800, 0, 0, 0, 0, false> MSP430C092VoltageInfo;


typedef Features<FLLPLUS, false, false, false, false, false, false> MSP430C092_Features;

template<
	const char* description,
	const unsigned int versionId
>
struct MSP430C0xx : Device<
						description, 
						ObjectId<0>,
						DefaultBits20Type, 
						regular, 
						MSP430C092_Match<versionId, 0xDEADBABE>,
						ExtraSmallEemMode,
						MSP430C092VoltageInfo,
						MSP430C0xx_Clock,
						FunctionMappingXv2,
						FuncletMappingXv2,
						MemoryList<boost::tuple<
							MSP430C0xx_MainRamMemory,
							MSP430C0xx_SystemRamMemory,
							MSP430C0xx_BootCodeMemoryInfo,
							MSP430F5xxx_peripherl16lbitMemoryInfo, 
							MSP430F5xxx_CPUMemoryInfo, 
							MSP430F5xxx_EEMMemoryInfo,
							MSP430C0xx_VectorTableMemoryInfo
						> >, //until C++0x, the space between the brackets is important
						MSP430C092_Features
					> {};

extern const char MSP430C092[] = "MSP430C092";

static const DeviceRegistrator< MSP430C0xx<MSP430C092, 0xC092> > regMSP430C092_type;
