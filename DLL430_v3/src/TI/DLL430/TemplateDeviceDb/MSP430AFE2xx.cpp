/*
 * MSP430AFE2xx.cpp
 *
 * Definition MSP430AFE2xx devices.
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

typedef ClockInfo<GCC_STANDARD, 0x60D7, EmptyEemTimer, TAClkEemClockNames> MSP430AFE2xx_ClockInfo;

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF, 0x0> MSP430AFE2xxIdMask;

template<const uint8_t fuses >
struct MSP430AFE2xx_Match : Match< IdCode<0x5302, 0x0, 0, 0, 0, 0, fuses, 0>, MSP430AFE2xxIdMask> {};

template<class FlashOffset, class FlashSize, class RamSize>
struct MemoryModel : MemoryList<boost::tuple<
			MSP430F2xxx_MainFlashMemory<FlashSize, FlashOffset>, 
			MSP430F2xxx_InfoFlashMemoryInfo, 
			MSP430F2xxx_SystemRamInfo< RamSize >, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_f3ffdfffc0ff>,
			MSP430F2xxx_CPUMemoryInfo,
			MSP430F2xxx_EEMMemoryInfo
		> > {};

typedef MemoryModel< Offset<0xF000>, Size<0x1000>, Size<0x100> > MemoryModel1000;
typedef MemoryModel< Offset<0xE000>, Size<0x2000>, Size<0x200> > MemoryModel2000;
typedef MemoryModel< Offset<0xC000>, Size<0x4000>, Size<0x200> > MemoryModel4000;

typedef Features<BC_2xx, false, true, true, false, false, true> MSP430AFE2xx_Features;


struct FunctionMappingAFE2xx : public FunctionMappingBase
{
	FunctionMappingAFE2xx()
	{
		FunctionMappingImpl::fcntMap_ = boost::assign::map_list_of
			(ID_EemDataExchange, ID_EemDataExchangeAFE2xx);
	}
};


template<
	const char* description,
	const char fuses,
	class MemoryModelType
>
struct MSP430AFE2xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MSP430AFE2xx_Match<fuses>,
		LowEemMode,
		MSP430F2xxx_DefaultVoltageTestVpp,
		MSP430AFE2xx_ClockInfo,
		FunctionMappingAFE2xx,
		FuncletMapping1_2xx,
		MemoryModelType,
		MSP430AFE2xx_Features
	> {};

extern const char MSP430AFE220[] = "MSP430AFE220";
extern const char MSP430AFE230[] = "MSP430AFE230";
extern const char MSP430AFE250[] = "MSP430AFE250";
extern const char MSP430AFE221[] = "MSP430AFE221";
extern const char MSP430AFE231[] = "MSP430AFE231";
extern const char MSP430AFE251[] = "MSP430AFE251";
extern const char MSP430AFE222[] = "MSP430AFE222";
extern const char MSP430AFE232[] = "MSP430AFE232";
extern const char MSP430AFE252[] = "MSP430AFE252";
extern const char MSP430AFE223[] = "MSP430AFE223";
extern const char MSP430AFE233[] = "MSP430AFE233";
extern const char MSP430AFE253[] = "MSP430AFE253";

static const DeviceRegistrator< MSP430AFE2xx<MSP430AFE253, 0x0, MemoryModel4000> > regMSP430AFE253;
static const DeviceRegistrator< MSP430AFE2xx<MSP430AFE233, 0x2, MemoryModel2000> > regMSP430AFE233;
static const DeviceRegistrator< MSP430AFE2xx<MSP430AFE223, 0x3, MemoryModel1000> > regMSP430AFE223;

static const DeviceRegistrator< MSP430AFE2xx<MSP430AFE252, 0x4, MemoryModel4000> > regMSP430AFE252;
static const DeviceRegistrator< MSP430AFE2xx<MSP430AFE232, 0x6, MemoryModel2000> > regMSP430AFE232;
static const DeviceRegistrator< MSP430AFE2xx<MSP430AFE222, 0x7, MemoryModel1000> > regMSP430AFE222;

static const DeviceRegistrator< MSP430AFE2xx<MSP430AFE251, 0x8, MemoryModel4000> > regMSP430AFE251;
static const DeviceRegistrator< MSP430AFE2xx<MSP430AFE231, 0xA, MemoryModel2000> > regMSP430AFE231;
static const DeviceRegistrator< MSP430AFE2xx<MSP430AFE221, 0xB, MemoryModel1000> > regMSP430AFE221;

static const DeviceRegistrator< MSP430AFE2xx<MSP430AFE250, 0xC, MemoryModel4000> > regMSP430AFE250;
static const DeviceRegistrator< MSP430AFE2xx<MSP430AFE230, 0xE, MemoryModel2000> > regMSP430AFE230;
static const DeviceRegistrator< MSP430AFE2xx<MSP430AFE220, 0xF, MemoryModel1000> > regMSP430AFE220;
