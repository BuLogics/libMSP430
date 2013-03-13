/*
 * DeviceUnknown.cpp
 *
 * Definition unknown device for V2 compability.
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

#include "MSP430Defaults.h"
#include "Registration.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

extern const char deviceUnknown[] = "DeviceUnknown";

typedef ClockInfo<GCC_NONE, 0x0000, EmptyEemTimer, EmptyEemClockNames> NoClockInfo;

//"null" id
typedef IdCode<0x00, 0x00, 0xFF, 0xFF, 0xFFFF, -1, 0xFF> IdCodeUnknownType;
//just some mask with no masking capability (all 1s)
typedef IdCode<0xFFFF, 0xFFFF, 0xFF, 0xFF, 0xFFFF, -1, 0xFF> MaskUnknownType;
typedef Match<IdCodeUnknownType, MaskUnknownType> MatchUnknownType;

typedef MemoryInfo<
		deviceUnknown, FlashType, Mapped, NotProtectable, Bits16Type, 
		Size<0> , Offset<0>, SegmentSize<0>, BankSize<0>, Banks<1>, 
		NoMask
	> NoMemoryType;

typedef Device<
		deviceUnknown, 
		ObjectId<0>,
		DefaultBits20Type,
		regular, 
		MatchUnknownType, 
		LargeEemMode,
		VoltageInfo<1800, 3600, 2700, 2500, 6000, 7000, false>,
		NoClockInfo,
		FunctionMappingXv2,
		FuncletMappingNone,
		MemoryList<boost::tuple<
			NoMemoryType
		> >
	> DeviceUnknownType;

static const DeviceRegistrator<DeviceUnknownType> regDeviceUnknownType;
