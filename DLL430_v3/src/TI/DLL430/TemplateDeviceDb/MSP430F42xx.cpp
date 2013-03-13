/*
 * MSP430F42xx.cpp
 *
 * Definition MSP430F42xx devices.
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

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x1F> MSP430F42xxIdMask;
typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x07> MSP430F42x0IdMask;
typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00> MSP430FW42xIdMask;


template<const unsigned int versionId, const unsigned int fuses>
struct MSP430F42xx_Match : Match< IdCode<versionId, 0x0, 0,0,0, 'E', fuses>, MSP430F42xxIdMask> {};

template<const unsigned int versionId, const unsigned int fuses>
struct MSP430F42x0_Match : Match< IdCode<versionId, 0x0, 0,0,0, 'G', fuses>, MSP430F42x0IdMask> {};

template<const unsigned int versionId>
struct MSP430FW42x_Match : Match< IdCode<versionId, 0x0, 0,0,0,'W', 0>, MSP430FW42xIdMask> {};


typedef ClockInfo<GCC_STANDARD, 0x60D7, EmptyEemTimer, TAClkEemClockNames> MSP430F42xx_ClockInfo;


typedef VoltageInfo<2700, 3600, 2700, 2700, 6000, 7000, false> MSP430F42xxVoltage;
typedef VoltageInfo<1800, 3600, 2500, 2700, 6000, 7000, false> MSP430F42x0Voltage;

template<class FlashOffset, class FlashSize, class RamSize, class LcdMemorySize, class SFRmask>
struct MemoryModel : MemoryList<boost::tuple<
			MSP430F4xxx_MainFlashMemory<FlashSize, FlashOffset>, 
			MSP430F4xxx_InfoFlashMemoryInfo, 
			MSP430F4xxx_BootFlashMemoryInfo, 
			MSP430F4xxx_LcdMemoryInfo<LcdMemorySize>,
			MSP430F4xxx_SystemRamInfo<RamSize>, 
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<SFRmask>,
			MSP430F4xxx_CPUMemoryInfo,
			MSP430F4xxx_EEMMemoryInfo> 
		> {};

typedef MemoryModel< Offset<0xE000>, Size<0x2000>, Size<0x100>, Size<0x15>, sfrMask_f380d380c0ff> MemoryModel3;
typedef MemoryModel< Offset<0xC000>, Size<0x4000>, Size<0x200>, Size<0x15>, sfrMask_f380d380c0ff> MemoryModel5;
typedef MemoryModel< Offset<0x8000>, Size<0x8000>, Size<0x400>, Size<0x15>, sfrMask_f380d380c0ff> MemoryModel7;
typedef MemoryModel< Offset<0x1100>, Size<0xEF00>, Size<0x800>, Size<0x15>, sfrMask_33801380ffff> MemoryModel9;

typedef MemoryModel< Offset<0xE000>, Size<0x2000>, Size<0x100>, Size<0x20>, sfrMask_33801f80ffff> MemoryModel4230;
typedef MemoryModel< Offset<0xC000>, Size<0x4000>, Size<0x100>, Size<0x20>, sfrMask_33801f80ffff> MemoryModel4250;
typedef MemoryModel< Offset<0x8000>, Size<0x8000>, Size<0x100>, Size<0x20>, sfrMask_33801f80ffff> MemoryModel4270;

template<
	const char* description,
	const unsigned int versionId,
	const unsigned int fuses,
	class MemoryModelType
>
struct MSP430F42xx : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MSP430F42xx_Match<versionId, fuses>, 
		LowEemMode,
		MSP430F42xxVoltage,
		MSP430F42xx_ClockInfo,
		FunctionMapping4xx,
		FuncletMapping4xx,
		MemoryModelType,
		MSP430F4xxx_DefaultFeatures,
		MSP430F4xxx_SyncExtFeatures
	> {};

extern const char MSP430F423[] = "MSP430F423";
extern const char MSP430F425[] = "MSP430F425";
extern const char MSP430F427[] = "MSP430F427";

extern const char MSP430F423A[] = "MSP430F423A";
extern const char MSP430F425A[] = "MSP430F425A";
extern const char MSP430F427A[] = "MSP430F427A";

extern const char MSP430FE423[] = "MSP430FE423";
extern const char MSP430FE425[] = "MSP430FE425";
extern const char MSP430FE427[] = "MSP430FE427";

extern const char MSP430FE423A[] = "MSP430FE423A";
extern const char MSP430FE425A[] = "MSP430FE425A";
extern const char MSP430FE427A[] = "MSP430FE427A";


template<
	const char* description,
	const unsigned int versionId,
	const unsigned int fuses,
	class MemoryModelType
>
struct MSP430F42x0 : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MSP430F42x0_Match<versionId, fuses>, 
		LowEemMode,
		MSP430F42x0Voltage,
		MSP430F42xx_ClockInfo,
		FunctionMapping4xx,
		FuncletMapping4xx,
		MemoryModelType,
		MSP430F4xxx_DefaultFeatures,
		ExtendedFeatures<false, false, false, true, true, false, false>
	> {};


extern const char MSP430F4230[] = "MSP430F4230";
extern const char MSP430F4250[] = "MSP430F4250";
extern const char MSP430F4270[] = "MSP430F42x0";

extern const char MSP430FG4250[] = "MSP430FG4250";
extern const char MSP430FG4270[] = "MSP430FG42x0";

extern const char MSP430FE4232[] = "MSP430FE4232";
extern const char MSP430FE4252[] = "MSP430FE42x2";
extern const char MSP430FE4272[] = "MSP430FE4272";

template<
	const char* description,
	const unsigned int versionId,
	class MemoryModelType
>
struct MSP430FW42x : Device<
		description, 
		ObjectId<0>,
		DefaultBits16Type, 
		regular, 
		MSP430FW42x_Match<versionId>, 
		LowEemMode,
		MSP430F4xxx_DefaultVoltageNoTestVpp,
		MSP430F42xx_ClockInfo,
		FunctionMapping4xx,
		FuncletMapping4xx,
		MemoryModelType,
		MSP430F4xxx_DefaultFeatures,
		ExtendedFeatures<false, false, false, false, true, false, false>
	> {};


extern const char MSP430FW429[] = "MSP430FW429";



//                                        description, versionId, fuses, MemoryModel
static const DeviceRegistrator< MSP430F42xx<MSP430F423, 0x27F4, 0x6, MemoryModel3> > regMSP430F423;
static const DeviceRegistrator< MSP430F42xx<MSP430F425, 0x27F4, 0x5, MemoryModel5> > regMSP430F425;
static const DeviceRegistrator< MSP430F42xx<MSP430F427, 0x27F4, 0x4, MemoryModel7> > regMSP430F427;

static const DeviceRegistrator< MSP430F42xx<MSP430F423A, 0x7A42, 0x6, MemoryModel3> > regMSP430F423A;
static const DeviceRegistrator< MSP430F42xx<MSP430F425A, 0x7A42, 0x5, MemoryModel5> > regMSP430F425A;
static const DeviceRegistrator< MSP430F42xx<MSP430F427A, 0x7A42, 0x4, MemoryModel7> > regMSP430F427A;

static const DeviceRegistrator< MSP430F42xx<MSP430FE423, 0x27F4, 0x2, MemoryModel3> > regMSP430FE423;
static const DeviceRegistrator< MSP430F42xx<MSP430FE425, 0x27F4, 0x1, MemoryModel5> > regMSP430FE425;
static const DeviceRegistrator< MSP430F42xx<MSP430FE427, 0x27F4, 0x0, MemoryModel7> > regMSP430FE427;

static const DeviceRegistrator< MSP430F42xx<MSP430FE423A, 0x7A42, 0x2, MemoryModel3> > regMSP430FE423A;
static const DeviceRegistrator< MSP430F42xx<MSP430FE425A, 0x7A42, 0x1, MemoryModel5> > regMSP430FE425A;
static const DeviceRegistrator< MSP430F42xx<MSP430FE427A, 0x7A42, 0x0, MemoryModel7> > regMSP430FE427A;

//FW42x covered in F41xx.cpp

static const DeviceRegistrator< MSP430FW42x<MSP430FW429, 0x29F4, MemoryModel9> > regMSP430FW429;

static const DeviceRegistrator< MSP430F42xx<MSP430FE4232, 0x5242, 0x12, MemoryModel3> > regMSP430FE4232;
static const DeviceRegistrator< MSP430F42xx<MSP430FE4252, 0x5242, 0x11, MemoryModel5> > regMSP430FE4252;
static const DeviceRegistrator< MSP430F42xx<MSP430FE4272, 0x5242, 0x10, MemoryModel7> > regMSP430FE4272;

static const DeviceRegistrator< MSP430F42x0<MSP430F4230, 0x27F4, 0x6, MemoryModel4230> > regMSP430F4230;
static const DeviceRegistrator< MSP430F42x0<MSP430F4250, 0x27F4, 0x5, MemoryModel4250> > regMSP430F4250;
static const DeviceRegistrator< MSP430F42x0<MSP430F4270, 0x27F4, 0x4, MemoryModel4270> > regMSP430F4270;

static const DeviceRegistrator< MSP430F42x0<MSP430FG4250, 0x27F4, 0x1, MemoryModel4250> > regMSP430FG4250;
static const DeviceRegistrator< MSP430F42x0<MSP430FG4270, 0x27F4, 0x0, MemoryModel4270> > regMSP430FG4270;
