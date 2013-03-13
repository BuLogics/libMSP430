/*
 * MSP430F54xx.cpp
 *
 * Definition MSP430F54xx devices.
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

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

const ClockPair MSP430F54xxTimerTA3_1 = {"Timer1_A3", 57};

struct LargeEemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	LargeEemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::ADC12_A, Eem::RTC, Eem::USCI3, Eem::USCI2,
		Eem::USCI1, Eem::USCI0, Eem::Empty, Eem::Empty,
		Eem::TB7_0, MSP430F54xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct SmallUsciEemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	SmallUsciEemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::ADC12_A, Eem::RTC, Eem::Empty, Eem::Empty,
		Eem::USCI1, Eem::USCI0, Eem::Empty, Eem::Empty,
		Eem::TB7_0, MSP430F54xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};


typedef ClockInfo<GCC_EXTENDED, 0x040F, LargeEemTimer, EmptyEemClockNames> LargeClockInfo;			// default settings as they are defined in v2
typedef ClockInfo<GCC_EXTENDED, 0x040F, SmallUsciEemTimer, EmptyEemClockNames> SmallUsciClockInfo;

template<const unsigned int nrFlashBlocks>
struct MSP430F54xx_MainFlashMemory : MemoryInfo<
										Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
										Size<nrFlashBlocks*0x40*0x400> , Offset<0x5C00>, 
										SegmentSize<0x200>, BankSize<0x10000>, Banks<4>, NoMask> {};

template<const unsigned int nrFlashBlock>
struct MSP430F54xxAMemoryList : MemoryList<boost::tuple<
									MSP430F54xx_MainFlashMemory<nrFlashBlock>,
									MSP430F5xxx_InfoFlashMemoryInfo, 
									MSP430F5xxx_BootFlashMemoryInfo, 
									MSP430F5xxx_BootCodeMemoryInfo,
									MSP430F5xxx_SystemRamInfo<Size<0x4000> >,
									MSP430F5xxx_peripherl16lbitMemoryInfo, 
									MSP430F5xxx_CPUMemoryInfo, 
									MSP430F5xxx_EEMMemoryInfo
								> > {};

template<const unsigned int nrFlashBlock>
struct MSP430F54xxNoBSLMemoryList : MemoryList<boost::tuple<
										MSP430F54xx_MainFlashMemory<nrFlashBlock>,
										MSP430F5xxx_InfoFlashMemoryInfo, 
										MSP430F5xxx_BootCodeMemoryInfo,
										MSP430F5xxx_SystemRamInfo<Size<0x4000> >,
										MSP430F5xxx_peripherl16lbitMemoryInfo, 
										MSP430F5xxx_CPUMemoryInfo, 
										MSP430F5xxx_EEMMemoryInfo
									> > {};

typedef MSP430F54xxAMemoryList<2> A2FlashBlocks;
typedef MSP430F54xxAMemoryList<3> A3FlashBlocks;
typedef MSP430F54xxAMemoryList<4> A4FlashBlocks;

typedef ExtendedFeatures<false, false, false, false, false, false, false> MSP430F5438_ExtFeatures;

template<
	const char* description,
	const unsigned int versionId,
	const unsigned int subVersionId,
	typename ClockInfo,
	typename MemoryList,
	typename ExtendedFeatures = MSP430F5xxx_ExtFeatures
>
struct MSP430F54xx : Device<
						description, 
						ObjectId<0>,
						DefaultBits20Type,
						regular, 
						MSP430F5xxx_Match<versionId, subVersionId>, 
						LargeEemMode, 
						MSP430F5xxx_DefaultVoltageTestVpp,
						ClockInfo,
						FunctionMappingXv2,
						FuncletMappingXv2,
						MemoryList,
						MSP430F5xxx_Features,
						ExtendedFeatures
					> {};

extern const char MSP430F5418[] = "MSP430F5418";
extern const char MSP430F5419[] = "MSP430F5419";
extern const char MSP430F5435[] = "MSP430F5435";
extern const char MSP430F5436[] = "MSP430F5436";
extern const char MSP430F5437[] = "MSP430F5437";
extern const char MSP430F5438[] = "MSP430F5438";

extern const char MSP430F5418A[] = "MSP430F5418A";
extern const char MSP430F5419A[] = "MSP430F5419A";
extern const char MSP430F5435A[] = "MSP430F5435A";
extern const char MSP430F5436A[] = "MSP430F5436A";
extern const char MSP430F5437A[] = "MSP430F5437A";
extern const char MSP430F5438A[] = "MSP430F5438A";

extern const char MSP430SL5438A[] = "MSP430SL5438A";

//description, versionId, subversionId, clock info, memory with nrFlashBlocks
typedef MSP430F54xx<MSP430F5418, 0x1854, 0x00, SmallUsciClockInfo, A2FlashBlocks> MSP430F5418_type;
typedef MSP430F54xx<MSP430F5419, 0x1954, 0x00, LargeClockInfo, A2FlashBlocks> MSP430F5419_type;

typedef MSP430F54xx<MSP430F5435, 0x3554, 0x00, SmallUsciClockInfo, A3FlashBlocks> MSP430F5435_type;
typedef MSP430F54xx<MSP430F5436, 0x3654, 0x00, LargeClockInfo, A3FlashBlocks> MSP430F5436_type;

typedef MSP430F54xx<MSP430F5437, 0x3754, 0x00, SmallUsciClockInfo, A4FlashBlocks> MSP430F5437_type;
typedef MSP430F54xx<MSP430F5438, 0x3854, 0x00, LargeClockInfo, A4FlashBlocks, MSP430F5438_ExtFeatures> MSP430F5438_type;

typedef MSP430F54xx<MSP430F5418A, 0x8000, 0x00, SmallUsciClockInfo, A2FlashBlocks> MSP430F5418A_type;
typedef MSP430F54xx<MSP430F5419A, 0x8001, 0x00, LargeClockInfo, A2FlashBlocks> MSP430F5419A_type;

typedef MSP430F54xx<MSP430F5435A, 0x8002, 0x00, SmallUsciClockInfo, A3FlashBlocks> MSP430F5435A_type;
typedef MSP430F54xx<MSP430F5436A, 0x8003, 0x00, LargeClockInfo, A3FlashBlocks> MSP430F5436A_type;

typedef MSP430F54xx<MSP430F5437A, 0x8004, 0x00, SmallUsciClockInfo, A4FlashBlocks> MSP430F5437A_type;
typedef MSP430F54xx<MSP430F5438A, 0x8005, 0x00, LargeClockInfo, A4FlashBlocks> MSP430F5438A_type;

typedef MSP430F54xx<MSP430F5438A, 0x8005, 0x01, LargeClockInfo, A4FlashBlocks> MSP430BT5190_type;

typedef MSP430F54xx<MSP430SL5438A, 0x81EE, 0x01, LargeClockInfo, A4FlashBlocks> MSP430SL5438A_type;

static const DeviceRegistrator<MSP430F5418_type> regMSP430F5418;
static const DeviceRegistrator<MSP430F5419_type> regMSP430F5419;
static const DeviceRegistrator<MSP430F5435_type> regMSP430F5435;
static const DeviceRegistrator<MSP430F5436_type> regMSP430F5436;
static const DeviceRegistrator<MSP430F5437_type> regMSP430F5437;
static const DeviceRegistrator<MSP430F5438_type> regMSP430F5438;

static const DeviceRegistrator<MSP430F5418A_type> regMSP430F5418A;
static const DeviceRegistrator<MSP430F5419A_type> regMSP430F5419A;
static const DeviceRegistrator<MSP430F5435A_type> regMSP430F5435A;
static const DeviceRegistrator<MSP430F5436A_type> regMSP430F5436A;
static const DeviceRegistrator<MSP430F5437A_type> regMSP430F5437A;
static const DeviceRegistrator<MSP430F5438A_type> regMSP430F5438A;

static const DeviceRegistrator<MSP430BT5190_type> regMSP430BT5190;

static const DeviceRegistrator<MSP430SL5438A_type> regMSP430SL5438A;
