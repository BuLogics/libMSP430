/*
 * MSP430F4xxx.h
 *
 * Default values for MSP430F1xxx familiy - to be used when devices are created, e.g. within MSP43054xx.cpp.
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

#ifndef TEMPLATE_DEVICE_DB_MSP430F4XXX_H
#define TEMPLATE_DEVICE_DB_MSP430F4XXX_H

#if _MSC_VER > 1000
#pragma once
#endif

#include "SpecialMemoryTypes.h"
#include "MSP430Defaults.h"
#include "MSP430F1_2_4xxx_masks.h"
#include "Registration.h"

namespace TI { namespace DLL430 { namespace TemplateDeviceDb {
	namespace Memory {
		typedef MemoryInfo<
			Name::information, FlashType, Mapped, NotProtectable, Bits16Type, 
			Size<0x100> , Offset<0x1000>, SegmentSize<0x80>, BankSize<0x80>, Banks<2>,
			NoMask, MemoryCreator<InformationFlashAccess>
		> MSP430F4xxx_InfoFlashMemoryInfo;

		///bsl memory
		typedef MemoryInfo<
			Name::boot, FlashType, Mapped, Protectable, Bits16Type, 
			Size<0x400> , Offset<0x0c00>, SegmentSize<0x200>, BankSize<0>, Banks<1>, 
			NoMask, MemoryCreator<BslFlashAccess>
		> MSP430F4xxx_BootFlashMemoryInfo;

		typedef MemoryInfo<
			Name::cpu, RegisterType, NotMapped, NotProtectable, BitsDeviceDefaultType, 
			Size<0x10> , Offset<0x0>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>, 
			NoMask
		> MSP430F4xxx_CPUMemoryInfo;

		typedef MemoryInfo<
			Name::eem, RegisterType, NotMapped, NotProtectable, BitsDeviceDefaultType, 
			Size<0x80> , Offset<0>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>, 
			NoMask
		> MSP430F4xxx_EEMMemoryInfo;

		template<class SizeType>
		struct MSP430F4xxx_LcdMemoryInfo : MemoryInfo<
						Name::lcd, RamType, Mapped, NotProtectable, Bits16Type, 
						SizeType , Offset<0x90>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>, 
						NoMask> {};

		//size is still definable
		template<class SizeType>
		struct MSP430F4xxx_SystemRamInfo : MemoryInfo<
						Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
						SizeType , Offset<0x200>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>, 
						NoMask> {};

		template<class SizeType>
		struct MSP430F4xxx_SystemRam2Info : MemoryInfo<
						Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
						SizeType , Offset<0x1100>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>, 
						NoMask> {};

		template<class FlashSizeType, class OffsetType>
		struct MSP430F4xxx_MainFlashMemory : MemoryInfo<
						Name::main, FlashType, Mapped, NotProtectable, Bits16Type, 
						FlashSizeType , OffsetType, SegmentSize<0x200>, BankSize<0x10000>, Banks<1>, 
						NoMask> {};
	} //namespace Memory

	typedef VoltageInfo<1800, 3600, 2700, 2500, 6000, 7000, true> MSP430F4xxx_DefaultVoltageTestVpp;
	typedef VoltageInfo<1800, 3600, 2700, 2500, 6000, 7000, false> MSP430F4xxx_DefaultVoltageNoTestVpp;

	typedef Features<FLLPLUS, false, false, true, true, false, false> MSP430F4xxx_DefaultFeatures;
	typedef Features<FLLPLUS, false, false, true, false, false, false> MSP430F43xx_Features;
	typedef Features<FLLPLUS, false, true, true, false, false, false> MSP430F46_7xx_Features;

	typedef ExtendedFeatures<false, false, false, true, false, false, false> MSP430F4xxx_SyncExtFeatures;

	struct FunctionMapping4xx : public FunctionMappingNone
	{
		FunctionMapping4xx() {
			ReplacePair(ID_GetDcoFrequency, ID_GetFllFrequency);
		}
	};

	struct FunctionMappingX4xx : public FunctionMappingX
	{
		FunctionMappingX4xx() {
			ReplacePair(ID_GetDcoFrequency, ID_GetFllFrequencyX);
		}
	};

	struct FuncletMapping4xx : FuncletMappingImpl
	{
		FuncletMapping4xx() 
			: FuncletMappingImpl(
				FuncletCode( eraseFuncletCodeFLL, sizeof(eraseFuncletCodeFLL), 4 ),
				FuncletCode( writeFuncletCodeFLL, sizeof(writeFuncletCodeFLL), 128 ) ) 
		{}
	};

	struct FuncletMappingX4xx : FuncletMappingImpl
	{
		FuncletMappingX4xx() 
			: FuncletMappingImpl(
				FuncletCode( eraseFuncletCodeXFLL, sizeof(eraseFuncletCodeXFLL), 4 ),
				FuncletCode( writeFuncletCodeXFLL, sizeof(writeFuncletCodeXFLL), 256 ) ) 
		{}
	};

} //namespace TemplateDeviceDb
}//namespace DLL430
}//namespace TI
#endif //TEMPLATE_DEVICE_DB_MSP430F1XXX_H

