/*
 * MSP430F5xxx.h
 *
 * Default values for MSP430F5xx familiy - to be used when devices are created, e.g. within MSP43054xx.cpp.
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

#ifndef TEMPLATE_DEVICE_DB_MSP430F5XXX_H
#define TEMPLATE_DEVICE_DB_MSP430F5XXX_H

#if _MSC_VER > 1000
#pragma once
#endif

#include "SpecialMemoryTypes.h"
#include "MSP430Defaults.h"
#include "Registration.h"

namespace TI { namespace DLL430 { namespace TemplateDeviceDb {
	typedef IdCode<0xFFFF, 0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x0> MSP430F5xxxIdMask;

	template<const unsigned int versionId, const unsigned int subVersionId = 0x00>
	struct MSP430F5xxx_Match : Match<
									IdCode<versionId, subVersionId, 0x00, 0x00, 0x00, 0x00, 0x0>, 
									MSP430F5xxxIdMask> {};

	namespace Memory {
		//Is there a way to get size automatically within MemoryMask template?
		typedef NoMask MSP4305xxx_peripherl16bitMask;


		//protectable due to INFO A
		typedef MemoryInfo<
			Name::information, FlashType, Mapped, Protectable, Bits16Type, 
			Size<0x200> , Offset<0x1800>, SegmentSize<0x80>, BankSize<0x80>, Banks<4>,
			NoMask, MemoryCreator<InformationFlashAccess>
		> MSP430F5xxx_InfoFlashMemoryInfo;


		///bsl memory
		typedef MemoryInfo<
			Name::boot, FlashType, Mapped, Protectable, Bits16Type, 
			Size<0x800> , Offset<0x1000>, SegmentSize<0x200>, BankSize<0>, Banks<4>, 
			NoMask, MemoryCreator<BslFlashAccess>
		> MSP430F5xxx_BootFlashMemoryInfo;

		//is bootcode correctly defined?
		typedef MemoryInfo<
			Name::bootCode, RomType, Mapped, Protectable, Bits16Type, 
			Size<0x100> , Offset<0x1A00>, SegmentSize<0x1>, BankSize<0>, Banks<1>, 
			NoMask, MemoryCreator<BootcodeRomAccess>
		> MSP430F5xxx_BootCodeMemoryInfo;

		typedef MemoryInfo<
			Name::peripheral16bit, RegisterType, Mapped, NotProtectable, Bits16Type, 
			Size<0x1000> , Offset<0x0000>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>, 
			MSP4305xxx_peripherl16bitMask
		> MSP430F5xxx_peripherl16lbitMemoryInfo;

		typedef MemoryInfo<
			Name::cpu, RegisterType, NotMapped, NotProtectable, BitsDeviceDefaultType, 
			Size<0x10> , Offset<0x0>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>, 
			NoMask
		> MSP430F5xxx_CPUMemoryInfo;

		typedef MemoryInfo<
			Name::eem, RegisterType, NotMapped, NotProtectable, BitsDeviceDefaultType, 
			Size<0x80> , Offset<0>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>, 
			NoMask
		> MSP430F5xxx_EEMMemoryInfo;


		//size is still definable
		template<class SizeType>
		struct MSP430F5xxx_SystemRamInfo : MemoryInfo<
												Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
												SizeType , Offset<0x1c00>, SegmentSize<0x1>, 
												BankSize<0x0>, Banks<1>, NoMask> {};

		template<class OffsetType, class SizeType>
		struct MSP430F5xxx_SystemRam2Info : MemoryInfo<
												Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
												SizeType , OffsetType, SegmentSize<0x1>, 
												BankSize<0x0>, Banks<1>, NoMask> {};
	} //namespace Memory

	typedef VoltageInfo<1800, 3600, 1800, 2500, 6000, 7000, true> MSP430F5xxx_DefaultVoltageTestVpp;
	typedef VoltageInfo<1800, 3600, 1800, 2500, 6000, 7000, false> MSP430F5xxx_DefaultVoltageNoTestVpp;

	typedef Features<MOD_OSC, false, true, true, false, false, false> MSP430F5xxx_Features;
	typedef ExtendedFeatures<false, false, false, false, false, true, false> MSP430F5xxx_ExtFeatures;
} //namespace TemplateDeviceDb
}//namespace DLL430
}//namespace TI
#endif //TEMPLATE_DEVICE_DB_MSP430F5XXX_H

