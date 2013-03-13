/*
 * Registration.h
 *
 * Basic device creating engine - interface mechanism to create devices and to convert them to be used by MSP430v3.dll. 
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

#ifndef TEMPLATE_DEVICE_DB_REGISTRATION_H
#define TEMPLATE_DEVICE_DB_REGISTRATION_H

#if _MSC_VER > 1000
#pragma once
#endif

#include <string>
#include <iostream>	
#include <map>

#include <boost/static_assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/type_traits/is_empty.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>

#include <hal.h>

#include "DatabaseImplementation.h"

/** \file	Registration.h
	\brief	This is the interface if you want to add new devices.
	\note	If you need examples look at MSP430Defaults.h and MSP430F5xxx.cpp.
	\note	Having an IDE with code completion can help you immensly filling out the device.
	\note	All templates call their Impl counterparts from DatabaseImplementation.h theirs ctor taking all parameters at once.
 */
namespace TI { namespace DLL430 { namespace TemplateDeviceDb {

template<
	const uint16_t verId, 
	const uint16_t verSubId, 
	const uint8_t  revisison, 
	const uint8_t  fab,
	const uint16_t self, 
	const char config, 
	const uint8_t fuses,
	const uint32_t activationKey = 0
>
struct IdCode : IdCodeImpl
{
	IdCode() : 
		IdCodeImpl(verId, verSubId, revisison, fab, self, config, fuses, activationKey)
	{}
};

template<
	class Value, 
	class Mask
>
struct Match : MatchImpl
{
	typedef Value Value_type;
	typedef Mask Mask_type;

	Match() :
		MatchImpl(Value_type(), Mask_type())
	{}
};

template<
		const uint8_t emulation_level,	///< backwards compatible to C-API 
		const uint8_t mem,				///< number of memory triggers
		const uint8_t reg,				///< number of register-write triggers
		const uint8_t combinations,		///< number of trigger combinations (trigger groups)
		const uint8_t options,			///< number of trigger options modes
		const uint8_t dma,				///< number of trigger dma modes
		const uint8_t readwrite,		///< number of trigger read/write modes
		const uint8_t regTrigOperations,///< number of register trigger operations
		const uint8_t compLevel,		///< 0: only ==, 1: == and !=, 2: all comparators
		const uint8_t mem_condLevel,	///< number of TRIGx bits - 2 > 0,1,2
		const uint8_t mem_umaskLevel	///< 0: only combinations of 0xFF, 0x00FF( and 0xF0000), 1: all values
>
struct Trigger : TriggerImpl
{
	Trigger() :
		TriggerImpl(emulation_level, mem, reg, combinations, 
			        options, dma, readwrite, regTrigOperations,
					compLevel, mem_condLevel, mem_umaskLevel)
	{}
};

template<
	const uint8_t states,			///< number of sequencer states (0 means "not present")
	const uint8_t start,			///< first trigger combination number
	const uint8_t end,				///< last trigger combination number
	const uint8_t reset,			///< trigger combination number that resets the sequencer
	const uint8_t blocked			///< trigger combination number that is blocked when the sequencer is active
>
struct Sequencer : SequencerImpl
{
	Sequencer() :
		SequencerImpl(states, start, end, reset, blocked)
	{}
} ;

template<
	const uint8_t stateStorage,		///< depth of available state storage modul. (0 means "not present")
	const uint8_t cycleCounter,		///< number of available cycle counters. (0 means "not present")
	const uint8_t cycleCounterOps,
	class Trigger, 
	class Sequencer
>
struct EemInfo  : EemInfoImpl
{
	typedef Trigger Trigger_type;
	typedef Sequencer Sequencer_type;

	EemInfo () :
		EemInfoImpl(stateStorage, cycleCounter, cycleCounterOps, Trigger_type(), Sequencer_type())
	{}
};

template<
	const uint16_t vccMin,
	const uint16_t vccMax,
	const uint16_t vccFlashMin,
	const uint16_t vccSecureMin,
	const uint16_t vppSecureMin,
	const uint16_t vppSecureMax,
	const bool hasTestVpp
>
struct VoltageInfo  : VoltageInfoImpl
{
	VoltageInfo () :
		VoltageInfoImpl(vccMin, vccMax, vccFlashMin, vccSecureMin, vppSecureMin, vppSecureMax, hasTestVpp)
	{}
};

template<
	const uint8_t clockControl, 
	const uint16_t mclkCntrl0,
	class EemTimer,
	class EemClockNames
>
struct ClockInfo : ClockInfoImpl
{
	typedef EemTimer EemTimer_type;
	typedef EemClockNames EemClockNames_type;

	ClockInfo() :
		ClockInfoImpl(clockControl, mclkCntrl0, EemTimer_type(), EemClockNames())
	{}
};


class MemoryListBase{};
//Memory list - access elements with use of struct GetAt
template<class List>
struct MemoryList : MemoryListBase
{	
	typedef List list_type;
	
	MemoryList() 
	{}
};


/** 
\struct GetAt
\brief Recursive getter for Types out of Tuples.

\param lastIdx Highest Index within TupleType

\param TupleType collection of types to iterate

*/
template<const unsigned int lastIdx, class TupleType>
struct GetAt
{
	const unsigned int idx_;
	GetAt(const unsigned int idx) : idx_(idx)
    {
        assert(idx_ <= lastIdx);
	}
	const MemoryInfoImpl Do()
	{
		if(idx_ == lastIdx) return typename boost::tuples::element<lastIdx, TupleType>::type();
		else return GetAt<lastIdx-1, TupleType>(idx_).Do();
	}

}; 

/** 
\struct GetAt<0, TupleType>
\brief  Specialization of recursive getter for Types out of Tuples above.
		Returns last (or in this case first) Type from the tuples - needed to break recursion.
\param TupleType collection of types to iterate

*/
template<class TupleType>
struct GetAt<0, TupleType>
{
	const unsigned int idx_;
	GetAt(const unsigned int idx) : idx_(idx)
    {
        assert(idx_ == 0);
	}
	const MemoryInfoImpl Do()
	{
		return typename boost::tuples::element<0, TupleType>::type();
	}
}; 

namespace Memory {
		template<uint32_t type> 
		struct Type : public SingleUint32<type>, TypeBase {};

		typedef Type<TI::DLL430::DeviceInfo::MEMTYPE_FLASH>		FlashType;
		typedef Type<TI::DLL430::DeviceInfo::MEMTYPE_ROM>		RomType;
		typedef Type<TI::DLL430::DeviceInfo::MEMTYPE_RAM>		RamType;
		typedef Type<TI::DLL430::DeviceInfo::MEMTYPE_REGISTER>  RegisterType;
		
		typedef Bits<0>		BitsDeviceDefaultType;
		typedef Bits<8>		Bits8Type;
		typedef Bits<16>	Bits16Type;
		typedef Bits<20>	Bits20Type;

		//types to make memory module definitions type safe!
		template<bool mapped>
		struct IsMapped : public SingleBool<mapped>, MappedBase {};

		//default types to default file
		typedef IsMapped<true> Mapped;
		typedef IsMapped<false> NotMapped;

		template<bool prot>
		struct IsProtectable : public SingleBool<prot>, ProtectableBase {};

		//default types to default file?
		typedef IsProtectable<true> Protectable;
		typedef IsProtectable<false> NotProtectable;
		
		template<uint32_t N>
		struct Size : public SingleUint32<N>, SizeBase {};

		
		template<uint32_t N>
		struct Offset : public SingleUint32<N>, OffsetBase {};

		
		template<uint32_t N>
		struct SegmentSize : public SingleUint32<N>, SegSizebase {};
		
		
		template<uint32_t N>
		struct BankSize : public SingleUint32<N>, BankSizebase {};

		
		template<uint32_t N>
		struct Banks : public SingleUint32<N>, BanksBase {};
	
		

		template<const uint8_t data[], const int N>
		struct MemoryMask : MaskBase
		{
			const MemoryMaskImpl memoryMask_value;

			MemoryMask() :
				memoryMask_value(data, N)
			{}
		};

		struct NoMask : MaskBase
		{
			const MemoryMaskImpl memoryMask_value;

			NoMask() : memoryMask_value(0, 0) {}
		};

		//Use this for registering own MemoryAreaBaseTypes
		template<class MemoryType>
		struct MemoryCreator : public TI::DLL430::MemoryCreatorBase
		{
			BOOST_STATIC_ASSERT((boost::is_base_of<TI::DLL430::MemoryAreaBase, MemoryType>::value));

			virtual bool isImplemented() const {return true;}
			virtual TI::DLL430::MemoryAreaBase* operator()(
				const std::string& name, TI::DLL430::DeviceHandleV3* devHandle,
				uint32_t start, uint32_t end, 
				uint32_t seg, uint32_t banks, 
				bool mapped, const bool protectable, TI::DLL430::MemoryManager* mm, uint8_t psa
			) const 
			{
				return new MemoryType(
					name,devHandle,
					start,end,seg,banks,
					mapped,protectable,mm,psa
				);
			}
		};

		//don't use this - it's implementation for NoMemoryCreator
		template<>
		struct MemoryCreator<void> : TI::DLL430::MemoryCreatorBase
		{
			virtual bool isImplemented() const {return false;}
			virtual TI::DLL430::MemoryAreaBase* operator()(
				const std::string& name, TI::DLL430::DeviceHandleV3* devHandle,
				uint32_t start, uint32_t end, 
				uint32_t seg, uint32_t banks, 
				bool mapped, const bool protectable, TI::DLL430::MemoryManager* mm, uint8_t psa
			) const  
			{
				return NULL;
			}
		};

		struct NoMemoryCreator : MemoryCreator<void> {};
	} //namespace Memory


template<
	const char* name,
	class TypeType,
	class MappedType,
	class ProtectableType,
	class BitsType,
	class SizeType, 
	class OffsetType,
	class SegmentSizeType,
	class BankSizeType,
	class BanksType,
	class MaskType = Memory::NoMask,
	class MemoryCreatorType = Memory::NoMemoryCreator
>
struct MemoryInfo : MemoryInfoImpl
{
	BOOST_STATIC_ASSERT((boost::is_base_of<Memory::TypeBase, TypeType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<Memory::MappedBase, MappedType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<Memory::ProtectableBase, ProtectableType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<BitsBase, BitsType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<Memory::SizeBase, SizeType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<Memory::OffsetBase, OffsetType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<Memory::SegSizebase, SegmentSizeType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<Memory::BankSizebase, BankSizeType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<Memory::BanksBase, BanksType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<Memory::MaskBase, MaskType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<TI::DLL430::MemoryCreatorBase, MemoryCreatorType>::value));

	typedef MaskType Mask_type;

	MemoryInfo() :
		MemoryInfoImpl(
			std::string(name), 
			CreateFlags(TypeType().value, MappedType().value, BitsType().value), 
			ProtectableType().value, 
			SizeType().value, 
			OffsetType().value, 
			SegmentSizeType().value, 
			BankSizeType().value, 
			BanksType().value, 
			Mask_type().memoryMask_value,
			TI::DLL430::MemoryCreatorPtr(new MemoryCreatorType)
		)
	{}
};


/// \note device settings used for LPMx.5
template<uint32_t powerTestRegMask, 
		 uint32_t enableLpmx5TestReg, 
		 uint32_t disableLpmx5TestReg, 
		 uint16_t powerTestReg3VMask, 
		 uint16_t enableLpmx5TestReg3V, 
		 uint16_t disableLpmx5TestReg3V>
struct PowerSettings : PowerSettingsImpl
{
	PowerSettings() : PowerSettingsImpl(powerTestRegMask, enableLpmx5TestReg, disableLpmx5TestReg, powerTestReg3VMask, enableLpmx5TestReg3V, disableLpmx5TestReg3V) {}
};

typedef PowerSettings<0, 0, 0 , 0, 0, 0> NoPowerSettings;


template<ClockSystem clock, bool i2c, bool lcfe, bool quickMemRead, bool sflldh, bool hasFram, bool noBsl>
struct Features : FeaturesImpl
{
	Features() : FeaturesImpl(clock, i2c, lcfe, quickMemRead, sflldh, hasFram, noBsl) {}
};

typedef Features<BC_2xx, false, true, true, false, false, false> DefaultFeatures;


template<bool tmr, bool jtag, bool dtc, bool sync, bool instr, bool _1377, bool psach>
struct ExtendedFeatures : ExtendedFeaturesImpl
{
	ExtendedFeatures() : ExtendedFeaturesImpl(tmr, jtag, dtc, sync, instr, _1377, psach) {}
};

typedef ExtendedFeatures<false, false, false, false, false, false, false> NoExtendedFeatures;



template<
	const char* description, 
	class ObjectIdType, 
	class DefaultBitsType,
	const Psa flags, 
	class MatchType, 
	class EemInfoType, 
	class VoltageInfoType, 
	class ClockInfoType,
	class FunctionMappingType,
	class FuncletMappingType,
	class MemoryListType,
	class FeaturesType = DefaultFeatures,
	class ExtendedFeaturesType = NoExtendedFeatures,
	class PowerSettingsType = NoPowerSettings
>
struct Device : public DeviceImplementation
{	
	BOOST_STATIC_ASSERT((boost::is_base_of<ObjectIdBase, ObjectIdType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<DefaultBitsBase, DefaultBitsType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<MatchImpl, MatchType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<EemInfoImpl, EemInfoType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<VoltageInfoImpl, VoltageInfoType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<ClockInfoImpl, ClockInfoType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<FunctionMappingImpl, FunctionMappingType>::value)); //Note as functionmapping is working via dynamic polymorphie instead of templates, this is correct
	BOOST_STATIC_ASSERT((boost::is_base_of<FuncletMappingImpl, FuncletMappingType>::value));
	BOOST_STATIC_ASSERT((boost::is_base_of<MemoryListBase, MemoryListType>::value));
	
	typedef MatchType Match_type;
	
	Device() : DeviceImplementation(
		std::string(description),
		ObjectIdType().value, 
		DefaultBitsType().value, 
		flags,
		MatchType(),
		EemInfoType(),
		VoltageInfoType(),
		ClockInfoType(),
		FunctionMappingType(),
		FuncletMappingType(),
		FeaturesType(),
		ExtendedFeaturesType(),
		PowerSettingsType()
	)
	{
	
	}
	//instead of virtual function, use typedefs to get instantiations
	virtual unsigned int DoGetMemorySize() const
	{
		typedef typename MemoryListType::list_type ListType;
		return boost::tuples::length<ListType>::value;
	}

	virtual const MemoryInfoImpl DoGetMemoryAt(unsigned int idx) const
	{
		typedef typename MemoryListType::list_type ListType;
		const unsigned int size = boost::tuples::length<ListType>::value;
		assert(idx < size);
		return GetAt<size - 1, ListType>(idx).Do();
	}
};

struct DeviceCreatorBase
{
	typedef boost::shared_ptr<DeviceImplementation> DeviceTypePtr;
	virtual DeviceTypePtr create() const = 0;
};

template<class DeviceType>
struct DeviceCreator : DeviceCreatorBase
{
	BOOST_STATIC_ASSERT((boost::is_base_of<DeviceImplementation, DeviceType>::value));
	virtual DeviceTypePtr create() const {return DeviceTypePtr(new DeviceType); }
};


class Registration
{
public:
	typedef boost::shared_ptr<DeviceCreatorBase> DeviceCreatorPtr;
	//as registration acts on module global map, you can use it as RValue
	Registration();

	size_t FindAndPrepareDevice(const IdCodeImpl& match);
	DeviceInfoPtr GetDeviceInfo(size_t id) const;
	size_t GetDatabaseSize() const;
	bool HasCurrentDeviceVpp() const;
	const char* GetCurrentDeviceDescription() const;
	void insertDeviceCreator(const MatchImpl& id, DeviceCreatorPtr devCreator);
private:
	//helper function to copy ClockPairs from eemTimer to CreateClockModuleNames of DeviceInfo
	void CreateClockModuleNames(DeviceInfo::ClockMapping& clockMapping, const EemTimerImpl& eemTimer) const;
	void CreateClockNames(DeviceInfo::ClockNames& clockNames, const EemClocksImpl& eemClocks) const;
};


template<class DeviceType>
class DeviceRegistrator
{
public:
	DeviceRegistrator()
	{
		//Just fill a vector with ids - create heap based object on request to save size
		Registration().insertDeviceCreator(
			typename DeviceType::Match_type(), 
			Registration::DeviceCreatorPtr(new DeviceCreator<DeviceType>)
		);
	}
	void Register(){}
};
} //namespace TemplateDeviceDb
}//namespace DLL430
}//namespace TI
#endif //TEMPLATE_DEVICE_DB_REGISTRATION_H
