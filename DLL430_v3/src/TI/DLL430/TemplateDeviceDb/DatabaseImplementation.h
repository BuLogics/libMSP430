/*
 * DatabaseImplementation.h
 *
 * Holds all Module implementations which are necessary to create Device for DeciceDb.
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

#ifndef TEMPLATE_DEVICE_DB_DATABASEIMPLEMENTATION_H
#define TEMPLATE_DEVICE_DB_DATABASEIMPLEMENTATION_H

#if _MSC_VER > 1000
#pragma once
#endif

#include <vector>
#include <boost/tr1/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/assign/list_of.hpp>

#include <inttypes.h>
#include <boost/array.hpp>

#include <DeviceInfo.h>
#include <MemoryAreaBase.h>
#include <FuncletCode.h>

#include "TemplateTypes.h"


namespace TI { namespace DLL430 { namespace TemplateDeviceDb {
/** 
\struct IdCodeImpl.

\brief	Non POD data holding struct which can be initialized at once through ctor.

\note	See IdCode
	
 */
struct IdCodeImpl
{
	const uint16_t verId_;
	const uint16_t verSubId_; 
	const uint8_t revisison_; 
	const uint8_t fab_;
	const uint16_t self_; 
	const char config_;
	const uint8_t fuses_;
	const uint32_t activationKey_;

	IdCodeImpl(
		const uint16_t verId,
		const uint16_t verSubId, 
		const uint8_t revisison, 
		const uint8_t fab,
		const uint16_t self, 
		const char config,
		const uint8_t fuses,
		const uint32_t activationKey
	) : 
		verId_(verId), 
		verSubId_(verSubId), 
		revisison_(revisison), 
		fab_(fab),
		self_(self), 
		config_(config),
		fuses_(fuses),
		activationKey_(activationKey)
	{}
	
};

/** 
\struct MatchImpl.

\brief Non POD data holding struct which can be initialized at once through ctor.

\note	See Match
	
 */
struct MatchImpl
{
	const IdCodeImpl value_;
	const IdCodeImpl mask_;

	MatchImpl(const IdCodeImpl& value, const IdCodeImpl& mask) :
		value_(value),
		mask_(mask)
	{}
};

/** 
\fn		inline bool operator<(const MatchImpl& lhs, const MatchImpl& rhs);

\brief  Needed as MatchImpl is used as key in the device database (default map sort is less)
	
 */
inline bool operator<(const MatchImpl& lhs, const MatchImpl& rhs)
{
	if(lhs.value_.verId_ != rhs.value_.verId_)
		return lhs.value_.verId_ < rhs.value_.verId_;

	else if (lhs.value_.verSubId_ != rhs.value_.verSubId_)
		return  lhs.value_.verSubId_ < rhs.value_.verSubId_;

	else if (lhs.value_.revisison_ != rhs.value_.revisison_)
		return  lhs.value_.revisison_ < rhs.value_.revisison_;

	else if (lhs.value_.fab_ != rhs.value_.fab_)
		return  lhs.value_.fab_ < rhs.value_.fab_;

	else if (lhs.value_.self_ != rhs.value_.self_)
		return  lhs.value_.self_ < rhs.value_.self_;

	else if (lhs.value_.config_ != rhs.value_.config_)
		return  lhs.value_.config_ < rhs.value_.config_;

	else if (lhs.value_.fuses_ != rhs.value_.fuses_)
		return  lhs.value_.fuses_ < rhs.value_.fuses_;

	else
		return  lhs.value_.activationKey_ < rhs.value_.activationKey_;
}

/** 
\fn		inline bool operator==(const MatchImpl& lhs, const MatchImpl& rhs).

\brief  Needed to find value to a given match as MatchImpl is used as key in the device database map file.
	
 */
inline bool operator== (const MatchImpl& match, const IdCodeImpl& idCode)
{
	return ((idCode.verId_ & match.mask_.verId_) == match.value_.verId_
			&& (idCode.verSubId_ & match.mask_.verSubId_) == match.value_.verSubId_
		    && (idCode.revisison_ & match.mask_.revisison_) == match.value_.revisison_
		    && (idCode.fab_ & match.mask_.fab_) == match.value_.fab_
		    && (idCode.self_ & match.mask_.self_) == match.value_.self_
		    && (idCode.config_ & match.mask_.config_) == match.value_.config_
		    && (idCode.fuses_ & match.mask_.fuses_) == match.value_.fuses_
		    && (idCode.activationKey_ & match.mask_.activationKey_) == match.value_.activationKey_);
}

/** 
\struct TriggerImpl.

\brief Non POD data holding struct which can be initialized at once through ctor.

\note See Trigger.

 */
struct TriggerImpl{
	const uint8_t emulation_level_;	
	const uint8_t mem_;			
	const uint8_t reg_;			
	const uint8_t combinations_;
	const uint8_t options_;
	const uint8_t dma_;
	const uint8_t readwrite_;
	const uint8_t regOperations_;
	const uint8_t compLevel_;		
	const uint8_t mem_condLevel_;	
	const uint8_t mem_umaskLevel_; 

	TriggerImpl(
		const uint8_t emulation_level,
		const uint8_t mem,			
		const uint8_t reg,			
		const uint8_t combinations,
		const uint8_t options,
		const uint8_t dma,	
		const uint8_t readwrite,
		const uint8_t regOperations,
		const uint8_t compLevel,		
		const uint8_t mem_condLevel,	
		const uint8_t mem_umaskLevel
	) :
		emulation_level_(emulation_level),
		mem_(mem),		
		reg_(reg),		
		combinations_(combinations),
		options_(options),
		dma_(dma),
		readwrite_(readwrite),
		regOperations_(regOperations),
		compLevel_(compLevel),		
		mem_condLevel_(mem_condLevel),	
		mem_umaskLevel_(mem_umaskLevel)
	{}
};

/** 
\struct SequencerImpl.

\brief Non POD data holding struct which can be initialized at once through ctor.

\note See Sequence.

 */
struct SequencerImpl{
	const uint8_t states_;			
	const uint8_t start_;			
	const uint8_t end_;		
	const uint8_t reset_;			
	const uint8_t blocked_;

	SequencerImpl(
		const uint8_t states,			
		const uint8_t start,			
		const uint8_t end,		
		const uint8_t reset,			
		const uint8_t blocked
	) :
		states_(states),			
		start_(start),			
		end_(end),		
		reset_(reset),			
		blocked_(blocked)
	{}
};

/** 
\struct EemInfoImpl.

\brief Non POD data holding struct which can be initialized at once through ctor.

\note See EemInfo.

 */
struct EemInfoImpl
{
	const uint8_t stateStorage_;		
	const uint8_t cycleCounter_;
	const uint8_t cycleCounterOperations_;
	const TriggerImpl trigger_;
	const SequencerImpl sequencer_;

	EemInfoImpl(
		const uint8_t stateStorage, 
		const uint8_t cycleCounter, 
		const uint8_t cycleCounterOperations, 
		const TriggerImpl& trigger, 
		const SequencerImpl& sequencer
	) :
		stateStorage_(stateStorage),		
		cycleCounter_(cycleCounter),
		cycleCounterOperations_(cycleCounterOperations), 
		trigger_(trigger),
		sequencer_(sequencer)
	{}
};


/** 
\struct VoltageInfoImpl.

\brief Non POD data holding struct which can be initialized at once through ctor.

\note See VoltageInfo.

 */
struct VoltageInfoImpl
{
	const uint16_t vccMin_;
	const uint16_t vccMax_;
	const uint16_t vccFlashMin_;
	const uint16_t vccSecureMin_;
	const uint16_t vppSecureMin_;
	const uint16_t vppSecureMax_;
	const bool hasTestVpp_;

	VoltageInfoImpl(
		const uint16_t vccMin,
		const uint16_t vccMax,
		const uint16_t vccFlashMin,
		const uint16_t vccSecureMin,
		const uint16_t vppSecureMin,
		const uint16_t vppSecureMax,
		const bool hasTestVpp
	) :
		vccMin_(vccMin),
		vccMax_(vccMax),
		vccFlashMin_(vccFlashMin),
		vccSecureMin_(vccSecureMin),
		vppSecureMin_(vppSecureMin),
		vppSecureMax_(vppSecureMax),
		hasTestVpp_(hasTestVpp)
	{}
};


/** 
\struct ClockPair.

\brief	POD holding ETW PID - Value Pairs.

\note	See EemTimer.

*/
struct ClockPair
{
	const std::string name_;
	const uint8_t value_;
	typedef std::string name_type;
	typedef uint8_t value_type;
};


typedef const std::string ClockName;

/** 
\struct EemTimerImpl.

\brief	Non POD data holding struct which can be initialized at once through ctor.

\note	See EemTimer.

\note	Once using C++0x, you could use std::initializer to initialize the array clockControlModules_ 
		with one single parameter of type std::array instead of naming 32 single parameters. 
		Needed as we want to stay as const as possible.

 */
struct EemTimerImpl
{
	struct Timer
	{
		static const ClockPair Empty;
		static const ClockPair CRC16;
		static const ClockPair WDT_A;
		static const ClockPair CCS;
		static const ClockPair USB;
		static const ClockPair AES;

		static const ClockPair TA;
		static const ClockPair TB;

		static const ClockPair TA3;
		static const ClockPair TB3;

		static const ClockPair BT;
		static const ClockPair BT_RTC;

		static const ClockPair TA2_0;
		static const ClockPair TA2_1;
		static const ClockPair TA2_2;
		static const ClockPair TA3_0;
		static const ClockPair TA3_1;
		static const ClockPair TA3_2;
		static const ClockPair TA3_3;
		static const ClockPair TA5_0;
		static const ClockPair TA5_1;
		static const ClockPair TA5_2;
		static const ClockPair TA7_0;
		static const ClockPair TA7_1;
		static const ClockPair TA7_2;

		static const ClockPair TD3_0;
		static const ClockPair TD3_1;
		static const ClockPair TD3_2;
		static const ClockPair TD3_3;

		static const ClockPair TB3_0;
		static const ClockPair TB3_1;
		static const ClockPair TB3_2;
		static const ClockPair TB5_0;
		static const ClockPair TB5_1;
		static const ClockPair TB5_2;
		static const ClockPair TB7_0;
		static const ClockPair TB7_1;
		static const ClockPair TB7_2;

		static const ClockPair FLASH_CTRL;
		static const ClockPair FLASH_CTRLER;

		static const ClockPair USART0;
		static const ClockPair USART1;

		static const ClockPair USCI0;
		static const ClockPair USCI1;
		static const ClockPair USCI2;
		static const ClockPair USCI3;

		static const ClockPair eUSCIA0;
		static const ClockPair eUSCIA1;
		static const ClockPair eUSCIA2;
		static const ClockPair eUSCIA3;
		static const ClockPair eUSCIB0;

		static const ClockPair TB_MCLK;
		static const ClockPair TA_SMCLK;
		static const ClockPair WDT_ACLK;

		static const ClockPair MCLKpin;
		static const ClockPair SMCLKpin;
		static const ClockPair ACLKpin;

		static const ClockPair TACLK;
		static const ClockPair SMCLK;
		static const ClockPair ACLK;

		static const ClockPair RTC;
		static const ClockPair BTRTC;

		static const ClockPair COMP_B;
		static const ClockPair COMP_D;
		static const ClockPair LCD_B;

		static const ClockPair LCD_FREQ;

		static const ClockPair APOOL;

		static const ClockPair RF1A;
		static const ClockPair RF1B;
		static const ClockPair RF2A;
		static const ClockPair RF2B;

		static const ClockPair DAC12_0;
		static const ClockPair DAC12_1;
		static const ClockPair SD16;
		static const ClockPair SD16A_4;
		static const ClockPair SD24B;
		static const ClockPair ADC10_A;
		static const ClockPair ADC10_B;
		static const ClockPair ADC12;
		static const ClockPair ADC12_A;
		static const ClockPair ADC12_B;
	};

	const ClockPair  _0_; const ClockPair  _1_; const ClockPair  _2_; const ClockPair  _3_; 
	const ClockPair  _4_; const ClockPair  _5_; const ClockPair  _6_; const ClockPair  _7_; 
	const ClockPair  _8_; const ClockPair  _9_; const ClockPair _10_; const ClockPair _11_; 
	const ClockPair _12_; const ClockPair _13_; const ClockPair _14_; const ClockPair _15_; 
	const ClockPair _16_; const ClockPair _17_; const ClockPair _18_; const ClockPair _19_; 
	const ClockPair _20_; const ClockPair _21_; const ClockPair _22_; const ClockPair _23_; 
	const ClockPair _24_; const ClockPair _25_; const ClockPair _26_; const ClockPair _27_; 
	const ClockPair _28_; const ClockPair _29_; const ClockPair _30_; const ClockPair _31_; 
protected:

	EemTimerImpl(
		const ClockPair&  _0, const ClockPair&  _1, const ClockPair&  _2, const ClockPair&  _3, 
		const ClockPair&  _4, const ClockPair&  _5, const ClockPair&  _6, const ClockPair&  _7, 
		const ClockPair&  _8, const ClockPair&  _9, const ClockPair& _10, const ClockPair& _11, 
		const ClockPair& _12, const ClockPair& _13, const ClockPair& _14, const ClockPair& _15,
		const ClockPair& _16, const ClockPair& _17, const ClockPair& _18, const ClockPair& _19, 
		const ClockPair& _20, const ClockPair& _21, const ClockPair& _22, const ClockPair& _23, 
		const ClockPair& _24, const ClockPair& _25, const ClockPair& _26, const ClockPair& _27, 
		const ClockPair& _28, const ClockPair& _29, const ClockPair& _30, const ClockPair& _31
	) : _0_(_0), _1_(_1), _2_(_2), _3_(_3), 
		_4_(_4), _5_(_5), _6_(_6), _7_(_7), 
		_8_(_8), _9_(_9), _10_(_10),_11_(_11), 
		_12_(_12), _13_(_13), _14_(_14), _15_(_15),
		_16_(_16), _17_(_17), _18_(_18), _19_(_19), 
		_20_(_20), _21_(_21), _22_(_22), _23_(_23), 
		_24_(_24), _25_(_25), _26_(_26),_27_(_27), 
		_28_(_28), _29_(_29), _30_(_30), _31_(_31)
	{}
};


struct EemClocksImpl
{
	struct Clocks
	{
		static const ClockName Empty;
		static const ClockName ACLK;
		static const ClockName SMCLK;
		static const ClockName TACLK;
	};

	const ClockName  _0_; const ClockName  _1_; const ClockName  _2_; const ClockName  _3_; 
	const ClockName  _4_; const ClockName  _5_; const ClockName  _6_; const ClockName  _7_; 
	const ClockName  _8_; const ClockName  _9_; const ClockName _10_; const ClockName _11_; 
	const ClockName _12_; const ClockName _13_; const ClockName _14_; const ClockName _15_;
protected:

	EemClocksImpl(
		const ClockName&  _0, const ClockName&  _1, const ClockName&  _2, const ClockName&  _3, 
		const ClockName&  _4, const ClockName&  _5, const ClockName&  _6, const ClockName&  _7, 
		const ClockName&  _8, const ClockName&  _9, const ClockName& _10, const ClockName& _11, 
		const ClockName& _12, const ClockName& _13, const ClockName& _14, const ClockName& _15
	) : _0_(_0), _1_(_1), _2_(_2), _3_(_3), 
		_4_(_4), _5_(_5), _6_(_6), _7_(_7), 
		_8_(_8), _9_(_9), _10_(_10),_11_(_11), 
		_12_(_12), _13_(_13), _14_(_14), _15_(_15)
	{}
};


/** 
\struct ClockInfoImpl.

\brief	Non POD data holding struct which can be initialized at once through ctor.

\note	See ClockInfo.

 */
struct ClockInfoImpl 
{
	const uint8_t clockControl_;		
	const uint16_t mclkCntrl0_;
	const EemTimerImpl eemTimer_;
	const EemClocksImpl eemClockNames_;
	ClockInfoImpl(uint8_t clockControl, uint16_t mclkCntrl0, 
				  const EemTimerImpl& eemTimer, const EemClocksImpl& eemClockNames) : 
		clockControl_(clockControl),
		mclkCntrl0_(mclkCntrl0),
		eemTimer_(eemTimer),
		eemClockNames_(eemClockNames)
	{}
};


/** 
\struct MemoryMaskImpl.

\brief	Non POD data holding struct which can be initialized at once through ctor.

\note	See MemoryMask.

 */
struct MemoryMaskImpl
{
	const uint8_t* data_;
	const unsigned int size_;

	MemoryMaskImpl(const uint8_t* data, const unsigned int size) :
		data_(data),
		size_(size)
	{}
};

/** 
\struct MemoryInfoImpl.

\brief	Non POD data holding struct which can be initialized at once through ctor.

\note	See MemoryInfo.

 */
struct MemoryInfoImpl
{	
	const std::string name_;
	const uint32_t flags_;
	const bool protected_;
	const uint32_t size_;
	const uint32_t offset_; 
	const uint32_t seg_size_; 
	const uint32_t bank_size_; 
	const uint32_t banks_;
	const MemoryMaskImpl memoryMaskImpl_;
	const TI::DLL430::MemoryCreatorPtr memoryCreator_;

	MemoryInfoImpl(
		const std::string& name, 
		const uint32_t flags, 
		const bool isProtected,
		const uint32_t size, 
		const uint32_t offset, 
		const uint32_t seg_size, 
		const uint32_t bank_size, 
		const uint32_t banks,
		const MemoryMaskImpl& memoryMaskImpl,
		const TI::DLL430::MemoryCreatorPtr memoryCreator
	) : 
		name_(name),
		flags_(flags),
		protected_(isProtected),
		size_(size),
		offset_(offset) ,
		seg_size_(seg_size) ,
		bank_size_(bank_size) ,
		banks_(banks),
		memoryMaskImpl_(memoryMaskImpl),
		memoryCreator_(memoryCreator)
	{}
};

/** 
\struct FunctionMappingImpl.

\brief	Non POD data holding struct which can be initialized at once through ctor.

\note	See FunctionMapping.

 */
struct FunctionMappingImpl
{
	typedef std::map<unsigned long, uint16_t> FunctionMapping;
	const FunctionMapping& GetMap() const {return fcntMap_;}
protected:
	 FunctionMapping fcntMap_;
	
	 //function to replace single entries, see MSP430Defaults.h
	 void ReplacePair(const FunctionMapping::key_type& key, const FunctionMapping::mapped_type& value)
	 {
		fcntMap_[key] = value;
	 }
};



/** 
\struct FuncletMappingImpl.

\brief	Non POD data holding struct which can be initialized at once through ctor.

\note	See FuncletMapping.

 */
class FuncletMappingImpl
{
public:
	FuncletMappingImpl(FuncletCode erase, FuncletCode write)
	{
		funclets[FuncletCode::ERASE] = erase;
		funclets[FuncletCode::WRITE] = write;
	}

	FuncletMappingImpl(FuncletCode erase, FuncletCode write, FuncletCode unlockBsl)
	{
		funclets[FuncletCode::ERASE] = erase;
		funclets[FuncletCode::WRITE] = write;
		funclets[FuncletCode::BSLUNLOCK] = unlockBsl;
	}

	const std::map<FuncletCode::Type, FuncletCode>& getMap() const
	{
		return funclets;
	}

private:
	std::map<FuncletCode::Type, FuncletCode> funclets;
};


struct PowerSettingsImpl
{
public:
	PowerSettingsImpl(uint32_t powerTestRegMask, 
					  uint32_t testRegEnableLpmx5, 
					  uint32_t testRegDisableLpmx5, 
					  uint16_t powerTestReg3VMask, 
					  uint16_t testReg3VEnableLpmx5, 
					  uint16_t testReg3VDisableLpmx5)
	: powerTestRegMask_(powerTestRegMask), 
	  testRegEnableLpmx5_(testRegEnableLpmx5), 
	  testRegDisableLpmx5_(testRegDisableLpmx5),
      powerTestReg3VMask_(powerTestReg3VMask),
	  testReg3VEnableLpmx5_(testReg3VEnableLpmx5),
	  testReg3VDisableLpmx5_(testReg3VDisableLpmx5){}
	

	uint32_t powerTestRegMask_, testRegEnableLpmx5_, testRegDisableLpmx5_;
	uint16_t powerTestReg3VMask_, testReg3VEnableLpmx5_, testReg3VDisableLpmx5_;
};


struct FeaturesImpl
{
public:
	FeaturesImpl(ClockSystem clock, bool i2c, bool lcfe, bool quickMemRead, 
		bool sflldh, bool hasFram, bool noBsl)
		: clock_(clock), i2c_(i2c), lcfe_(lcfe), quickMemRead_(quickMemRead), 
		  sflldh_(sflldh), hasFram_(hasFram), noBsl_(noBsl) {}

	ClockSystem clock_;
	bool i2c_, lcfe_, quickMemRead_, sflldh_, hasFram_, noBsl_;
};


struct ExtendedFeaturesImpl
{
public:
	ExtendedFeaturesImpl(bool tmr, bool jtag, bool dtc, bool sync, bool instr, bool _1377, bool psach)
		: tmr_(tmr), jtag_(jtag), dtc_(dtc), sync_(sync), instr_(instr), _1377_(_1377), psach_(psach) {}

	bool tmr_, jtag_, dtc_, sync_, instr_, _1377_, psach_;
};


/** 
\struct DeviceImplementation.

\brief	Non POD data holding struct which can be initialized at once through ctor.

\note	See Device.

\note	Retrieve Memory with help of getMemorySize() and getMemoryAt().

\note	Contrary to all other implementation types, all templated devices will derive ffrom this one.
		Needed to get Memory Modules integrated and have runtime iterable map. Idea would be to have Registration 
		construct the needed Device (and only that one as soon as we know which device - and handle on the type of 
		that device in DeviceDbmanagerExt or even throughout the whole program).

*/
struct DeviceImplementation
{
public:
	const std::string description_;
	const size_t objectDbEntry_;
	const uint8_t bits_; 
	const Psa flags_;
	const MatchImpl match_;
	const EemInfoImpl eemInfo_;
	const VoltageInfoImpl voltageInfo_;
	const ClockInfoImpl clockInfo_;
	const FunctionMappingImpl fnctMap_;
	const FuncletMappingImpl funcletMapping_;
	const FeaturesImpl featuresInfo_;
	const ExtendedFeaturesImpl extFeaturesInfo_;
	const PowerSettingsImpl powerSettings_;

public:
	DeviceImplementation(	
		const std::string& description,
		const size_t objectId, 
		const uint8_t bits, 
		const Psa flags,
		const MatchImpl& match,
		const EemInfoImpl& eemInfo,
		const VoltageInfoImpl& voltageInfo,
		const ClockInfoImpl& clockInfo,
		const FunctionMappingImpl& fnctMap,
		const FuncletMappingImpl& funcletMapping,
		const FeaturesImpl& featuresInfo,
		const ExtendedFeaturesImpl& extFeaturesInfo,
		const PowerSettingsImpl& powerSettings
	)
	:
		description_(description),
		objectDbEntry_(objectId),
		bits_(bits),
		flags_(flags),
		match_(match),
		eemInfo_(eemInfo),
		voltageInfo_(voltageInfo),
		clockInfo_(clockInfo),
		fnctMap_(fnctMap),
		funcletMapping_(funcletMapping),
		featuresInfo_(featuresInfo),
		extFeaturesInfo_(extFeaturesInfo),
		powerSettings_(powerSettings)
	{
	
	}

	/*! 
	\brief	Returns number of memory modules.

	\returns number of memory modules

	*/
	unsigned int getMemorySize() const {return DoGetMemorySize();}

	/*! 
	\brief	Returns memory module at specific position.

	\returns memory module

	*/
	const MemoryInfoImpl getMemoryAt(unsigned int idx) const {return DoGetMemoryAt(idx);}

	/*! 
	\brief Return flag enum as uint8_t

	\returns flag as uint8_t

	*/
	uint8_t getFlagsAsUint8_t() const {return static_cast<uint8_t>(flags_);}

protected:
	virtual unsigned int DoGetMemorySize() const = 0;
	virtual const MemoryInfoImpl DoGetMemoryAt(unsigned int idx) const = 0;
};

	namespace Memory {
		//base classes to insure type-safety for memory parameters
		class TypeBase{};
		class MappedBase{};
		class ProtectableBase{};
		class SizeBase{};
		class OffsetBase{};
		class SegSizebase{};
		class BankSizebase{};
		class BanksBase{};
		class MaskBase {}; 
		
		/// All known memory module types at the moment. - In case this list should be extendend
		/// in device modules, make const ints within a namespace, that way you could alter the "list" within another dependent cpp-file.
		/// See and merge with EwtModules
		namespace Name {
			namespace {
				//flash
				extern const char main[] = "main";
				extern const char information[] = "information";
				extern const char boot[] = "boot";
				//ROM
				extern const char bootCode[] = "bootcode";
				//RAM
				extern const char system[] = "system";
				//registers
				extern const char cpu[] = "CPU";
				extern const char eem[] = "EEM"; 
				extern const char sfr[] = "SFR"; 
				extern const char peripheral8bit[] = "peripheral8bit";
				extern const char peripheral16bit[] = "peripheral16bit";
				extern const char lcd[] = "lcd";
			} //namespace
		} //namespace Name
	} //namespace Memory
} //namespace TemplateDeviceDb
}//namespace DLL430
}//namespace TI

#endif //TEMPLATE_DEVICE_DB_DATABASEIMPLEMENTATION_H

