/*
 * Registration.cpp
 *
 * Registration of devices at central instance to be accessed by a DeviceDbManager.
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

#include "Registration.h"

#include <iostream>
#include <map>
#include <vector>
#include "boost/pool/detail/singleton.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

#include <DeviceInfo.h>

#include "DatabaseImplementation.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;

#define DLL430_FLAG_TESTVPP (1 << 7)
#define DLL430_PSA_REGULAR  0
#define DLL430_PSA_ENHANCED 1
#define DLL430_PSA_MASK     0x1
#define DLL430_MEMFLAG_FLASH     (0x0 << 8) /**< flash memory (main or information)*/
#define DLL430_MEMFLAG_ROM       (0x1 << 8) /**< read only memory (boot)*/
#define DLL430_MEMFLAG_RAM       (0x2 << 8) /**< random access memory */
#define DLL430_MEMFLAG_REG       (0x3 << 8) /**< registers (peripheral, cpu or eem) */
#define DLL430_MEMFLAG_MAPPED    (1 << 7)   /**< memory type is mapped to global device address */
#define DLL430_MEMFLAG_BITS_MASK 0x7F       /**< the lowest 7 bits of the flags are fixed bit width (8/16/20) or target bit width (0) */

typedef std::map<const MatchImpl, const Registration::DeviceCreatorPtr> DeviceMapImpl;
typedef DeviceMapImpl::const_iterator DeviceMapConstIterator;
typedef boost::details::pool::singleton_default<DeviceMapImpl> DeviceMap;
DeviceCreatorBase::DeviceTypePtr currentDevice_;

Registration::Registration() 
{

}

size_t Registration::GetDatabaseSize() const
{
	return DeviceMap::instance().size();
}

void Registration::insertDeviceCreator(const MatchImpl& match, DeviceCreatorPtr devCreator)
{
	DeviceMap::instance().insert(std::make_pair(match, devCreator));
}

size_t Registration::FindAndPrepareDevice(const IdCodeImpl& idCode)
{
	const DeviceMapImpl& map = DeviceMap::instance();
	
	DeviceMapConstIterator it = map.begin();
	for(; it != map.end(); ++it) 
	{
		if(it->first == idCode) 
		{
			break;
		}
	}

	if(it != map.end() ) 
	{
		Registration::DeviceCreatorPtr creator = (*it).second;
		currentDevice_ = (*creator).create();
		return std::distance<DeviceMapConstIterator>(map.begin(), it);
	}
	return -1;
}

const char* Registration::GetCurrentDeviceDescription() const
{
	return currentDevice_->description_.c_str();
}

bool Registration::HasCurrentDeviceVpp() const
{
	return currentDevice_->voltageInfo_.hasTestVpp_;
}

DeviceInfoPtr Registration::GetDeviceInfo(size_t id) const 
{
	using TI::DLL430::DeviceInfo;

	DeviceMapConstIterator it = DeviceMap::instance().begin();
	if(id < DeviceMap::instance().size())
	{
		std::advance(it, id);
	}
	else
	{
		it = DeviceMap::instance().end();
	}
	DeviceInfoPtr info = DeviceInfoPtr(new DeviceInfo);
	
	if(it == DeviceMap::instance().end())
	{
		return info;
	}

	const Registration::DeviceCreatorPtr devCreatorPtr = (*it).second;
	const DeviceCreatorBase::DeviceTypePtr currDevice = devCreatorPtr->create();
	currentDevice_ = currDevice;

	info->setObjectId(currDevice->objectDbEntry_);
	uint8_t bits = currDevice->bits_;
	switch (currDevice->flags_ & DLL430_PSA_MASK) 
	{
	case DLL430_PSA_REGULAR:
		info->setPsaType(DeviceInfo::PSATYPE_REGULAR);
		break;	
	case DLL430_PSA_ENHANCED:
		info->setPsaType(DeviceInfo::PSATYPE_ENHANCED);
		break;
	}

	info->minVcc(currDevice->voltageInfo_.vccMin_);
	info->maxVcc(currDevice->voltageInfo_.vccMax_);
	info->minFlashVcc(currDevice->voltageInfo_.vccFlashMin_);
	info->minSecureVcc(currDevice->voltageInfo_.vccSecureMin_);
	info->minSecureVpp(currDevice->voltageInfo_.vppSecureMin_);
	info->maxSecureVpp(currDevice->voltageInfo_.vppSecureMax_);
	info->hasTestVpp(currDevice->voltageInfo_.hasTestVpp_);

	info->quickMemRead(currDevice->featuresInfo_.quickMemRead_);
	info->hasFram(currDevice->featuresInfo_.hasFram_);
	info->clockSystem(currDevice->featuresInfo_.clock_);
	info->noBsl(currDevice->featuresInfo_.noBsl_);
	info->psach(currDevice->extFeaturesInfo_.psach_);
	info->b1377(currDevice->extFeaturesInfo_._1377_);

	info->setDescription(currDevice->description_.c_str());

	info->powerTestRegMask(currDevice->powerSettings_.powerTestRegMask_);
	info->testRegEnableLpmx5(currDevice->powerSettings_.testRegEnableLpmx5_);
	info->testRegDisableLpmx5(currDevice->powerSettings_.testRegDisableLpmx5_);

	info->powerTestReg3VMask(currDevice->powerSettings_.powerTestReg3VMask_);
	info->testReg3VEnableLpmx5(currDevice->powerSettings_.testReg3VEnableLpmx5_);
	info->testReg3VDisableLpmx5(currDevice->powerSettings_.testReg3VDisableLpmx5_);

	for (unsigned int i = 0; i < currDevice->getMemorySize(); ++i) 
	{
		const MemoryInfoImpl& memInfo = currDevice->getMemoryAt(i);
		DeviceInfo::memoryInfo* m = new DeviceInfo::memoryInfo;

		m->name.assign(memInfo.name_);
		switch (memInfo.flags_ & 0x300) 
		{
		case DLL430_MEMFLAG_FLASH:
			m->type = DeviceInfo::MEMTYPE_FLASH;
			break;
		case DLL430_MEMFLAG_ROM:
			m->type = DeviceInfo::MEMTYPE_ROM;
			break;
		case DLL430_MEMFLAG_RAM:
			m->type = DeviceInfo::MEMTYPE_RAM;
			break;
		case DLL430_MEMFLAG_REG:
			m->type = DeviceInfo::MEMTYPE_REGISTER;
			break;
		}
		m->bits = memInfo.flags_ & DLL430_MEMFLAG_BITS_MASK;
		if (!m->bits)
			m->bits = bits;

		m->size = memInfo.size_;
		m->offset = memInfo.offset_;
		m->seg_size = memInfo.seg_size_;
		m->banks = memInfo.banks_;
		m->mask = std::vector<uint8_t>(memInfo.memoryMaskImpl_.data_, 
									   memInfo.memoryMaskImpl_.data_ + memInfo.memoryMaskImpl_.size_);
		m->memoryCreatorPtr = memInfo.memoryCreator_;
		m->mmapped = ((memInfo.flags_ & DLL430_MEMFLAG_MAPPED) != 0);
		m->isProtected= memInfo.protected_;
		info->addMemoryInfo(m);
	}
	
	const FunctionMappingImpl::FunctionMapping& fcntMap = currDevice->fnctMap_.GetMap();
	if(!fcntMap.empty()) 
	{
		FunctionMappingImpl::FunctionMapping::const_iterator it = fcntMap.begin();
		for (unsigned int i = 0; it != fcntMap.end(); ++it, ++i) 
		{
			info->addFunctionMapping((*it).first, (*it).second);
		}
	}

	info->setFuncletMap( currDevice->funcletMapping_.getMap() );

	info->setClockControl(currDevice->clockInfo_.clockControl_);
	this->CreateClockModuleNames(info->getWritableClockMapping(), currDevice->clockInfo_.eemTimer_);
	this->CreateClockNames(info->getClockNames(), currDevice->clockInfo_.eemClockNames_);
	info->setClockModDefault(currDevice->clockInfo_.mclkCntrl0_);

	info->setStateStorage(currDevice->eemInfo_.stateStorage_);
	info->setCycleCounter(currDevice->eemInfo_.cycleCounter_);
	info->setCycleCounterOperations(currDevice->eemInfo_.cycleCounterOperations_);
	info->setEmulationLevel(currDevice->eemInfo_.trigger_.emulation_level_);
	info->setSFll(currDevice->featuresInfo_.sflldh_);
	info->setTriggerMask(currDevice->eemInfo_.trigger_.mem_umaskLevel_);

	info->setPossibleTrigger(currDevice->eemInfo_.trigger_.mem_,0);			/**< number of memory triggers */
	info->setPossibleTrigger(currDevice->eemInfo_.trigger_.reg_,1);			/**< number of register-write triggers */
	info->setPossibleTrigger(currDevice->eemInfo_.trigger_.combinations_,2);/**< number of trigger combinations (trigger groups) */ 
	
	info->setTriggerOptionsModes(currDevice->eemInfo_.trigger_.options_);
	info->setTriggerDmaModes(currDevice->eemInfo_.trigger_.dma_);
	info->setTriggerReadWriteModes(currDevice->eemInfo_.trigger_.readwrite_);

	info->setRegTriggerOperations(currDevice->eemInfo_.trigger_.regOperations_);

	info->setMaxSequencerStates(currDevice->eemInfo_.sequencer_.states_);

	return info;
}

void Registration::CreateClockModuleNames(DeviceInfo::ClockMapping& clockMapping, const EemTimerImpl& eemTimer) const
{
	assert(clockModulesSize == clockMapping.size());

	clockMapping[0].first = eemTimer._0_.name_;
	clockMapping[0].second = eemTimer._0_.value_;
	clockMapping[1].first = eemTimer._1_.name_;
	clockMapping[1].second = eemTimer._1_.value_;
	clockMapping[2].first = eemTimer._2_.name_;
	clockMapping[2].second = eemTimer._2_.value_;
	clockMapping[3].first = eemTimer._3_.name_;
	clockMapping[3].second = eemTimer._3_.value_;
	clockMapping[4].first = eemTimer._4_.name_;
	clockMapping[4].second = eemTimer._4_.value_;
	clockMapping[5].first = eemTimer._5_.name_;
	clockMapping[5].second = eemTimer._5_.value_;
	clockMapping[6].first = eemTimer._6_.name_;
	clockMapping[6].second = eemTimer._6_.value_;
	clockMapping[7].first = eemTimer._7_.name_;
	clockMapping[7].second = eemTimer._7_.value_;
	clockMapping[8].first = eemTimer._8_.name_;
	clockMapping[8].second = eemTimer._8_.value_;
	clockMapping[9].first = eemTimer._9_.name_;
	clockMapping[9].second = eemTimer._9_.value_;
	clockMapping[10].first = eemTimer._10_.name_;
	clockMapping[10].second = eemTimer._10_.value_;
	clockMapping[11].first = eemTimer._11_.name_;
	clockMapping[11].second = eemTimer._11_.value_;
	clockMapping[12].first = eemTimer._12_.name_;
	clockMapping[12].second = eemTimer._12_.value_;
	clockMapping[13].first = eemTimer._13_.name_;
	clockMapping[13].second = eemTimer._13_.value_;
	clockMapping[14].first = eemTimer._14_.name_;
	clockMapping[14].second = eemTimer._14_.value_;
	clockMapping[15].first = eemTimer._15_.name_;
	clockMapping[15].second = eemTimer._15_.value_;
	clockMapping[16].first = eemTimer._16_.name_;
	clockMapping[16].second = eemTimer._16_.value_;
	clockMapping[17].first = eemTimer._17_.name_;
	clockMapping[17].second = eemTimer._17_.value_;
	clockMapping[18].first = eemTimer._18_.name_;
	clockMapping[18].second = eemTimer._18_.value_;
	clockMapping[19].first = eemTimer._19_.name_;
	clockMapping[19].second = eemTimer._19_.value_;
	clockMapping[20].first = eemTimer._20_.name_;
	clockMapping[20].second = eemTimer._20_.value_;
	clockMapping[21].first = eemTimer._21_.name_;
	clockMapping[21].second = eemTimer._21_.value_;
	clockMapping[22].first = eemTimer._22_.name_;
	clockMapping[22].second = eemTimer._22_.value_;
	clockMapping[23].first = eemTimer._23_.name_;
	clockMapping[23].second = eemTimer._23_.value_;
	clockMapping[24].first = eemTimer._24_.name_;
	clockMapping[24].second = eemTimer._24_.value_;
	clockMapping[25].first = eemTimer._25_.name_;
	clockMapping[25].second = eemTimer._25_.value_;
	clockMapping[26].first = eemTimer._26_.name_;
	clockMapping[26].second = eemTimer._26_.value_;
	clockMapping[27].first = eemTimer._27_.name_;
	clockMapping[27].second = eemTimer._27_.value_;
	clockMapping[28].first = eemTimer._28_.name_;
	clockMapping[28].second = eemTimer._28_.value_;
	clockMapping[29].first = eemTimer._29_.name_;
	clockMapping[29].second = eemTimer._29_.value_;
	clockMapping[30].first = eemTimer._30_.name_;
	clockMapping[30].second = eemTimer._30_.value_;
	clockMapping[31].first = eemTimer._31_.name_;
	clockMapping[31].second = eemTimer._31_.value_;
}


void Registration::CreateClockNames(DeviceInfo::ClockNames& clockNames, const EemClocksImpl& eemClocks) const
{
	clockNames[0] = eemClocks._0_;
	clockNames[1] = eemClocks._1_;
	clockNames[2] = eemClocks._2_;
	clockNames[3] = eemClocks._3_;
	clockNames[4] = eemClocks._4_;
	clockNames[5] = eemClocks._5_;
	clockNames[6] = eemClocks._6_;
	clockNames[7] = eemClocks._7_;
	clockNames[8] = eemClocks._8_;
	clockNames[9] = eemClocks._9_;
	clockNames[10] = eemClocks._10_;
	clockNames[11] = eemClocks._11_;
	clockNames[12] = eemClocks._12_;
	clockNames[13] = eemClocks._13_;
	clockNames[14] = eemClocks._14_;
	clockNames[15] = eemClocks._15_;
}
