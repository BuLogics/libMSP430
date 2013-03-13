/*
 * DeviceInfo.cpp
 *
 * Data of device currently under control.
 *
 * Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include "DeviceInfo.h"

using namespace TI::DLL430;


DeviceInfo::DeviceInfo ()
 : objectId(0)
 , type(DeviceInfo::PSATYPE_REGULAR)
 , hasTestVpp(false)
 , description(NULL)
 , clockControl(0)
 , mclkcntrl0(0)
 , stateStorage(0)
 , cycleCounter(0)
 , cycleCounterOperations(0)
 , emulationLevel(0)
 , triggerMask(0)
 , triggerOptionsModes(0)
 , triggerDmaModes(0)
 , triggerReadWriteModes(0)
 , regTriggerOperations(0)
 , maxSequencerStates(0)
 , sFll(0)
{
	memset(possibleTrigger, 0, sizeof(possibleTrigger));	
}

DeviceInfo::~DeviceInfo ()
{
}

void DeviceInfo::setDescription(const char * dev_decription)
{
	description=dev_decription;
}

const char * DeviceInfo::getDescription() const
{
	return description;
}

void DeviceInfo::addMemoryInfo (memoryInfo* info)
{
	this->mem.push_back(info);
}

const DeviceInfo::memoryInfo_list_type& DeviceInfo::getMemoryInfo () const
{
	return this->mem;
}

void DeviceInfo::setObjectId (size_t id)
{
	this->objectId = id;
}

size_t DeviceInfo::getObjectId () const
{
	return this->objectId;
}

void DeviceInfo::setPsaType (enum DeviceInfo::psaType type)
{
	this->type = type;
}

enum DeviceInfo::psaType DeviceInfo::getPsaType () const
{
	return this->type;
}

void DeviceInfo::addFunctionMapping (unsigned long apiId, uint16_t halId)
{
	this->map[apiId] = halId;
}

const IoChannel::function_map_type& DeviceInfo::getMap () const
{
	return this->map;
}

void DeviceInfo::setFuncletMap(const funclet_map_type& map)
{
	this->funcletTable = map;
}

const DeviceInfo::funclet_map_type& DeviceInfo::getFuncletMap() const
{
	return this->funcletTable;
}

void DeviceInfo::setPossibleTrigger (uint8_t triggerSize, uint8_t triggerType)
{
	if (triggerType <= 2)
		possibleTrigger[triggerType]=triggerSize;
}

uint8_t DeviceInfo::getPossibleTrigger (uint8_t triggerType) const
{
	if (triggerType <= 2)
		return possibleTrigger[triggerType];
	return 0;
}

void DeviceInfo::setClockControl (uint8_t clock)
{
	this->clockControl=clock;
}

uint8_t DeviceInfo::getClockControl () const
{
	return this->clockControl;
}

void DeviceInfo::setClockModDefault (uint16_t mclkcntrl0)
{
	this->mclkcntrl0=mclkcntrl0;
}

uint16_t DeviceInfo::getClockModDefault () const
{
	return this->mclkcntrl0;
}

void DeviceInfo::setStateStorage (uint8_t state)
{
	this->stateStorage=state;
}

uint8_t DeviceInfo::getStateStorage () const
{
	return this->stateStorage;
}

void DeviceInfo::setCycleCounter (uint8_t cycle)
{
	this->cycleCounter=cycle;
}

uint8_t DeviceInfo::getCycleCounter () const
{
	return this->cycleCounter;
}

void DeviceInfo::setCycleCounterOperations (uint8_t cycleOps)
{
	this->cycleCounterOperations=cycleOps;
}

uint8_t DeviceInfo::getCycleCounterOperations () const
{
	return this->cycleCounterOperations;
}

void DeviceInfo::setEmulationLevel(uint8_t level)
{
	emulationLevel=level;
}

uint8_t DeviceInfo::getEmulationLevel () const
{
	return this->emulationLevel;
}

void DeviceInfo::setTriggerOptionsModes(uint8_t optModes)
{
	triggerOptionsModes = optModes;
}

uint8_t DeviceInfo::getTriggerOptionsModes() const
{
	return this->triggerOptionsModes;
}

void DeviceInfo::setTriggerDmaModes(uint8_t dmaModes)
{
	triggerDmaModes = dmaModes;
}

uint8_t DeviceInfo::getTriggerDmaModes() const
{
	return this->triggerDmaModes;
}

void DeviceInfo::setTriggerReadWriteModes(uint8_t rwModes)
{
	triggerReadWriteModes = rwModes;
}

uint8_t DeviceInfo::getTriggerReadWriteModes() const
{
	return this->triggerReadWriteModes;
}

void DeviceInfo::setRegTriggerOperations(uint8_t regTrigOps)
{
	regTriggerOperations = regTrigOps;
}

uint8_t DeviceInfo::getRegTriggerOperations() const
{
	return this->regTriggerOperations;
}

void DeviceInfo::setMaxSequencerStates(uint8_t states)
{
	maxSequencerStates = states;
}

uint8_t DeviceInfo::getMaxSequencerStates() const
{
	return this->maxSequencerStates;
}

void DeviceInfo::setTriggerMask(uint8_t mask)
{
	triggerMask = mask;
}

uint8_t DeviceInfo::getTriggerMask() const
{
	return this->triggerMask;
}

void DeviceInfo::setSFll (uint8_t Fll)
{
	this->sFll=Fll;
}

uint8_t DeviceInfo::getSFll () const
{
	return this->sFll;
}
