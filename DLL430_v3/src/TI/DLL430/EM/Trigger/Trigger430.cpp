/*
 * Trigger430.cpp
 *
 * Common implementation for triggers on 430
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


#include <map>
#include <boost/foreach.hpp>

#include "Trigger430.h"
#include "../EemRegisters/EemRegisterAccess.h"
#include "../Exceptions/Exceptions.h"

using namespace std;
using namespace TI::DLL430;



namespace 
{
	enum RegisterOffset
	{
		VALUE_REGISTER = 0, 
		CONTROL_REGISTER = 2, 
		MASK_REGISTER = 4,
		COMBINATION_REGISTER = 6,
		TRIGGER_BLOCK_SIZE = 8
	};
}


map<ComparisonOperation, uint16_t> Trigger430::comparisonOpBits;

map<AccessType, uint16_t> Trigger430::accessTypeBits;

bool Trigger430::bitwiseMasks;




//Default is MAB, ==, Instruction Fetch, compare all bits
Trigger430::Trigger430(TYPE type, uint32_t id)
	: type_(type)
	, triggerValueRegister_(0)
	, triggerControlRegister_(0)
	, triggerMaskRegister_(0)
	, id_(id)
	, isInUse_(false)
	, isEnabled_(true)
	, isCombinationTrigger_(false)
{
}


uint32_t Trigger430::getId() const 
{
	return id_;
}



void Trigger430::combineWith(Trigger430* trigger)
{
	combinedWith_.insert(trigger);
	trigger->isCombinationTrigger(true);
}


void Trigger430::uncombineWith(Trigger430* trigger)
{
	combinedWith_.erase(trigger);
	trigger->isCombinationTrigger(false);
}


uint32_t Trigger430::getCombinationValue() const
{
	uint32_t combinationValue = (1 << getId());
	BOOST_FOREACH(const Trigger430* trigger, combinedWith_)
	{
		combinationValue |= (1 << trigger->getId());
	}
	return combinationValue;
}


void Trigger430::swapTriggerBlock(Trigger430& trigger)
{
	if (type_ == trigger.type_)
	{
		swap(id_, trigger.id_);
	}
}


void Trigger430::read()
{
}


void Trigger430::write()
{
	writeEemRegister(valueRegisterAddress(), triggerValueRegister_);
	writeEemRegister(controlRegisterAddress(), triggerControlRegister_);
	writeEemRegister(maskRegisterAddress(), triggerMaskRegister_);
}


void Trigger430::reset()
{
	triggerValueRegister_ = 0;
	triggerControlRegister_ = 0;
	triggerMaskRegister_ = 0;

	isInUse_ = false;
	isEnabled_ = true;
	isCombinationTrigger_ = false;
	
	reactions_.clear();
	combinedWith_.clear();
}


void Trigger430::addReaction(TriggerReaction reaction)
{
	reactions_.insert(reaction);
}

void Trigger430::removeReaction(TriggerReaction reaction)
{
	reactions_.erase(reaction);
}

const std::set<TriggerReaction>& Trigger430::getReactions() const 
{
	return reactions_;
}


void Trigger430::setComparisonOperation(ComparisonOperation op)
{
	if (comparisonOpBits.find(op) == comparisonOpBits.end())
		throw EM_TriggerParameterException();

	triggerControlRegister_ &= ~0x18;
	triggerControlRegister_ |= comparisonOpBits[op];
}


void Trigger430::setValue(uint32_t value)
{
	triggerValueRegister_ = value;
}


void Trigger430::setMask(uint32_t mask)
{
	if (!bitwiseMasks)
	{
		const uint32_t maskLow  = mask & 0x000FF;
		const uint32_t maskHigh = mask & 0x0FF00;
		const uint32_t maskX    = mask & 0xF0000;

		if ((maskLow  != 0 && maskLow  != 0x000FF) || 
			(maskHigh != 0 && maskHigh != 0x0FF00) || 
			(maskX    != 0 && maskX    != 0xF0000))
		{
			throw EM_TriggerParameterException();
		}
	}

	triggerMaskRegister_ = ~mask;
}



bool Trigger430::isCombinationTrigger() const
{
	return isCombinationTrigger_;
}

void Trigger430::isCombinationTrigger(bool isInCombination)
{
	isCombinationTrigger_ = isInCombination;
}



bool Trigger430::isInUse() const
{
	return isInUse_;
}

void Trigger430::isInUse(bool inUse)
{
	isInUse_ = inUse;
}



bool Trigger430::isEnabled() const
{
	return isEnabled_;
}

void Trigger430::isEnabled(bool enabled)
{
	isEnabled_ = enabled;
}



uint32_t Trigger430::valueRegisterAddress() const
{
	return (getId() * TRIGGER_BLOCK_SIZE) + VALUE_REGISTER;
}


uint32_t Trigger430::controlRegisterAddress() const
{
	return (getId() * TRIGGER_BLOCK_SIZE) + CONTROL_REGISTER;
}


uint32_t Trigger430::maskRegisterAddress() const
{
	return (getId() * TRIGGER_BLOCK_SIZE) + MASK_REGISTER;
}




void Trigger430::setMemoryAddressBus()
{
	if (type_ != BUS_TRIGGER)
		throw EM_TriggerParameterException();

	triggerControlRegister_ &= ~0x1;
}


void Trigger430::setMemoryDataBus()
{
	if (type_ != BUS_TRIGGER)
		throw EM_TriggerParameterException();

	triggerControlRegister_ |= 0x1;
}


void Trigger430::setAccessType(AccessType accessType)
{
	if ((type_ != BUS_TRIGGER) || (accessTypeBits.find(accessType) == accessTypeBits.end()))
		throw EM_TriggerParameterException();

	triggerControlRegister_ &= ~0x66;
	triggerControlRegister_ |= accessTypeBits[accessType];	
}




void Trigger430::setRegister(uint32_t reg)
{
	if (type_ != REGISTER_TRIGGER)
		throw EM_TriggerParameterException();
	
	triggerControlRegister_ &= ~(0xf << 8);
	triggerControlRegister_ |= (reg & 0xf) << 8;
}

void Trigger430::setHold(bool hold)
{
	if (type_ != REGISTER_TRIGGER)
		throw EM_TriggerParameterException();

	if (hold)
	{
		triggerControlRegister_ |= (1 << 6);
	}
	else
	{
		triggerControlRegister_ &= ~(1 << 6);
	}
}
