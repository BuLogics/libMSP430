/*
 * TriggerManager430.cpp
 *
 * Handles trigger resources on 430
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



#include <boost/foreach.hpp>
#include <vector>


#include "../Trigger/Trigger430.h"

#include "TriggerManager430.h"
#include "TriggerConfigurator430.h"

#include "../EemRegisters/EemRegisterAccess.h"
#include "../Exceptions/Exceptions.h"

using namespace TI::DLL430;


namespace {
	enum ReactionRegisters { BREAK_REACT = 0x80, CCNT1_REACT = 0xBE, STOR_REACT = 0x98 };
}

TriggerManager430::TriggerManager430(int numBusTriggers, int numRegisterTriggers, int numCombinationTriggers, int sequencerOutId)
	: mSequencerOutTriggerId(sequencerOutId)
	, mSequencerOutReactionsEnabled(false)
{
	uint32_t id = 0;

	for (int busTrigger = 0; busTrigger < numBusTriggers; ++busTrigger)
	{
		mBusTriggers.push_back( Trigger430(Trigger430::BUS_TRIGGER, id++) );
	}

	for (int registerTrigger = 0; registerTrigger < numRegisterTriggers; ++registerTrigger)
	{
		mRegisterTriggers.push_back( Trigger430(Trigger430::REGISTER_TRIGGER, id++) );
	}

	mCombinationTriggers.resize(numCombinationTriggers, NULL);


	//Configure always available options
	Trigger430::comparisonOpBits.clear();
	Trigger430::comparisonOpBits[CO_EQUAL] = 0x0;
	Trigger430::comparisonOpBits[CO_NOT_EQUAL] = 0x18;

	Trigger430::accessTypeBits.clear();
	Trigger430::accessTypeBits[AT_FETCH] = 0x0;
	Trigger430::accessTypeBits[AT_FETCH_HOLD] = 0x2;
	Trigger430::accessTypeBits[AT_NO_FETCH] = 0x4;
	Trigger430::accessTypeBits[AT_DONT_CARE] = 0x6;

	Trigger430::bitwiseMasks = false;
}


void TriggerManager430::setExtendedComparisons() const
{
	Trigger430::comparisonOpBits[CO_GREATER_EQUAL] = 0x8;
	Trigger430::comparisonOpBits[CO_LESS_EQUAL] = 0x10;
}


void TriggerManager430::setExtendedAccessTypes() const
{
	Trigger430::accessTypeBits[AT_NO_FETCH_READ] = 0x20;
	Trigger430::accessTypeBits[AT_NO_FETCH_WRITE] = 0x22;
	Trigger430::accessTypeBits[AT_READ] = 0x24;
	Trigger430::accessTypeBits[AT_WRITE] = 0x26;
	Trigger430::accessTypeBits[AT_NO_FETCH_NO_DMA] = 0x40;
	Trigger430::accessTypeBits[AT_DMA] = 0x42;
	Trigger430::accessTypeBits[AT_NO_DMA] = 0x44;
	Trigger430::accessTypeBits[AT_WRITE_NO_DMA] = 0x46;
	Trigger430::accessTypeBits[AT_NO_FETCH_READ_NO_DMA] = 0x60;
	Trigger430::accessTypeBits[AT_READ_NO_DMA] = 0x62;
	Trigger430::accessTypeBits[AT_READ_DMA] = 0x64;
	Trigger430::accessTypeBits[AT_WRITE_DMA] = 0x66;
}


void TriggerManager430::setBitwiseMasking() const
{
	Trigger430::bitwiseMasks = true;
}


Trigger430* TriggerManager430::getBusTrigger() 
{
	BOOST_FOREACH(Trigger430& trigger, mBusTriggers)
	{
		if (!trigger.isInUse())
		{
			trigger.isInUse(true);
			return &trigger;
		}
	}
	return NULL;
}


Trigger430* TriggerManager430::getRegisterTrigger() 
{
	BOOST_FOREACH(Trigger430& trigger, mRegisterTriggers)
	{
		if (!trigger.isInUse())
		{
			trigger.isInUse(true);
			return &trigger;
		}
	}
	return NULL;
}


void TriggerManager430::releaseTrigger(Trigger430* trigger)
{
	trigger->reset();
}


uint32_t TriggerManager430::getCombinationTrigger(const Trigger430* trigger) const
{
	if (trigger != NULL)
	{
		for (uint32_t id = 0; id < (uint32_t)mCombinationTriggers.size(); ++id)
		{
			if (mCombinationTriggers[id] == trigger)
			{
				return id;
			}
		}
	}
	return 0xFF;
}


bool TriggerManager430::hasRegisterTriggers() const
{
	return !mRegisterTriggers.empty();
}

int TriggerManager430::numAvailableBusTriggers() const 
{
	int count = 0;

	BOOST_FOREACH(const Trigger430& trigger, mBusTriggers)
	{
		if (!trigger.isInUse())
		{
			++count;
		}
	}
	return count;
}


int TriggerManager430::numAvailableRegisterTriggers() const 
{
	int count = 0;

	BOOST_FOREACH(const Trigger430& trigger, mRegisterTriggers)
	{
		if (!trigger.isInUse())
		{
			++count;
		}
	}
	return count;
}


void TriggerManager430::configureTriggers(bool sequencerEnabled) 
{
	//Collect all active triggers with reactions, that aren't combination triggers
	std::deque<const Trigger430*> triggers;

	BOOST_FOREACH(const Trigger430& trigger, mBusTriggers)
	{
		if (trigger.isInUse() && trigger.isEnabled() && !trigger.isCombinationTrigger() && !trigger.getReactions().empty())
		{
			triggers.push_back(&trigger);
		}
	}

	BOOST_FOREACH(const Trigger430& trigger, mRegisterTriggers)
	{
		if (trigger.isInUse() && trigger.isEnabled() && !trigger.isCombinationTrigger() && !trigger.getReactions().empty())
		{
			triggers.push_back(&trigger);
		}
	}


	verifyForSingleStepping(triggers);


	TriggerConfigurator430 configurator(triggers, mCombinationTriggers, sequencerEnabled);
	if (!configurator.checkTriggerConfiguration())
	{
		if (!configurator.configureTriggerConfiguration())
			throw EM_TriggerConfigurationException();
	}
}


void TriggerManager430::verifyForSingleStepping(const std::deque<const Trigger430*> usedTriggers)
{
	std::map<uint32_t, std::set<TriggerReaction> > reactionsByTriggerBlock;
	BOOST_FOREACH(const Trigger430* trigger, usedTriggers)
	{
		uint32_t triggerBlocks = trigger->getCombinationValue();
		for (size_t i = 0; i < mBusTriggers.size(); ++i)
		{
			if (triggerBlocks & (1<<i))
			{
				std::set<TriggerReaction>& reactions = reactionsByTriggerBlock[i];
				const std::set<TriggerReaction> newReactions = trigger->getReactions();
				reactions.insert(newReactions.begin(), newReactions.end());
			}
		}
	}

	uint32_t firstValidForBlock0 = 0;
	for (uint32_t triggerId = 0; triggerId < mBusTriggers.size(); ++triggerId)
	{
		const std::set<TriggerReaction>& reactions = reactionsByTriggerBlock[triggerId];

		if (reactions.empty() || (reactions.size() == 1 && reactions.count(TR_BREAK) == 1))
		{
			firstValidForBlock0 = triggerId;
			break;
		}
	}
		
	if (firstValidForBlock0 > 0)
	{
		Trigger430* triggerAt0 = getTriggerAtBlock(0);

		Trigger430* validTrigger = getTriggerAtBlock(firstValidForBlock0);

		if (triggerAt0 == NULL || validTrigger == NULL)
			throw EM_TriggerConfigurationException();

		triggerAt0->swapTriggerBlock(*validTrigger);
	}
}


Trigger430* TriggerManager430::getTriggerAtBlock(uint32_t blockId)
{
	Trigger430* trigger = 0;				
	BOOST_FOREACH(Trigger430& trigger0, mBusTriggers)
	{
		if (trigger0.getId() == blockId)
		{
			trigger = &trigger0;
			break;
		}
	}
	return trigger;
}


void TriggerManager430::writeAllTriggers()
{
	BOOST_FOREACH(Trigger430& trigger, mBusTriggers)
	{
		trigger.write();
	}
	BOOST_FOREACH(Trigger430& trigger, mRegisterTriggers)
	{
		trigger.write();
	}
}


template<typename Container, typename Value>
bool contains(const Container& c, const Value& v)
{
	return c.find(v) != c.end();
}


void TriggerManager430::writeTriggerReactions()
{
	uint16_t breakReactionRegister = 0;
	uint16_t cycleCounterReactionRegister = 0;
	uint16_t stateStorageReactionRegister = 0;

	for (uint32_t combinationTriggerId = 0; combinationTriggerId < (uint32_t)mCombinationTriggers.size(); ++combinationTriggerId) 
	{
		const Trigger430* trigger = mCombinationTriggers[combinationTriggerId];

		writeEemRegister((combinationTriggerId * 8) + 6, trigger ? trigger->getCombinationValue() : 0);

		if (trigger != NULL)
		{
			const std::set<TriggerReaction> reactions = trigger->getReactions();
		
			if (contains(reactions, TR_BREAK))
			{
				breakReactionRegister |= (1 << combinationTriggerId);
			}

			if (contains(reactions, TR_CYCLE_COUNTER))
			{
				cycleCounterReactionRegister |= (1 << combinationTriggerId);
			}

			if (contains(reactions, TR_STATE_STORAGE))
			{
				stateStorageReactionRegister |= (1 << combinationTriggerId);
			}

			if (contains(reactions, TR_VARIABLE_WATCH))
			{
				stateStorageReactionRegister |= (1 << combinationTriggerId);
			}
		}
	}

	//Set all reactions for the sequencer
	if (mSequencerOutReactionsEnabled)
	{
		if (contains(mSequencerReactions, TR_BREAK))
		{
			breakReactionRegister |= (1 << mSequencerOutTriggerId);
		}
		if (contains(mSequencerReactions, TR_CYCLE_COUNTER))
		{
			cycleCounterReactionRegister |= (1 << mSequencerOutTriggerId);
		}
		if (contains(mSequencerReactions, TR_STATE_STORAGE))
		{
			stateStorageReactionRegister |= (1 << mSequencerOutTriggerId);
		}
		if (contains(mSequencerReactions, TR_VARIABLE_WATCH))
		{
			stateStorageReactionRegister |= (1 << mSequencerOutTriggerId);
		}
	}


	writeEemRegister(BREAK_REACT, breakReactionRegister);		
	writeEemRegister(CCNT1_REACT, cycleCounterReactionRegister);
	writeEemRegister(STOR_REACT, stateStorageReactionRegister);
}



void TriggerManager430::enableSequencerOutReactions()
{
	mSequencerOutReactionsEnabled = true;
}


void TriggerManager430::disableSequencerOutReactions()
{
	mSequencerOutReactionsEnabled = false;
}


void TriggerManager430::addSequencerOutReaction(TriggerReaction reaction)
{
	mSequencerReactions.insert(reaction);
}


void TriggerManager430::removeSequencerOutReaction(TriggerReaction reaction)
{
	mSequencerReactions.erase(reaction);
}

void TriggerManager430::clearSequencerOutReactions()
{
	mSequencerReactions.clear();
}
