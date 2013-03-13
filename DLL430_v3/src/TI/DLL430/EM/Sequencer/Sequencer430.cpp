/*
 * Sequencer430.cpp
 *
 * Sequencer implementation for 430
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



#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>

#include "Sequencer430.h"
#include "../TriggerManager/TriggerManager430.h"
#include "../EemRegisters/EemRegisterAccess.h"
#include "../Exceptions/Exceptions.h"


using namespace TI::DLL430;


namespace {
	enum CNTRL_BITS { SEQ_ENABLE = 0x1, SEQ_RESET_EN = 0x2, SEQ_LEVEL_SENSITIVE = 0x4, SEQ_RESET = 0x40 };

	enum REGISTERS { SEQ_NXTSTATE0 = 0xA0, SEQ_NXTSTATE1 = 0xA2, SEQ_CTL = 0xA6 };
}

Sequencer430::Sequencer430(TriggerManager430Ptr triggerManager, bool sixTriggerEem)
	: triggerManager_(triggerManager)
	, states_(NUM_STATES)
	, seqCntrl_(0)
	, nxtState0_(0)
	, nxtState1_(0)
	, sixTriggerEem_(sixTriggerEem)
{
}


Sequencer430::~Sequencer430()
{
}


void Sequencer430::initialize()
{
}


void Sequencer430::cleanup()
{
}


void Sequencer430::writeConfiguration()
{
	const uint8_t TRANS_A = 0, TRANS_B = 1;

	nxtState0_ = 0;
	nxtState1_ = 0;

	for (uint32_t i = 0; i < NUM_STATES * NUM_TRANSITIONS; ++i)
	{
		const uint32_t state = i / NUM_TRANSITIONS;
		const uint32_t trans = i % NUM_TRANSITIONS;
		const uint32_t offset = (i < 4) ? (i*4) : (i-4)*4;
		uint16_t& reg = (i < 4) ? nxtState0_ : nxtState1_;
		
		reg |= states_[state].nextState[trans] << offset;

		if (states_[state].trigger[trans])
		{
			const uint32_t combinationTriggerId = states_[state].trigger[trans]->getId();

			if (combinationTriggerId == 0xFF)
				throw EM_SequencerException();

			const uint8_t triggerBits = (combinationTriggerId - (sixTriggerEem_ ? 2 : 4)) & 0x3;
			reg |= triggerBits << (offset + 2);
		}
	}

	const bool forceCntrlRegisterWrite = (seqCntrl_ & SEQ_RESET) != 0;

	seqCntrl_ &= ~SEQ_RESET;

	writeEemRegister(SEQ_NXTSTATE0, nxtState0_);
	writeEemRegister(SEQ_NXTSTATE1, nxtState1_);
	writeEemRegister(SEQ_CTL, seqCntrl_, forceCntrlRegisterWrite);
}


uint32_t Sequencer430::getMaxStates() const 
{
	return NUM_STATES;
}


uint32_t Sequencer430::getMaxTransitions() const 
{
	return NUM_TRANSITIONS;
}


uint32_t Sequencer430::readCurrentState() const
{
	const uint32_t cntrlRegister = readEemRegister(SEQ_CTL);
	return (cntrlRegister >> 8) & 0x3;
}


void Sequencer430::setResetTrigger(TriggerConditionPtr triggerCondition)
{
	resetTrigger_ = triggerCondition;

	if (resetTrigger_ && (seqCntrl_ & SEQ_ENABLE))
	{
		resetTrigger_->addReaction(TR_SEQUENCER_RESET);
	}

	seqCntrl_ |= SEQ_RESET_EN;
}


void Sequencer430::clearResetTrigger()
{
	resetTrigger_.reset();

	seqCntrl_ &= ~SEQ_RESET_EN;
}


void Sequencer430::setTransition(uint32_t state, uint32_t transition, uint32_t nextState, TriggerConditionPtr triggerCondition)
{
	if (state >= NUM_STATES || nextState >= NUM_STATES || transition >= NUM_TRANSITIONS)
		throw EM_SequencerException();

	if (states_[state].trigger[transition])
	{
		states_[state].trigger[transition]->removeReaction(TR_SEQUENCER);
	}

	if (triggerCondition && (seqCntrl_ & SEQ_ENABLE))
	{
		triggerCondition->addReaction(TR_SEQUENCER);
	}

	states_[state].trigger[transition] = triggerCondition;
	states_[state].nextState[transition] = nextState;
}


void Sequencer430::clearTransition(uint32_t state, uint32_t transition)
{
	if (state >= NUM_STATES || transition >= NUM_TRANSITIONS)
		throw EM_SequencerException();

	if (states_[state].trigger[transition])
	{
		states_[state].trigger[transition]->removeReaction(TR_SEQUENCER);
		states_[state].trigger[transition].reset();
	}

	states_[state].nextState[transition] = 0;
}


void Sequencer430::clearAllTransitions()
{
	for (int state = 0; state < NUM_STATES; ++state)
	{
		for (int trans = 0; trans < NUM_TRANSITIONS; ++trans)
		{
			clearTransition(state, trans);
		}
	}
}





void Sequencer430::reset() 
{
	seqCntrl_ |= SEQ_RESET;
}


void Sequencer430::setTriggerMode(SequencerTriggerMode mode) 
{
	if (mode == STM_LEVEL_TRIGGER)
	{
		seqCntrl_ |= SEQ_LEVEL_SENSITIVE;
	}
	else
	{
		seqCntrl_ &= ~SEQ_LEVEL_SENSITIVE;
	}
}


void Sequencer430::addReaction(TriggerReaction reaction)
{
	triggerManager_->addSequencerOutReaction(reaction);
}


void Sequencer430::removeReaction(TriggerReaction reaction)
{
	triggerManager_->removeSequencerOutReaction(reaction);
}


void Sequencer430::clearReactions()
{
	triggerManager_->clearSequencerOutReactions();
}


void Sequencer430::enable()
{
	seqCntrl_ |= SEQ_ENABLE;

	triggerManager_->enableSequencerOutReactions();

	BOOST_FOREACH(State& state, states_)
	{
		if (state.trigger[0])
			state.trigger[0]->addReaction(TR_SEQUENCER);

		if (state.trigger[1])
			state.trigger[1]->addReaction(TR_SEQUENCER);
	}

	if (resetTrigger_)
	{
		resetTrigger_->addReaction(TR_SEQUENCER_RESET);
	}
}

void Sequencer430::disable()
{
	seqCntrl_ &= ~SEQ_ENABLE;

	triggerManager_->disableSequencerOutReactions();

	BOOST_FOREACH(State& state, states_)
	{
		if (state.trigger[0])
			state.trigger[0]->removeReaction(TR_SEQUENCER);

		if (state.trigger[1])
			state.trigger[1]->removeReaction(TR_SEQUENCER);
	}

	if (resetTrigger_)
	{
		resetTrigger_->removeReaction(TR_SEQUENCER_RESET);
	}
}

bool Sequencer430::isEnabled() const
{
	return (seqCntrl_ & SEQ_ENABLE) != 0;
}
