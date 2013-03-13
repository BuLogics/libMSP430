/*
 * TriggerCondition430.cpp
 *
 * Common implementation of trigger conditions for 430
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


#include <list>
#include <boost/foreach.hpp>

#include "TriggerCondition430.h"
#include "../Trigger/Trigger430.h"
#include "../TriggerManager/TriggerManager430.h"

using namespace TI::DLL430;


TriggerCondition430::TriggerCondition430(TriggerManager430Ptr triggerManager)
	: triggerManager_(triggerManager)
{
}


TriggerCondition430::~TriggerCondition430()
{
	while (!triggers_.empty())
	{
		triggerManager_->releaseTrigger(triggers_.front());
		triggers_.pop_front();
	}
}


void TriggerCondition430::addReaction(TriggerReaction reaction)
{
	BOOST_FOREACH(Trigger430* trigger, triggers_)
	{
		if (!trigger->isCombinationTrigger())
		{
			trigger->addReaction(reaction);
		}
	}
}


void TriggerCondition430::removeReaction(TriggerReaction reaction)
{
	BOOST_FOREACH(Trigger430* trigger, triggers_)
	{
		trigger->removeReaction(reaction);
	}
}


void TriggerCondition430::combine(TriggerConditionPtr condition)
{
	if (TriggerCondition430* otherCondition = dynamic_cast<TriggerCondition430*>(condition.get()))
	{
		if (!triggers_.empty())
		{
			BOOST_FOREACH(Trigger430* trigger, otherCondition->triggers_)
			{
				triggers_.front()->combineWith(trigger);
				triggers_.push_back(trigger);
			}
		}
		otherCondition->triggers_.clear();
	}
}


uint32_t TriggerCondition430::getId() const
{
	const Trigger430* mainTrigger = triggers_.empty() ? 0 : triggers_.front();
	return triggerManager_->getCombinationTrigger(mainTrigger);
}


void TriggerCondition430::enable()
{
	BOOST_FOREACH(Trigger430* trigger, triggers_)
	{
		trigger->isEnabled(true);
	}
}


void TriggerCondition430::disable()
{
	BOOST_FOREACH(Trigger430* trigger, triggers_)
	{
		trigger->isEnabled(false);
	}
}
