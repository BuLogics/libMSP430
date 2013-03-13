/*
 * Trace430.cpp
 *
 * Trace implementation for 430
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

#include "../TriggerCondition/ITriggerCondition.h"
#include "../StateStorage430/StateStorage430.h"
#include "../Exceptions/Exceptions.h"
#include "../../MessageData.h"

using namespace TI::DLL430;


void StateStorage430::onEventTrace(MessageDataPtr msgData)
{
	msgData->reset();

	uint16_t eventMask = 0;
	(*msgData) >> eventMask;
	
	if (eventMask & 0x2)
	{
		uint16_t newEntries = 0;
		(*msgData) >> newEntries;

		boost::mutex::scoped_lock lock(traceBufferMutex_);

		traceBuffer_.clear();
		do
		{
			TraceData data;
			(*msgData) >> data.mab >> data.mdb >> data.ctl;

			if (!msgData->fail())
			{
				traceBuffer_.push_back(data);
			}
		} while (!msgData->fail());
	}
}


void StateStorage430::addTriggerCondition(TriggerConditionPtr triggerCondition) 
{
	if (triggerCondition)
	{
		triggerConditions_.push_back(triggerCondition);
	}
	if (controlRegister_ & STOR_EN)
	{
		triggerCondition->addReaction(TR_STATE_STORAGE);
	}
}


void StateStorage430::clearTriggerConditions()
{
	BOOST_FOREACH(TriggerConditionPtr& condition, triggerConditions_)
	{
		condition->removeReaction(TR_STATE_STORAGE);
	}
	triggerConditions_.clear();
}


void StateStorage430::enableTrace()
{
	//Can't use trace if already active in variable watch mode
	if ((controlRegister_ & STOR_EN) && (controlRegister_ & 0x6) == STOR_MODE_VAR_WATCH)
		throw EM_StateStorageConflictException();

	BOOST_FOREACH(TriggerConditionPtr& condition, triggerConditions_)
	{
		condition->addReaction(TR_STATE_STORAGE);
	}
	controlRegister_ |= STOR_EN;
}


void StateStorage430::disableTrace() 
{
	//Ignore if in variable watch mode
	if ((controlRegister_ & 0x6) != STOR_MODE_VAR_WATCH)
	{
		BOOST_FOREACH(TriggerConditionPtr& condition, triggerConditions_)
		{
			condition->removeReaction(TR_STATE_STORAGE);
		}
		controlRegister_ &= ~STOR_EN;
	}
}


void StateStorage430::reset()
{
	controlRegister_ |= STOR_RESET;
}


void StateStorage430::setStartOnTrigger(bool startOnTrigger)
{
	if (startOnTrigger)
	{
		controlRegister_ |= STOR_START_TRIG;
	}
	else
	{
		controlRegister_ &= ~STOR_START_TRIG;
	}
}


void StateStorage430::setStopOnTrigger(bool stopOnTrigger)
{
	if (stopOnTrigger)
	{
		controlRegister_ |= STOR_STOP_TRIG;
	}
	else
	{
		controlRegister_ &= ~STOR_STOP_TRIG;
	}
}


void StateStorage430::setStoreOnInstructionFetch()
{
	controlRegister_ &= ~STOR_MODE_CLEAR;
	controlRegister_ |= STOR_MODE_INSTR_FETCH;
}


void StateStorage430::setStoreOnTrigger()
{
	controlRegister_ &= ~STOR_MODE_CLEAR;
	controlRegister_ |= STOR_MODE_TRIGGER;
}


void StateStorage430::setStoreOnClock()
{
	controlRegister_ &= ~STOR_MODE_CLEAR;
	controlRegister_ |= STOR_MODE_ALL_CYCLES;
}


void StateStorage430::setStoreContinuously()
{
	controlRegister_ &= ~STOR_ONE_SHOT;
}


void StateStorage430::setStoreUntilFull()
{
	controlRegister_ |= STOR_ONE_SHOT;
}


const TraceBuffer StateStorage430::getTraceData()
{
	boost::mutex::scoped_lock lock(traceBufferMutex_);
	return TraceBuffer(traceBuffer_);
}
