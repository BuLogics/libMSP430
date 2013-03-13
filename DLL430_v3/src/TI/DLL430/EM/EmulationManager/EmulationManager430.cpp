/*
 * EmulationManager430.cpp
 *
 * Emulation manager holds and manages the different modules
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


#include "EmulationManager430.h"
#include "../BreakpointManager/IBreakpointManager.h"
#include "../CycleCounter/ICycleCounter.h"
#include "../TriggerManager/TriggerManager430.h"
#include "../TriggerCondition/TriggerConditionManager430.h"
#include "../Sequencer/Sequencer430.h"
#include "../StateStorage430/StateStorage430.h"
#include "../Exceptions/Exceptions.h"


using namespace TI::DLL430;


EmulationManager430::EmulationManager430()
{
}


EmulationManager430::~EmulationManager430()
{
	if (mBreakpointManager)
		mBreakpointManager->cleanup();

	if (mCycleCounter)
		mCycleCounter->cleanup();

	if (mTrace)
		mTrace->cleanup();

	if (mSequencer)
		mSequencer->cleanup();
}


TriggerConditionManagerPtr EmulationManager430::getTriggerConditionManager() const
{
	if (!mTriggerConditionManager)
		throw EM_NoTriggerConditionManagerException();

	return mTriggerConditionManager;
}


BreakpointManagerPtr EmulationManager430::getBreakpointManager() const
{
	if (!mBreakpointManager)
		throw EM_NoBreakpointManagerException();

	return mBreakpointManager;
}


ClockControlPtr EmulationManager430::getClockControl() const
{
	if (!mClockControl)
		throw EM_NoClockControlException();

	return mClockControl;
}


CycleCounterPtr EmulationManager430::getCycleCounter() const
{
	if (!mCycleCounter)
		throw EM_NoCycleCounterException();

	return mCycleCounter;
}


SequencerPtr EmulationManager430::getSequencer() const
{
	if (!mSequencer)
		throw EM_NoSequencerException();

	return mSequencer;
}


TracePtr EmulationManager430::getTrace() const
{
	if (!mTrace)
		throw EM_NoTraceException();

	return mTrace;
}


VariableWatchPtr EmulationManager430::getVariableWatch() const
{
	if (!mVariableWatch)
		throw EM_NoVariableWatchException();

	return mVariableWatch;
}


bool EmulationManager430::hasTriggerConditionManager() const 
{
	return mTriggerConditionManager;
}


bool EmulationManager430::hasBreakpointManager() const 
{
	return mBreakpointManager;
}


bool EmulationManager430::hasClockControl() const 
{
	return mClockControl;
}


bool EmulationManager430::hasCycleCounter() const 
{
	return mCycleCounter;
}


bool EmulationManager430::hasSequencer() const 
{
	return mSequencer;
}


bool EmulationManager430::hasTrace() const 
{
	return mTrace;
}


bool EmulationManager430::hasVariableWatch() const 
{
	return mVariableWatch;
}


void EmulationManager430::writeConfiguration() const
{
	if (mTriggerManager)
	{
		const bool sequencerEnabled = mSequencer && mSequencer->isEnabled();

		mTriggerManager->configureTriggers(sequencerEnabled);
		mTriggerManager->writeAllTriggers();
		mTriggerManager->writeTriggerReactions();
	}

	if (mBreakpointManager)
	{
		mBreakpointManager->writeConfiguration();
	}

	if (mCycleCounter)
	{
		//mCycleCounter->writeConfiguration();
	}

	if (mTrace)
	{
		mTrace->writeConfiguration();
	}

	if (mSequencer)
	{
		mSequencer->writeConfiguration();
	}
}


void EmulationManager430::onEvent(MessageDataPtr messageData)  const
{
	if (mTrace)
	{
		mTrace->onEvent(messageData);
	}

	if (mVariableWatch)
	{
		mVariableWatch->onEvent(messageData);
	}
}
