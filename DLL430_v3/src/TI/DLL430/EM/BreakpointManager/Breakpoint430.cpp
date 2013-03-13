/*
 * Breakpoint430.cpp
 *
 * Breakpoint implementation for MSP430.
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

#include "Breakpoint430.h"
#include "../TriggerCondition/ITriggerCondition.h"


using namespace TI::DLL430;

Breakpoint430::Breakpoint430()
	: mIsEnabled(true)
{
}


Breakpoint430::Breakpoint430(TriggerConditionPtr condition)
	: mIsEnabled(true)
{
	addTriggerCondition(condition);
}


Breakpoint430::~Breakpoint430()
{
	disable();
}


void Breakpoint430::addTriggerCondition(TriggerConditionPtr condition)
{
	if (condition)
	{
		mTriggerConditions.push_back(condition);
	}
	if (mIsEnabled)
	{
		condition->addReaction(TR_BREAK);
	}
		
}


void Breakpoint430::enable()
{
	BOOST_FOREACH(TriggerConditionPtr& condition, mTriggerConditions)
	{
		condition->addReaction(TR_BREAK);
	}
	mIsEnabled = true;
}


void Breakpoint430::disable()
{
	BOOST_FOREACH(TriggerConditionPtr& condition, mTriggerConditions)
	{
		condition->removeReaction(TR_BREAK);
	}
	mIsEnabled = false;
}
