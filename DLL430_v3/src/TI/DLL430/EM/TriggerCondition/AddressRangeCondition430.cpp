/*
 * AddressRangeCondition430.cpp
 *
 * Implementation of address range trigger conditions for 430
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


#include "AddressRangeCondition430.h"
#include "../TriggerManager/TriggerManager430.h"
#include "../Trigger/Trigger430.h"

using namespace TI::DLL430;


AddressRangeCondition430::AddressRangeCondition430(TriggerManager430Ptr triggerManager, uint32_t minAddress, uint32_t maxAddress, 
												   uint32_t minMask, uint32_t maxMask, AccessType accessType, bool outside)
	: TriggerCondition430(triggerManager)
	, minTrigger_(NULL)
	, maxTrigger_(NULL)
{
	minTrigger_ = triggerManager->getBusTrigger();
	maxTrigger_ = triggerManager->getBusTrigger();

	if (minTrigger_ != NULL && maxTrigger_ != NULL)
	{
		addTrigger(minTrigger_);
		addTrigger(maxTrigger_);

		setMinAddress(minAddress, minMask);		
		setMaxAddress(maxAddress, maxMask);
		setAccessType(accessType);

		if (outside)
		{
			setOutside();
		}
		else
		{
			setInside();
		}
	}
}

void AddressRangeCondition430::setAccessType(AccessType accessType)
{
	if (minTrigger_ != NULL && maxTrigger_ != NULL)
	{
		minTrigger_->setAccessType(accessType);
		maxTrigger_->setAccessType(accessType);
	}
}

void AddressRangeCondition430::setInside()
{
	if (minTrigger_ != NULL && maxTrigger_ != NULL)
	{
		minTrigger_->setComparisonOperation(CO_GREATER_EQUAL);
		maxTrigger_->setComparisonOperation(CO_LESS_EQUAL);

		minTrigger_->combineWith(maxTrigger_);
	}
}

void AddressRangeCondition430::setOutside()
{
	if (minTrigger_ != NULL && maxTrigger_ != NULL)
	{
		minTrigger_->setComparisonOperation(CO_LESS_EQUAL);
		maxTrigger_->setComparisonOperation(CO_GREATER_EQUAL);

		minTrigger_->uncombineWith(maxTrigger_);
	}
}

void AddressRangeCondition430::setMinAddress(uint32_t address, uint32_t mask)
{
	if (minTrigger_ != NULL)
	{
		minTrigger_->setValue(address);
		minTrigger_->setMask(mask);
	}
}

void AddressRangeCondition430::setMaxAddress(uint32_t address, uint32_t mask)
{
	if (maxTrigger_ != NULL)
	{
		maxTrigger_->setValue(address);
		maxTrigger_->setMask(mask);
	}
}
