/*
 * RegisterCondition430.cpp
 *
 * Implementation of register trigger conditions for 430
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



#include "RegisterCondition430.h"
#include "../TriggerManager/TriggerManager430.h"
#include "../Trigger/Trigger430.h"

using namespace TI::DLL430;


RegisterCondition430::RegisterCondition430(TriggerManager430Ptr triggerManager, uint8_t reg, uint32_t value, uint32_t mask, ComparisonOperation op)
	: TriggerCondition430(triggerManager)
	, registerTrigger_(NULL)
{
	registerTrigger_ = triggerManager->getRegisterTrigger();
	
	if (registerTrigger_ != NULL)
	{
		addTrigger(registerTrigger_);
		
		setRegister(reg);
		setValue(value, mask);
		setComparator(op);
	}
}

void RegisterCondition430::setComparator(ComparisonOperation op)
{
	if (registerTrigger_ != NULL)
	{
		registerTrigger_->setComparisonOperation(op);
	}
}

void RegisterCondition430::setRegister(uint8_t reg)
{
	if (registerTrigger_ != NULL)
	{
		registerTrigger_->setRegister(reg);
	}
}

void RegisterCondition430::setValue(uint32_t value, uint32_t mask)
{
	if (registerTrigger_ != NULL)
	{
		registerTrigger_->setValue(value);
		registerTrigger_->setMask(mask);
	}
}
