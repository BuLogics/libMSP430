/*
 * RegisterCondition430.h
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


#ifndef DLL430_REGISTER_CONDITION_430_H
#define DLL430_REGISTER_CONDITION_430_H

#include <stdint.h>
#include "TriggerCondition430.h"
#include "IRegisterCondition.h"


namespace TI { namespace DLL430 {

class Trigger430;

class RegisterCondition430 : public TriggerCondition430, public IRegisterCondition
{
public:
	RegisterCondition430(TriggerManager430Ptr triggerManager, uint8_t reg, uint32_t value, uint32_t mask = 0xffffffff, ComparisonOperation = CO_EQUAL);

	virtual void setComparator(ComparisonOperation op);
	virtual void setRegister(uint8_t reg);
	virtual void setValue(uint32_t value, uint32_t mask = 0xffffffff);
	
	virtual void addReaction(TriggerReaction reaction) { TriggerCondition430::addReaction(reaction); }
	virtual void removeReaction(TriggerReaction reaction) { TriggerCondition430::removeReaction(reaction); }

	virtual void combine(TriggerConditionPtr condition) { TriggerCondition430::combine(condition); }
	virtual uint32_t getId() const { return TriggerCondition430::getId(); }

	virtual void enable() { TriggerCondition430::enable(); }
	virtual void disable() { TriggerCondition430::disable(); }

private:
	Trigger430* registerTrigger_;
};

}}

#endif
