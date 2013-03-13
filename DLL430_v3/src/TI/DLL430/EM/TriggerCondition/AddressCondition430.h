/*
 * AddressCondition430.h
 *
 * Implementation of data and instruction address trigger conditions for 430
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



#ifndef DLL430_ADDRESS_CONDITION_430_H
#define DLL430_ADDRESS_CONDITION_430_H

#include <stdint.h>
#include "TriggerCondition430.h"
#include "IInstructionAddressCondition.h"
#include "IDataAddressCondition.h"
#include "../Trigger/Trigger430.h"

namespace TI { namespace DLL430 {

class AddressCondition430 : public TriggerCondition430, public IInstructionAddressCondition, public IDataAddressCondition
{
public:
	AddressCondition430(TriggerManager430Ptr triggerManager, uint32_t address, uint32_t mask = 0xffffffff, AccessType = AT_FETCH, ComparisonOperation = CO_EQUAL);

	virtual void setComparator(ComparisonOperation op);
	virtual void setAccessType(AccessType accessType);
	virtual void setAddress(uint32_t address, uint32_t mask = 0xffffffff);

	virtual void addReaction(TriggerReaction reaction) { TriggerCondition430::addReaction(reaction); }
	virtual void removeReaction(TriggerReaction reaction) { TriggerCondition430::removeReaction(reaction); }

	virtual void combine(TriggerConditionPtr condition) { TriggerCondition430::combine(condition); }
	virtual uint32_t getId() const { return TriggerCondition430::getId(); }

	virtual void enable() { TriggerCondition430::enable(); }
	virtual void disable() { TriggerCondition430::disable(); }

private:
	Trigger430* mainTrigger_;
};

}}

#endif
