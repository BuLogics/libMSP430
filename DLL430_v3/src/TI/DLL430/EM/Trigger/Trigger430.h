/*
 * Trigger430.h
 *
 * Common implementation for triggers on 430
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


#ifndef DLL430_TRIGGER_430_H
#define DLL430_TRIGGER_430_H

#include <stdint.h>
#include <set>
#include <map>

#include "../TriggerCondition/TriggerDefinitions.h"

namespace TI { namespace DLL430 {

class Trigger430
{
public:
	enum TYPE { BUS_TRIGGER, REGISTER_TRIGGER };

	Trigger430(TYPE type, uint32_t id);
	virtual ~Trigger430() {}

	virtual uint32_t getId() const;
	
	virtual void read();
	virtual void write();
	virtual void reset();

	virtual void combineWith(Trigger430* trigger);
	virtual void uncombineWith(Trigger430* trigger);
	virtual uint32_t getCombinationValue() const;

	virtual void swapTriggerBlock(Trigger430& trigger);

	virtual void addReaction(TriggerReaction reaction);
	virtual void removeReaction(TriggerReaction reaction);
	virtual const std::set<TriggerReaction>& getReactions() const;

	virtual void setComparisonOperation(ComparisonOperation op);
	virtual void setValue(uint32_t value);
	virtual void setMask(uint32_t mask);

	virtual bool isCombinationTrigger() const;
	virtual void isCombinationTrigger(bool isInCombination);

	virtual bool isInUse() const;
	virtual void isInUse(bool inUse);

	virtual bool isEnabled() const;
	virtual void isEnabled(bool enabled);

	//Bus triggers
	virtual void setMemoryAddressBus();
	virtual void setMemoryDataBus();
	virtual void setAccessType(AccessType accessType);

	//Register triggers
	virtual void setRegister(uint32_t reg);
	virtual void setHold(bool hold);


	static std::map<ComparisonOperation, uint16_t> comparisonOpBits;
	static std::map<AccessType, uint16_t> accessTypeBits;
	static bool bitwiseMasks;

private:
	uint32_t valueRegisterAddress() const;
	uint32_t controlRegisterAddress() const;
	uint32_t maskRegisterAddress() const;

	TYPE type_;

	uint32_t triggerValueRegister_;
	uint16_t triggerControlRegister_;
	uint32_t triggerMaskRegister_;
	uint32_t id_;

	bool isInUse_;
	bool isEnabled_;
	bool isCombinationTrigger_;
	
	std::set<TriggerReaction> reactions_;
	std::set<Trigger430*> combinedWith_;
};

}}

#endif
