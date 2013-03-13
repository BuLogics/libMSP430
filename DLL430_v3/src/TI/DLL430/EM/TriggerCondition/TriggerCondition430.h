/*
 * TriggerCondition430.h
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



#ifndef DLL430_TRIGGER_CONDITION_430_H
#define DLL430_TRIGGER_CONDITION_430_H

#include <stdint.h>
#include <set>
#include <list>
#include <boost/shared_ptr.hpp>
#include "ITriggerCondition.h"

namespace TI { namespace DLL430 {

class Trigger430;
class TriggerManager430;

typedef boost::shared_ptr<TriggerManager430> TriggerManager430Ptr;

class TriggerCondition430
{
protected:
	TriggerCondition430(TriggerManager430Ptr triggerManager);
	virtual ~TriggerCondition430();

	virtual void addReaction(TriggerReaction reaction);
	virtual void removeReaction(TriggerReaction reaction);

	virtual void combine(TriggerConditionPtr condition);
	virtual uint32_t getId() const;
	
	virtual void enable();
	virtual void disable();

	TriggerManager430Ptr triggerManager() { return triggerManager_; }
	void addTrigger(Trigger430* trigger) { triggers_.push_back(trigger); }

private:	
	std::list<Trigger430*> triggers_;
	TriggerManager430Ptr triggerManager_;
};

}}

#endif
