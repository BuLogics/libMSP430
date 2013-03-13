/*
 * TriggerManager430.h
 *
 * Handles trigger resources on 430
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


#ifndef DLL430_TRIGGER_MANAGER_H
#define DLL430_TRIGGER_MANAGER_H

#include <deque>
#include <set>
#include <map>

#include "../Trigger/Trigger430.h"


namespace TI { namespace DLL430 {

class TriggerManager430
{
public:
	TriggerManager430(int numBusTriggers, int numRegisterTriggers, int numCombinationTriggers, int sequencerOutId);

	void setExtendedComparisons() const;
	void setExtendedAccessTypes() const;
	void setBitwiseMasking() const;
	
	Trigger430* getBusTrigger();
	Trigger430* getRegisterTrigger();
	void releaseTrigger(Trigger430* trigger);

	uint32_t getCombinationTrigger(const Trigger430* trigger) const;

	bool hasRegisterTriggers() const;

	int numAvailableBusTriggers() const;
	int numAvailableRegisterTriggers() const;
	
	void configureTriggers(bool sequencerEnabled);
	void writeAllTriggers();
	void writeTriggerReactions();


	void enableSequencerOutReactions();
	void disableSequencerOutReactions();

	void addSequencerOutReaction(TriggerReaction reaction);
	void removeSequencerOutReaction(TriggerReaction reaction);
	void clearSequencerOutReactions();

private:
	void verifyForSingleStepping(const std::deque<const Trigger430*> usedTriggers);
	Trigger430* getTriggerAtBlock(uint32_t blockId);

	std::deque<Trigger430> mBusTriggers;
	std::deque<Trigger430> mRegisterTriggers;
	std::deque<const Trigger430*> mCombinationTriggers;

	std::set<TriggerReaction> mSequencerReactions;
	uint8_t mSequencerOutTriggerId;
	bool mSequencerOutReactionsEnabled;
};

}}

#endif
