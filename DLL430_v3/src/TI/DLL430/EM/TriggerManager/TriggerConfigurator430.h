/*
 * TriggerConfigurator430.h
 *
 * Handles assignment of triggers to trigger blocks
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



#ifndef DLL430_TRIGGER_CONFIGURATOR_H
#define DLL430_TRIGGER_CONFIGURATOR_H

#include <stdint.h>
#include <set>
#include <map>
#include <deque>
#include <vector>
#include <boost/foreach.hpp>

#include "../TriggerCondition/TriggerDefinitions.h"
#include "../Trigger/Trigger430.h"

namespace TI { namespace DLL430 {

class TriggerConfigurator430
{
public:
	TriggerConfigurator430(std::deque<const Trigger430*>& busTriggers, 
						   std::deque<const Trigger430*>& combinationTriggers,
						   bool sequencerEnabled);

	bool checkTriggerConfiguration() const;

	bool configureTriggerConfiguration();

private:
	void setupConstraints(size_t numBusTriggers);

	bool configUsesReaction(TriggerReaction reaction) const;

	bool checkReactionCounts() const;

	std::set<uint32_t> getValidIDsForTriggerReactions(const Trigger430& trigger);

	void buildInitialTriggerOptions();

	void filterSequencerIDs();

	bool hasImpossibleTrigger();

	bool assignTriggers();

	uint32_t getCombinationTriggerId(const Trigger430* trigger) const;

	std::set<uint32_t> availableIDs;

	std::set<uint32_t> possibleIDs[TR_NUM_REACTIONS];

	const std::deque<const Trigger430*>& triggers;

	std::vector< std::set<uint32_t> > triggerOptions;

	std::deque<const Trigger430*>& combinationTriggers;
	
	bool sequencerEnabled;
	uint32_t sequencerOutId;
	uint32_t sequencerResetId;
};

}}

#endif
