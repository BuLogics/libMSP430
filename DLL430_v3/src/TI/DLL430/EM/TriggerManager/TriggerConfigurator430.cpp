/*
 * TriggerConfigurator430.cpp
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



#include "TriggerConfigurator430.h"

#include <vector>
#include <algorithm>

using namespace std;
using namespace TI::DLL430;

//@OPTION In case of performance issues with trigger configuration
#define REDUCED_TRIGGER_TABLE


template<typename T, int N>
set<T> arrayToSet(const T(&a)[N])
{
	return set<T>(a, a+N);
}

TriggerConfigurator430::TriggerConfigurator430(deque<const Trigger430*>& triggers,
											   deque<const Trigger430*>& combinationTriggers,
											   bool sequencerEnabled)
	: triggers(triggers)
	, combinationTriggers(combinationTriggers)
	, sequencerEnabled(sequencerEnabled)
	, sequencerOutId(0xFF)
	, sequencerResetId(0xFF)
	
{
	setupConstraints(combinationTriggers.size());
}


//Must be adjusted for each EEM (use this for large >=8 combinations)
void TriggerConfigurator430::setupConstraints(size_t numCombinationTriggers)
{
	for (uint32_t id = 0; id < numCombinationTriggers; ++id)
	{
		availableIDs.insert(id);

		possibleIDs[TR_BREAK].insert(id);

		//Trigger 0 can only be used for breakpoints, due to single stepping hijacking trigger 0
		//Other reactions wouldn't trigger when stepping
		if (id > 0)
		{
			possibleIDs[TR_CYCLE_COUNTER].insert(id);
		
			if (numCombinationTriggers >= 8)
			{
				possibleIDs[TR_STATE_STORAGE].insert(id);
				possibleIDs[TR_VARIABLE_WATCH].insert(id);
			}
		}
	}

	if (numCombinationTriggers == 6)
	{
		const uint32_t validForSequencer[] = {2,3,4,5};	
		possibleIDs[TR_SEQUENCER] = arrayToSet(validForSequencer);
		possibleIDs[TR_SEQUENCER_RESET].insert(1);		
		
		sequencerOutId = 5;
		sequencerResetId = 1;
	}

	if (numCombinationTriggers >= 8)
	{
		const uint32_t validForSequencer[] = {4,5,6,7};
		possibleIDs[TR_SEQUENCER] = arrayToSet(validForSequencer);
		possibleIDs[TR_SEQUENCER_RESET].insert(3);		
		
		sequencerOutId = 7;
		sequencerResetId = 3;
	}
}


//Check that there aren't more active primary triggers than available combination triggers and
//check if there more triggers causing a reaction than there are possible inputs for the reaction
//ex. not more than one trigger can cause a sequencer reset
bool TriggerConfigurator430::checkReactionCounts() const
{
	if (triggers.size() > combinationTriggers.size())
	{
		return false;
	}

	size_t reactionCounts[TR_NUM_REACTIONS] = {0};
	BOOST_FOREACH(const Trigger430* trigger, triggers)
	{
		BOOST_FOREACH(TriggerReaction reaction, trigger->getReactions())
		{
			if (++reactionCounts[reaction] > possibleIDs[reaction].size())
				return false;
		}
	}
	return true;
}


//Intersect all possible IDs for all reactions configured for a trigger
set<uint32_t> TriggerConfigurator430::getValidIDsForTriggerReactions(const Trigger430& trigger)
{
	set<uint32_t> idSet = availableIDs;

	BOOST_FOREACH(TriggerReaction r, trigger.getReactions())
	{
		set<uint32_t> intersection;
		set_intersection(idSet.begin(), idSet.end(), 
							possibleIDs[r].begin(), possibleIDs[r].end(),
							inserter(intersection, intersection.end()));
		idSet = intersection;
	}
	return idSet;
}


//Maps trigger to all allowed combination trigger ids
void TriggerConfigurator430::buildInitialTriggerOptions()
{
	triggerOptions.resize(triggers.size());

	for (size_t i = 0; i < triggers.size(); ++i)
	{
		triggerOptions[i] = getValidIDsForTriggerReactions(*triggers[i]);
	}
}


//When the sequencer is enabled, there are special rules for two combination trigger ids (sequencer output and reset)
void TriggerConfigurator430::filterSequencerIDs()
{
	if (sequencerEnabled)
	{
		for (size_t i = 0; i < triggers.size(); ++i)
		{
			//The sequencerOutId is blocked by the sequencer and can only be used for triggers that ONLY trigger the sequencer
			if (triggers[i]->getReactions().size() > 1 || triggers[i]->getReactions().count(TR_SEQUENCER) == 0 )
			{
				triggerOptions[i].erase(sequencerOutId);
			}

			//The sequencerResetId may only be used for triggers that ALSO trigger a sequencer reset
			if (triggers[i]->getReactions().count(TR_SEQUENCER_RESET) == 0 )
			{
				triggerOptions[i].erase(sequencerResetId);
			}
		}
	}
}


//Quick check if there are still any valid ids left for all triggers
bool TriggerConfigurator430::hasImpossibleTrigger()
{
	BOOST_FOREACH(const set<uint32_t>& options, triggerOptions)
	{
		if (options.empty())
			return true;
	}
	return false;
}



bool TriggerConfigurator430::assignTriggers()
{
	//Setup helper constructs
	const int32_t numTriggers = (int32_t)triggers.size();
	const int numCombinationTriggers = (int)combinationTriggers.size();

	vector<bool> idInUse(numCombinationTriggers, false);
	vector<int32_t> assignedCombinationTrigger(numTriggers, -1);

	//Find valid trigger assignment via backtracking
	int32_t curTrigger = 0;

	while (curTrigger < numTriggers)
	{
		int id = assignedCombinationTrigger[curTrigger];

		if (id >= 0)
		{
			idInUse[id] = false;
		}

		while (++id < numCombinationTriggers && (idInUse[id] || triggerOptions[curTrigger].count(id) == 0) ) ;
			
		if (id < numCombinationTriggers)
		{
			idInUse[id] = true;
			assignedCombinationTrigger[curTrigger++] = id;
		}
		else
		{
			assignedCombinationTrigger[curTrigger--] = -1;
		}

		if (curTrigger < 0)
		{
			return false;
		}
	}


	//Write assignment back to combination trigger table
	combinationTriggers = deque<const Trigger430*>(numCombinationTriggers);

	if (curTrigger > 0)
	{
		for (int i = 0; i < numTriggers; ++i)
		{
			combinationTriggers[ assignedCombinationTrigger[i] ] = triggers[i];
		}
	}
	return true;
}


bool TriggerConfigurator430::configureTriggerConfiguration()
{
	if (!checkReactionCounts())
	{
		return false;
	}

	buildInitialTriggerOptions();

	filterSequencerIDs();

	if (hasImpossibleTrigger())
	{
		return false;
	}

	return assignTriggers();
}


uint32_t TriggerConfigurator430::getCombinationTriggerId(const Trigger430* trigger) const
{
	uint32_t id = 0;
	for (id = 0; id < combinationTriggers.size(); ++id)
	{
		if (combinationTriggers[id] == trigger)
		{
			return id;
		}
	}
	return id;
}


bool TriggerConfigurator430::checkTriggerConfiguration() const
{
	//Can't have more active primary triggers than combination triggers
	if (triggers.size() > combinationTriggers.size())
	{
		return false;
	}

	//Check for unassigned triggers, invalid combination trigger ids
	//or multiple triggers mapped to the same combination trigger
	vector<bool> inUse(combinationTriggers.size(), false);
	BOOST_FOREACH(const Trigger430* trigger, triggers)
	{
		const uint32_t id = getCombinationTriggerId(trigger);
		if (id >= combinationTriggers.size() || inUse[id])
		{
			return false;
		}
		inUse[id] = true;
	}

	//Make sure no combination triggers are configured for unused triggers
	BOOST_FOREACH(const Trigger430* triggerPtr, combinationTriggers)
	{
		if ( triggerPtr != NULL && find(triggers.begin(), triggers.end(), triggerPtr) == triggers.end() )
		{
			return false;
		}
	}


	int collisions = 0;

	BOOST_FOREACH(const Trigger430* trigger, triggers)
	{
		if (!trigger->getReactions().empty())
		{
			const uint32_t assignedCombinationTriggerId = getCombinationTriggerId(trigger);

			//Check if trigger id is valid for all reactions
			BOOST_FOREACH(TriggerReaction reaction, trigger->getReactions())
			{
				if (possibleIDs[reaction].count(assignedCombinationTriggerId) == 0)
				{
					++collisions;
				}
			}
			
			//Check if trigger using sequencer out id is only configured as sequencer input
			if (sequencerEnabled && assignedCombinationTriggerId == sequencerOutId)
			{
				if ( trigger->getReactions().size() > 1 || trigger->getReactions().count(TR_SEQUENCER) == 0 )
				{
					++collisions;
				}
			}

			//Check if trigger using sequencer reset id is supposed to trigger sequencer reset
			if (sequencerEnabled && assignedCombinationTriggerId == sequencerResetId)
			{
				if (trigger->getReactions().count(TR_SEQUENCER_RESET) == 0)
				{
					++collisions;
				}
			}
		}
	}
	return collisions == 0;
}
