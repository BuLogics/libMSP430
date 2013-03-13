/*
 * ISequencer.h
 *
 * Interface for sequencer module
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


#ifndef DLL430_ISEQUENCER_H
#define DLL430_ISEQUENCER_H

#include <stdint.h>
#include "../TriggerCondition/ITriggerCondition.h"

namespace TI { namespace DLL430 {


enum SequencerTriggerMode { STM_EDGE_TRIGGER, STM_LEVEL_TRIGGER };


class ISequencer : public ITriggerCondition
{
public:
	virtual ~ISequencer() {}

	//Returns max. number of states (only final state can trigger reactions)
	virtual uint32_t getMaxStates() const = 0;

	//Return number of transitions going out from each state
	virtual uint32_t getMaxTransitions() const = 0;

	//Read current sequencer state from device
	virtual uint32_t readCurrentState() const = 0;

	//Reset sequencer to state 0
	virtual void reset() = 0;
	
	//Set triggers to be either edge or level sensitive
	virtual void setTriggerMode(SequencerTriggerMode mode) = 0;
	
	//Set a trigger condition to automatically reset the sequencer
	virtual void setResetTrigger(TriggerConditionPtr triggerCondition) = 0;

	//Remove a set reset trigger
	virtual void clearResetTrigger() = 0;

	//Remove all reactions to be triggered when entering final state
	virtual void clearReactions() = 0;

	//Configure a transition between states and trigger to cause state change
	//Important: all transitions must be configured (set unused transitions to be redundant or change to same state)
	virtual void setTransition(uint32_t state, uint32_t transition, uint32_t nextState, TriggerConditionPtr triggerCondition) = 0;

	//Remove trigger condition and reset transition (note: this means undefined behavior for this transition)
	virtual void clearTransition(uint32_t state, uint32_t transition) = 0;

	//Remove all trigger conditions and reset all transitions (their behavior will be undefined)
	virtual void clearAllTransitions() = 0;

	//Only used by EmulationManager
	virtual void initialize() = 0;
	virtual void cleanup() = 0;	
	virtual void writeConfiguration() = 0;
	virtual bool isEnabled() const = 0;
};


}}

#endif
