/*
 * IVariableWatch.h
 *
 * Interface for variable watch module
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


#ifndef DLL430_IVARIABLE_WATCH_H
#define DLL430_IVARIABLE_WATCH_H

#include <stdint.h>
#include "../TriggerCondition/ITriggerConditionManager.h"

namespace TI { namespace DLL430 {


class IWatchedVariable;
class MessageData;

typedef boost::shared_ptr<IWatchedVariable> WatchedVariablePtr;
typedef boost::shared_ptr<MessageData> MessageDataPtr;
typedef boost::shared_ptr<ITriggerConditionManager> TriggerConditionManagerPtr;


//Class is responsible to create and update watched variables
class IVariableWatch
{
public:
	virtual ~IVariableWatch() {}

	//Enable/disable variable watch function (on 430 variable watch and trace are mutually exclusive)
	virtual void enable() = 0;
	virtual void disable() = 0;
	
	//Create a watched variable and setup required triggers
	virtual WatchedVariablePtr createWatchedVariable(uint32_t address, uint32_t bitSize, TriggerConditionManagerPtr tcManager) = 0;

	//Only used by EmulationManager
	virtual void initialize() = 0;
	virtual void cleanup() = 0;
	virtual void writeConfiguration() = 0;
	virtual void onEvent(MessageDataPtr msgData) = 0;
};

}}

#endif
