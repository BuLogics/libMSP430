/*
 * VariableWatch430.cpp
 *
 * Variable watch implementation for 430
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



#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include "../TriggerCondition/ITriggerCondition.h"
#include "../TriggerCondition/ITriggerConditionManager.h"
#include "../StateStorage430/StateStorage430.h"
#include "../Exceptions/Exceptions.h"

#include "../../MessageData.h"

using namespace std;
using namespace TI::DLL430;


void StateStorage430::onEventVWatch(MessageDataPtr msgData)
{
	msgData->reset();

	uint16_t eventMask = 0;

	(*msgData) >> eventMask;
	
	if (eventMask & 0x8)
	{
		do
		{
			uint32_t variableAddress = 0;
			uint16_t variableValue = 0;

			(*msgData) >> variableAddress >> variableValue;

			if (!msgData->fail())
			{
				updateWatchedVariable(variableAddress, variableValue);
			}
		} while (!msgData->fail());
	}
}



void StateStorage430::updateWatchedVariable(uint32_t address, uint16_t value)
{
	vector< boost::weak_ptr<WatchedVariable430> >::iterator it = watchedVariables_.begin();
	while (it != watchedVariables_.end())
	{
		if (boost::shared_ptr<WatchedVariable430> ptr = it->lock())
		{
			if (ptr->address() == address)
			{
				ptr->setValue(value);
			}

			//upper word of 32bit variable
			if ((ptr->address() + 2 == address) && (ptr->bitSize() > 16))
			{
				ptr->setValueHighWord(value);
			}
			++it;
		}
		else
		{
			it = watchedVariables_.erase(it);
		}			
	}
}



void StateStorage430::enableVWatch() 
{
	//Can't use variable watch if already active in another mode
	if ((controlRegister_ & STOR_EN) && (controlRegister_ & 0x6) != STOR_MODE_VAR_WATCH)
		throw EM_StateStorageConflictException();

	controlRegister_ &= ~STOR_MODE_CLEAR;
	controlRegister_ |= STOR_MODE_VAR_WATCH | STOR_EN | STOR_RESET;
	controlRegister_ |= 0xe000; //Watch triggers 0-15


	vector< boost::weak_ptr<WatchedVariable430> >::iterator it = watchedVariables_.begin();
	while (it != watchedVariables_.end())		
	{
		vector< boost::weak_ptr<WatchedVariable430> >::iterator tmpIt = it++;

		if (boost::shared_ptr<WatchedVariable430> ptr = tmpIt->lock())
		{
			ptr->enable();
		}
	}
}


void StateStorage430::disableVWatch()
{
	//Ignore if not in variable watch mode
	if ((controlRegister_ & 0x6) == STOR_MODE_VAR_WATCH)
	{
		controlRegister_ &= ~(STOR_MODE_CLEAR | STOR_EN);
		controlRegister_ |= STOR_MODE_INSTR_FETCH | STOR_RESET;
	}

	vector< boost::weak_ptr<WatchedVariable430> >::iterator it = watchedVariables_.begin();
	while (it != watchedVariables_.end())		
	{
		vector< boost::weak_ptr<WatchedVariable430> >::iterator tmpIt = it++;

		if (boost::shared_ptr<WatchedVariable430> ptr = tmpIt->lock())
		{
			ptr->disable();
		}
	}
}



WatchedVariablePtr StateStorage430::createWatchedVariable(uint32_t address, uint32_t bitSize, TriggerConditionManagerPtr tcManager)
{
	if ((controlRegister_ & 0x6) != STOR_MODE_VAR_WATCH)
		throw EM_NotVariableWatchModeException();

	DataAddressConditionPtr	lowWordCondition = tcManager->createDataAddressCondition(address);
	lowWordCondition->setAccessType(AT_WRITE);
		
	DataAddressConditionPtr highWordCondition;
	if (bitSize == 32)
	{
		highWordCondition = tcManager->createDataAddressCondition(address + 2);
		highWordCondition->setAccessType(AT_WRITE);
	}

	boost::shared_ptr<WatchedVariable430> variable = boost::make_shared<WatchedVariable430>(address, bitSize, lowWordCondition, highWordCondition);
	watchedVariables_.push_back( boost::weak_ptr<WatchedVariable430>(variable) );
	
	return variable;
}
