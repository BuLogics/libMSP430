/*
 * TriggerConditionManager430.cpp
 *
 * Implementation of trigger condition manager for 430
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


#include <boost/make_shared.hpp>

#include "TriggerConditionManager430.h"
#include "TriggerCondition430.h"
#include "TriggerCondition430.h"
#include "RegisterCondition430.h"
#include "AddressCondition430.h"
#include "DataValueCondition430.h"
#include "AddressRangeCondition430.h"
#include "DataRangeCondition430.h"
#include "../TriggerManager/TriggerManager430.h"
#include "../Exceptions/Exceptions.h"


using namespace TI::DLL430;


TriggerConditionManager430::TriggerConditionManager430(TriggerManager430Ptr triggerManager)
	: triggerManager_(triggerManager)
{
}


SoftwareTriggerConditionPtr TriggerConditionManager430::createSoftwareTriggerCondition(uint32_t)
{
	throw EM_NotSupportedException();
}


RegisterConditionPtr TriggerConditionManager430::createRegisterCondition(uint8_t reg, uint32_t value, uint32_t mask, 
																		 ComparisonOperation op) 
{
	if (!triggerManager_->hasRegisterTriggers())
		throw EM_TriggerParameterException();

	if (triggerManager_->numAvailableRegisterTriggers() < 1)
		throw EM_TriggerResourceException();

	return boost::make_shared<RegisterCondition430>(triggerManager_, reg, value, mask, op);
}


InstructionAddressConditionPtr TriggerConditionManager430::createInstructionAddressCondition(uint32_t address, uint32_t mask, 
																							 AccessType accessType, 
																							 ComparisonOperation op) 
{
	if (triggerManager_->numAvailableBusTriggers() < 1)
		throw EM_TriggerResourceException();

	return boost::make_shared<AddressCondition430>(triggerManager_, address, mask, accessType, op);
}


DataAddressConditionPtr TriggerConditionManager430::createDataAddressCondition(uint32_t address, uint32_t mask, 
																			   AccessType accessType, ComparisonOperation op) 
{
	if (triggerManager_->numAvailableBusTriggers() < 1)
		throw EM_TriggerResourceException();

	return boost::make_shared<AddressCondition430>(triggerManager_, address, mask, accessType, op);
}


DataValueConditionPtr TriggerConditionManager430::createDataValueCondition(uint32_t value, uint32_t mask, 
																		   AccessType accessType,  ComparisonOperation op) 
{
	if (triggerManager_->numAvailableBusTriggers() < 1)
		throw EM_TriggerResourceException();

	return boost::make_shared<DataValueCondition430>(triggerManager_, value, mask, accessType, op);
}


InstructionRangeConditionPtr TriggerConditionManager430::createInstructionRangeCondition(uint32_t minAddress, uint32_t maxAddress, 
																						 uint32_t minMask, uint32_t maxMask, 
																						 AccessType accessType, bool outside) 
{
	if (triggerManager_->numAvailableBusTriggers() < 2)
		throw EM_TriggerResourceException();

	return boost::make_shared<AddressRangeCondition430>(triggerManager_, minAddress, maxAddress, minMask, maxMask, accessType, outside);
}


AddressRangeConditionPtr TriggerConditionManager430::createAddressRangeCondition(uint32_t minAddress, uint32_t maxAddress, 
																				 uint32_t minMask, uint32_t maxMask,
																				 AccessType accessType, bool outside) 
{
	if (triggerManager_->numAvailableBusTriggers() < 2)
		throw EM_TriggerResourceException();

	return boost::make_shared<AddressRangeCondition430>(triggerManager_, minAddress, maxAddress, minMask, maxMask, accessType, outside);	
}


DataRangeConditionPtr TriggerConditionManager430::createDataRangeCondition(uint32_t minValue, uint32_t maxValue, 
																		   uint32_t minMask, uint32_t maxMask,
																		   AccessType accessType, bool outside) 
{
	if (triggerManager_->numAvailableBusTriggers() < 2)
		throw EM_TriggerResourceException();

	return boost::make_shared<DataRangeCondition430>(triggerManager_, minValue, maxValue, minMask, maxMask, accessType, outside);
}
