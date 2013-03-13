/*
 * TriggerConditionManager430.h
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


#ifndef DLL430_TRIGGER_CONDITION_FACTORY_430_H
#define DLL430_TRIGGER_CONDITION_FACTORY_430_H

#include <list>

#include "ITriggerConditionManager.h"
#include "ITriggerCondition.h"
#include "ITriggerConditionManager.h"
#include "ISoftwareTriggerCondition.h"
#include "IRegisterCondition.h"
#include "IInstructionAddressCondition.h"
#include "IDataAddressCondition.h"
#include "IDataValueCondition.h"
#include "IInstructionRangeCondition.h"
#include "IAddressRangeCondition.h"
#include "IDataRangeCondition.h"

namespace TI { namespace DLL430 {

class TriggerManager430;
typedef boost::shared_ptr<TriggerManager430> TriggerManager430Ptr;


class TriggerConditionManager430 : public ITriggerConditionManager
{
public:
	TriggerConditionManager430(TriggerManager430Ptr triggerManager);

	virtual SoftwareTriggerConditionPtr createSoftwareTriggerCondition(uint32_t address = 0);
	
	virtual RegisterConditionPtr createRegisterCondition(uint8_t reg = 0, 
														 uint32_t value = 0, 
														 uint32_t mask = 0xffffffff, 
														 ComparisonOperation = CO_EQUAL);
	
	virtual InstructionAddressConditionPtr createInstructionAddressCondition(uint32_t address = 0, 
																			 uint32_t mask = 0xffffffff, 
																			 AccessType = AT_FETCH, 
																			 ComparisonOperation = CO_EQUAL);
	
	virtual DataAddressConditionPtr createDataAddressCondition(uint32_t address = 0, 
															   uint32_t mask = 0xffffffff, 
															   AccessType = AT_FETCH, 
															   ComparisonOperation = CO_EQUAL);
	
	virtual DataValueConditionPtr createDataValueCondition(uint32_t value = 0, 
														   uint32_t mask = 0xffffffff, 
														   AccessType = AT_FETCH, 
														   ComparisonOperation = CO_EQUAL);
	
	virtual InstructionRangeConditionPtr createInstructionRangeCondition(uint32_t minAddress = 0, 
																		 uint32_t maxAddress = 0, 
																		 uint32_t minMask = 0xffffffff, 
																		 uint32_t maxMask = 0xffffffff, 
																		 AccessType = AT_FETCH, 
																		 bool outside = false);
	
	virtual AddressRangeConditionPtr createAddressRangeCondition(uint32_t minAddress = 0, 
																 uint32_t maxAddress = 0, 
																 uint32_t minMask = 0xffffffff, 
																 uint32_t maxMask = 0xffffffff, 
																 AccessType = AT_FETCH, 
																 bool outside = false);
	
	virtual DataRangeConditionPtr createDataRangeCondition(uint32_t minValue = 0, 
														   uint32_t maxValue = 0, 
														   uint32_t minMask = 0xffffffff, 
														   uint32_t maxMask = 0xffffffff, 
														   AccessType = AT_FETCH, 
														   bool outside = false);

private:
	TriggerManager430Ptr triggerManager_;
};

}}

#endif
