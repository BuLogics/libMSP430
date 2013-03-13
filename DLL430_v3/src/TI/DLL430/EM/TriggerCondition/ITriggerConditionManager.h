/*
 * ITriggerConditionManager.h
 *
 * Interface for trigger condition manager
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


#ifndef DLL430_ITRIGGER_CONDITION_MANAGER_H
#define DLL430_ITRIGGER_CONDITION_MANAGER_H

#include <stdint.h>
#include <boost/shared_ptr.hpp>

#include "ISoftwareTriggerCondition.h"
#include "IRegisterCondition.h"
#include "IInstructionAddressCondition.h"
#include "IInstructionRangeCondition.h"
#include "IDataAddressCondition.h"
#include "IDataValueCondition.h"
#include "IDataRangeCondition.h"
#include "IAddressRangeCondition.h"


namespace TI { namespace DLL430 {

typedef boost::shared_ptr<ITriggerCondition> TriggerConditionPtr;
typedef boost::shared_ptr<ISoftwareTriggerCondition> SoftwareTriggerConditionPtr;
typedef boost::shared_ptr<IRegisterCondition> RegisterConditionPtr;
typedef boost::shared_ptr<IInstructionAddressCondition> InstructionAddressConditionPtr;
typedef boost::shared_ptr<IDataAddressCondition> DataAddressConditionPtr;
typedef boost::shared_ptr<IDataValueCondition> DataValueConditionPtr;
typedef boost::shared_ptr<IInstructionRangeCondition> InstructionRangeConditionPtr;
typedef boost::shared_ptr<IAddressRangeCondition> AddressRangeConditionPtr;
typedef boost::shared_ptr<IDataRangeCondition> DataRangeConditionPtr;


//Responsible for checking resources and creating trigger conditions
class ITriggerConditionManager
{
public:
	virtual ~ITriggerConditionManager() {}

	//Creator functions for all trigger condition types (will throw on error)
	virtual SoftwareTriggerConditionPtr createSoftwareTriggerCondition(uint32_t address = 0) = 0;

	virtual RegisterConditionPtr createRegisterCondition(uint8_t reg = 0, uint32_t value = 0, uint32_t mask = 0xffffffff, 
														 ComparisonOperation op = CO_EQUAL) = 0;
	
	virtual InstructionAddressConditionPtr createInstructionAddressCondition(uint32_t address = 0, uint32_t mask = 0xffffffff, 
																			 AccessType = AT_FETCH, ComparisonOperation = CO_EQUAL) = 0;	
	
	virtual DataAddressConditionPtr createDataAddressCondition(uint32_t address = 0, uint32_t mask = 0xffffffff, 
															   AccessType = AT_FETCH, ComparisonOperation = CO_EQUAL) = 0;
	
	virtual DataValueConditionPtr createDataValueCondition(uint32_t value = 0, uint32_t mask = 0xffffffff, 
														   AccessType = AT_FETCH, ComparisonOperation = CO_EQUAL) = 0;
	
	virtual InstructionRangeConditionPtr createInstructionRangeCondition(uint32_t minAddress = 0, uint32_t maxAddress = 0, 
																		 uint32_t minMask = 0xffffffff, uint32_t maxMask = 0xffffffff, 
																		 AccessType = AT_FETCH, bool outside = false) = 0;
	
	virtual AddressRangeConditionPtr createAddressRangeCondition(uint32_t minAddress = 0, uint32_t maxAddress = 0, 
																 uint32_t minMask = 0xffffffff, uint32_t maxMask = 0xffffffff, 
																 AccessType = AT_FETCH, bool outside = false) = 0;
	
	virtual DataRangeConditionPtr createDataRangeCondition(uint32_t minValue = 0, uint32_t maxValue = 0,
														   uint32_t minMask = 0xffffffff, uint32_t maxMask = 0xffffffff, 
														   AccessType = AT_FETCH, bool outside = false) = 0;
};

}}

#endif
