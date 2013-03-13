/*
 * IInstructionRangeCondition.h
 *
 * Interface for instruction address range trigger conditions
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



#ifndef DLL430_IINSTRUCTION_RANGE_CONDITION_H
#define DLL430_IINSTRUCTION_RANGE_CONDITION_H

#include <stdint.h>
#include "ITriggerCondition.h"

namespace TI { namespace DLL430 {

//Will trigger if an instruction within a specified address range is accessed
class IInstructionRangeCondition : public ITriggerCondition
{
public:
	//Type of access to trigger on (instruction fetch, etc.)
	virtual void setAccessType(AccessType accessType) = 0;

	//Trigger if instruction address is within set range
	virtual void setInside() = 0;

	//Trigger is instruction address is outside set range
	virtual void setOutside() = 0;

	//Set lower bound of address range
	virtual void setMinAddress(uint32_t address, uint32_t mask = 0xffffffff) = 0;

	//Set upper bound of address range
	virtual void setMaxAddress(uint32_t address, uint32_t mask = 0xffffffff) = 0;
};

}}

#endif
