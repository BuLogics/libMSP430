/*
 * TriggerDefinitions.cpp
 *
 * Definitions used for trigger condition configuration
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


#ifndef DLL430_TRIGGER_DEFINITIONS_H
#define DLL430_TRIGGER_DEFINITIONS_H

namespace TI { namespace DLL430 {

	enum TriggerReaction 
	{ 
		TR_BREAK,
		TR_CYCLE_COUNTER,
		TR_SEQUENCER,
		TR_SEQUENCER_RESET,
		TR_STATE_STORAGE,
		TR_VARIABLE_WATCH,
		TR_NUM_REACTIONS
	};

	enum ComparisonOperation
	{ 
		CO_EQUAL,
		CO_GREATER_EQUAL,
		CO_LESS_EQUAL,
		CO_NOT_EQUAL
	};
	
	enum AccessType 
	{
		AT_FETCH,
		AT_FETCH_HOLD,
		AT_NO_FETCH,
		AT_DONT_CARE,
		AT_NO_FETCH_READ,
		AT_NO_FETCH_WRITE,
		AT_READ,
		AT_WRITE,
		AT_NO_FETCH_NO_DMA,
		AT_DMA,
		AT_NO_DMA,
		AT_WRITE_NO_DMA,
		AT_NO_FETCH_READ_NO_DMA,
		AT_READ_NO_DMA,
		AT_READ_DMA,
		AT_WRITE_DMA
	};

}}

#endif
