/*
 * Exceptions.h
 *
 * Emulation module exceptions
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



#ifndef DLL430_EXCEPTIONS_H
#define DLL430_EXCEPTIONS_H

#include <stdexcept>
#include <stdint.h>

namespace TI { namespace DLL430 {

class EM_Exception : public std::runtime_error
{
public:
	uint32_t errorCode() const { return errorCode_; }

protected:
	EM_Exception(uint32_t error, const char* msg) : errorCode_(error), runtime_error(msg) {}

private:
	uint32_t errorCode_;
};



class EM_NoEmulationManagerException : public EM_Exception 
{
public:
	EM_NoEmulationManagerException();
};



class EM_NoTriggerConditionManagerException : public EM_Exception 
{
public:
	EM_NoTriggerConditionManagerException();
};



class EM_NoBreakpointManagerException : public EM_Exception 
{
public:
	EM_NoBreakpointManagerException();
};



class EM_NoClockControlException : public EM_Exception 
{
public:
	EM_NoClockControlException();
};



class EM_NoCycleCounterException : public EM_Exception 
{
public:
	EM_NoCycleCounterException();
};



class EM_NoSequencerException : public EM_Exception 
{
public:
	EM_NoSequencerException();
};



class EM_NoTraceException : public EM_Exception
{
public:
	EM_NoTraceException();
};



class EM_NoVariableWatchException : public EM_Exception
{
public:
	EM_NoVariableWatchException();
};



class EM_NotSupportedException : public EM_Exception 
{
public:
	EM_NotSupportedException();
};



class EM_TriggerResourceException : public EM_Exception 
{
public:
	EM_TriggerResourceException();
};



class EM_TriggerParameterException : public EM_Exception
{
public:
	EM_TriggerParameterException();
};



class EM_RegisterWriteException : public EM_Exception
{
public:
	EM_RegisterWriteException();
};



class EM_RegisterReadException : public EM_Exception
{
public:
	EM_RegisterReadException();
};



class EM_SequencerException : public EM_Exception
{
public:
	EM_SequencerException();
};



class EM_StateStorageConflictException : public EM_Exception
{
public:
	EM_StateStorageConflictException();
};



class EM_NotVariableWatchModeException : public EM_Exception
{
public:
	EM_NotVariableWatchModeException();
};



class EM_TriggerConfigurationException : public EM_Exception
{
public:
	EM_TriggerConfigurationException();
};

}}

#endif
