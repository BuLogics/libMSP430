/*
 * Logger.cpp
 *
 * Logging class with different levels.
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

#include "Logger.h"
#include <algorithm>

using namespace TI::DLL430;

Logger* Logger::instance ()
{
	static Logger* instance = 0;
	if (!instance) instance = new Logger;
	return instance;
}


void Logger::fatal (Logger::Id id, const char* message)
{
	this->log(LogTarget::FATAL, id, message);
}

void Logger::error (Logger::Id id, const char* message)
{
	this->log(LogTarget::ERR, id, message);
}

void Logger::warn (Logger::Id id, const char* message)
{
	this->log(LogTarget::WARN, id, message);
}

void Logger::info (Logger::Id id, const char* message)
{
	this->log(LogTarget::INFO, id, message);
}

void Logger::log (LogTarget::Severity severity, Logger::Id id, const char* message)
{
	mutex_type::scoped_lock lock(this->listMutex);
	list_type::iterator it = this->targets.begin();
	for (; it != this->targets.end(); ++it) 
	{
		LogTarget* target = *it;
		target->log(severity, id, message);
	}
}

void Logger::registerLogTarget (LogTarget* t)
{
	mutex_type::scoped_lock lock(this->listMutex);
	list_type::iterator it = std::find(this->targets.begin(), this->targets.end(), t);
	if (it == this->targets.end())
		this->targets.push_back(t);
}

void Logger::deregisterLogTarget (LogTarget* t)
{
	mutex_type::scoped_lock lock(this->listMutex);
	list_type::iterator it = std::find(this->targets.begin(), this->targets.end(), t);
	if (it != this->targets.end())
		this->targets.erase(it);
}
