/*
 * Log.h
 *
 * Log handler interface.
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
 
#if _MSC_VER > 1000
#pragma once
#endif
#ifndef DLL430_LOGGER_H
#define DLL430_LOGGER_H

#include <DLL430_SYMBOL.h>
namespace TI
{
	namespace DLL430
	{
		/** \brief log target interface
		 *
		 * If you want to get log messages, create a class that
		 * implements this interface.
		 */
		class DLL430_SYMBOL LogTarget
		{
		public:
			enum Severity {
				FATAL = 0, /**< marks a fatal error, that cannot be recovered from */
				ERR = 1, /**< marks a non-fatal error */
				WARN = 2, /**< marks a warning */
				INFO = 3 /**< marks an informational message */
			};
			virtual ~LogTarget () {};

			/** \brief called once for each message
			 *
			 * \param sev the log severity
			 * \param id  the log id (no strict relationship to message!)
			 * \param message textural description
			 */
			virtual void log (Severity sev, unsigned int id, const char* message) = 0;
		};

		/** \brief register LogTarget instances to receive log messages */
		class DLL430_SYMBOL LogRegistry
		{
		public:
			/** \brief register a class instance to call log() on
			 *
			 * \param target pointer to instance of a class that implements LogTarget
			 */
			virtual void registerLogTarget (LogTarget* target) = 0;

			/** \brief deregister a class instance
			 *
			 * \param target pointer to instance of a class that implements LogTarget
			 */
			virtual void deregisterLogTarget (LogTarget* target) = 0;
			
		protected:
			virtual ~LogRegistry () {};
		};

	};
};

#endif /* DLL430_LOGGER_H */
