/*
 * DebugManager.h 
 *
 * Interface for classes which provide access to a device DB.
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
#ifndef DLL430_DEVICEDBMANAGER_H
#define DLL430_DEVICEDBMANAGER_H

#include <stdlib.h>

#include <DLL430_SYMBOL.h>
namespace TI
{
	namespace DLL430
	{
		/** \brief device information unrelated to other managers */
		class DLL430_SYMBOL DeviceDbManager
		{
		public:
			virtual ~DeviceDbManager () {};

			/** \brief get the name of a device
			 *
			 * \param id the device ID
			 * \return pointer to the name of the device
			 */
			virtual const char* getEntryName (size_t id) const = 0;
			
			/** \brief the maximum device ID
			 *
			 * \return the highest device ID known
			 */
			virtual size_t getMaxId () const = 0;
			
			/** \brief check if the device has TestVPP
			 *
			 * \param id the device ID to check
			 * \return true if the device has a TestVPP, else false
			 */
			virtual bool hasTestVpp (size_t id) const = 0;
		};

	};
};

#endif /* DLL430_DEVICEDBMANAGER_H */
