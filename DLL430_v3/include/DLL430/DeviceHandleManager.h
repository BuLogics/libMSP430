/*
 * DeviceHandleManager.h 
 *
 * Controls access to one or more DeviceHandles.
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
#ifndef DLL430_DeviceHandleManager_H
#define DLL430_DeviceHandleManager_H

#include <DLL430/DeviceHandle.h>
#include <DLL430/DeviceChainInfo.h>

#include <DLL430_SYMBOL.h>

namespace TI
{
	namespace DLL430
	{
		typedef DeviceChainInfo value_type;
		typedef std::vector<value_type> DeviceChainInfoList;

		class DLL430_SYMBOL DeviceHandleManager
		{
		public:
			
			virtual ~DeviceHandleManager () {};

			/** \brief get a list of available devices
			 *
			 * The scans the system for connected and supported devices. The result
			 * is cached and reused on the next call if update is not true.
			 *
			 * \param update force rescanning
			 * \return pointer to the list
			 */
			virtual DeviceChainInfoList* getDeviceChainInfo (bool update = false) = 0;

			/** \brief create a new device handle instance
			 *
			 * This creates a new device handle. 
			 *
			 * \param port the DeviceChainInfoList iterator of the selected port
			 * \param vcc  optional parameter that sets the voltage (in mV)
			 * \return pointer to the API handle
			 */
			virtual DeviceHandle* createDeviceHandle (DeviceChainInfoList::iterator& device, uint32_t device_code) = 0;

			/** \brief destroy a given device handle
			 *
			 * \param handle the device handle to destroy
			 */
			virtual void destroyDeviceHandle (DeviceHandle*) = 0;
		protected:

		private:
		};

	};
};

#endif /* DLL430_DeviceHandleManager_H */
