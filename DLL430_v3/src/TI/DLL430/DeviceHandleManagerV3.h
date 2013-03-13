/*
 * DeviceHandleManagerV3.h
 *
 * Handles one or more device handles via JTAG.
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
#ifndef DLL430_DEVICEHANDLEMANAGERV3_H
#define DLL430_DEVICEHANDLEMANAGERV3_H

#include <DLL430/DeviceHandleManager.h>
#include "DeviceHandleV3.h"
#include <fstream>

namespace TI
{
	namespace DLL430
	{
		class FetHandleV3;
		class DeviceHandleV3;
		class DeviceChainInfo;
		        
		class DeviceHandleManagerV3 : public DeviceHandleManager
		{
		public:
			DeviceHandleManagerV3 (FetHandleV3*);
			~DeviceHandleManagerV3 ();

			DeviceChainInfoList* getDeviceChainInfo (bool update);

			DeviceHandleV3* createDeviceHandle (DeviceChainInfoList::iterator& device, uint32_t deviceCode); // Used with a chain
			void destroyDeviceHandle(DeviceHandle* handle);
		protected:

		private:
			FetHandleV3* parent;
			DeviceChainInfoList list;
		};

	};
};

#endif /* DLL430_DEVICEHANDLEMANAGERV3_H */
