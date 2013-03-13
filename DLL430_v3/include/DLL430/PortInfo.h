/*
 * PortInfo.h 
 *
 * Information about a physical communication port.
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
#ifndef DLL430_PORTINFO_H
#define DLL430_PORTINFO_H

#include <DLL430_SYMBOL.h>

#include <map>
#include <string>

namespace TI
{
	namespace DLL430
	{

		/** \brief information about a port
		 *
		 * This class allows to identify and select a port by its name.
		 * Ports have a prefix like HID (for USB device using the
		 * Human Interface Devices protocol) or LPT (for devices
		 * using the parallel port) and a number as postfix.
		 */
		class DLL430_SYMBOL PortInfo
		{
			PortInfo(const PortInfo&);
			void operator=(const PortInfo&);
		public:
			enum Status{freeForUse, inUseByAnotherInstance};
			PortInfo(){}
			virtual ~PortInfo () {};
			
			/** \brief get the name of the port
			 *
			 * \return pointer to the port name
			 */
			virtual const char* getName () = 0;

			/** \brief get the name of the interface
			 *
			 * \return pointer to the port name
			 */
			virtual const char* getInterfaceName () = 0;

			/** \brief open an interface to an external (USB) device
			 *
			 * \return true on success
			 */
			virtual bool open () = 0;

			/** \brief close an interface to an external (USB) device
			 *
			 * \return true on success
			 */
			virtual bool close () = 0;

			/** \brief return the status of the port
			 *
			 * \return status
			 */
			virtual long getStatus () = 0;
		};

		typedef std::map<std::string,PortInfo*> PortMap;
	};
};

#endif /* DLL430_PORTINFO_H */
