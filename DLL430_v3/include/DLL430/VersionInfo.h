/*
 * VersionInfo.h
 *
 * Creates string out of version number.
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
#ifndef DLL430_VERSIONINFO_H
#define DLL430_VERSIONINFO_H

#include <inttypes.h>

#include <DLL430_SYMBOL.h>
namespace TI
{
	namespace DLL430
	{

		/** \brief version information */
		class DLL430_SYMBOL VersionInfo
		{
		public:
			VersionInfo (uint8_t major, uint8_t minor = 0, uint8_t patch = 0, uint16_t flavor = 0);
			virtual ~VersionInfo ();
			
			/** \brief combined value of major, minor, patch level and flavor
			 *
			 * This combined version value is compatible to the version values of the DLLv2.
			 * For all other cases, the other functions should be preferred.
			 *
			 * \return the combined version value
			 */
			virtual uint32_t get () const;
			
			/** \brief return the major version number
			 *
			 * \return the major version value
			 */
			virtual uint8_t getMajor () const { return this->imajor; };

			/** \brief return the minor version number
			 *
			 * \return the minor version value
			 */
			virtual uint8_t getMinor () const { return this->iminor; };

			/** \brief return the patch level version number
			 *
			 * \return the patch level version value
			 */
			virtual uint8_t getPatchLevel () const { return this->patch; };

			/** \brief return the flavor version number
			 *
			 * \return the flavor version value
			 */
			virtual uint16_t getFlavor () const { return this->flavor; };
			
			virtual bool operator< (const VersionInfo& v) { return v.get() < this->get(); };
			virtual bool operator<= (const VersionInfo& v) { return v.get() <= this->get(); };
			virtual bool operator== (const VersionInfo& v) { return v.get() == this->get(); };
			virtual bool operator>= (const VersionInfo& v) { return v.get() >= this->get(); };
			virtual bool operator> (const VersionInfo& v) { return v.get() > this->get(); }; 

		private:
			uint8_t imajor;
			uint8_t iminor;
			uint8_t patch;
			uint16_t flavor;
		};

	};
};

#endif /* DLL430_VERSIONINFO_H */
