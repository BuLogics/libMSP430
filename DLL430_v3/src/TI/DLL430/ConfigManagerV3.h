/*
 * ConfigManagerV3.h
 *
 * Functionality for configuring target device.
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
#ifndef DLL430_CONFIGMANAGERV3_H
#define DLL430_CONFIGMANAGERV3_H

#include <DLL430/ConfigManager.h>
#include <inttypes.h>

#include "HalExecCommand.h"
#include "FileFuncImpl.h"

namespace TI
{
	namespace DLL430
	{
		class FetHandleV3;
		class FileFuncImpl;

		class ConfigManagerV3 : public ConfigManager
		{
		public:
			ConfigManagerV3 (FetHandleV3*);
			~ConfigManagerV3 ();

			bool init ();
			
			VersionInfo getBiosVersion () const;

			bool setDeviceVcc (uint16_t vcc);
			
			uint16_t getDeviceVcc() const;
				
			void setJtagMode (ConfigManager::jtagMode mode);
			ConfigManager::jtagMode getInterfaceMode() const;
			uint16_t getExternalVcc() const;
			uint16_t start ();
			uint16_t start (const std::string& pwd, uint32_t deviceCode);
			bool stop ();
			bool checkCoreVersion();
			
			bool reset (bool vcc = false, uint16_t pin = 0, uint16_t JtagId = 0);
			
			bool firmWareUpdate(const char *fname, FetNotifyCallback callback);

			void setPassword(const string& pwd);
			bool setDeviceCode(uint32_t deviceCode);
			bool setJtagSpeed (JTAG_4WIRE_SPEED speed);

			bool freqCalibrationEnabled() const;
			
			bool totalErase(); 

		private:
			FetHandleV3* parent;
			uint16_t vcc;
			enum ConfigManager::jtagMode mode;

			HalExecCommand updateCmd;

			bool upInit(unsigned char level);
			bool upErase(const FileFuncImpl& firmware);
			bool upWrite(const FileFuncImpl& firmware, FetNotifyCallback callback);
			bool upRead(const FileFuncImpl& firmware);
			bool upCoreErase();
			bool upCoreWrite();
			bool upCoreRead();
			std::string password;
			uint32_t deviceCode;
			bool freqCalibration;
		};

	};
};

#endif /* DLL430_CONFIGMANAGERV3_H */
