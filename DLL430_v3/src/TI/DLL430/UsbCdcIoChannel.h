/*
 * UsbCdcIoChannel.h
 *
 * IOChannel via CDC (VCOM) over USB communication.
 *
 * Copyright (C) 2007 - 2010 Texas Instruments Incorporated - http://www.ti.com/ 
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

#define _WIN32_WINNT 0x0501

#include "UsbIoChannel.h"
#include "HalResponse.h"

#include <string>
#include <boost/array.hpp>
#include <boost/asio/serial_port.hpp>

namespace TI
{
	namespace DLL430
	{
		class UsbCdcIoChannel : public UsbIoChannel
		{
		public:
			
			UsbCdcIoChannel (std::string name, std::string id);
			~UsbCdcIoChannel ();

			enum ComState poll ();
			bool open();
			bool isOpen ();
			bool close () ;
			long getStatus();

			int read (HalResponse& resp, uint32_t timeout);
			int write (const uint8_t* payload, size_t len);

			static void enumeratePorts (PortMap & list, bool open);

			const char* getName ();
			const char* getInterfaceName ();
			const std::string getSerial();

		private:
			static const size_t maxBufferLength = 128;
			uint16_t inputReportSize;
			std::vector<uint8_t> inputBuffer;
			uint16_t actSize;
			uint16_t expSize;
			std::string name;
			bool isXoffFlowOn;
			Status status;
			boost::asio::io_service* ioService;
			boost::asio::serial_port* port;
			ComState comState;

			std::string serial;

			bool openPort();
			
			void cleanup();
			void retrieveStatus();
			static std::string retrieveSerialFromId(const std::string& id);
			static const std::string UsbDevClass() { return "USB"; }
		};
	}
}
