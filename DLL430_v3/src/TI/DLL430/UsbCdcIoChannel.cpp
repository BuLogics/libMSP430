/*
 * UsbCdcIoChannel.cpp
 *
 * IOChannel via CDC (VCOM) over USB communication.
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

#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS //disabling warnings to use secure c-function versions (e.g. strcpy_s) as this is not compatible with none MS development
#endif
#endif

#include "UsbCdcIoChannel.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include "logging/Logging.h"

#include <boost/thread/thread.hpp>

#if defined(_WIN32) || defined(_WIN64)

extern "C" {
	#include <setupapi.h>
	#include <dbt.h>
}

	static const std::string portPrefix("\\\\.\\");

#else

	#include <unistd.h>
	#include <boost/filesystem.hpp>

	using namespace boost::filesystem;

	static const std::string portPrefix("");

#endif

using namespace TI::DLL430;
using namespace std;
using namespace boost::asio;

#define  XOFF		0x13
#define  XON		0x11
#define  XMASK		0x10
#define  XMASK_OFF	0x03
#define  XMASK_ON	0x01
#define  XMASK_MASK	0x00

#ifndef NDEBUG
#define DB_PRINT
#endif

UsbCdcIoChannel::UsbCdcIoChannel(std::string name, std::string id)
 : inputReportSize(255)
 , actSize(0)
 , expSize(0)
 , name(name)
 , isXoffFlowOn(true)
 , status(freeForUse)
 , ioService(0)
 , port(0)
 , comState(ComStateRcv)
{
	serial = retrieveSerialFromId(id);
	retrieveStatus();
}

UsbCdcIoChannel::~UsbCdcIoChannel ()
{
	this->cleanup();
}


void UsbCdcIoChannel::enumeratePorts (PortMap & portList, bool open)
{
#if defined(_WIN32) || defined(_WIN64)
	typedef boost::array<BYTE, maxBufferLength> RegistryProperty;

	const std::string tiVendorId("VID_2047");
	const std::string cdcClassDev("PID_0010"); ///< product id 0010 means CDC
	const std::string cdcId(UsbDevClass() + '\\' + tiVendorId + '&' + cdcClassDev);
	
	char deviceId[maxBufferLength];

	HDEVINFO hDevInfo = ::SetupDiGetClassDevs(NULL, UsbDevClass().c_str(), 0, DIGCF_PRESENT | DIGCF_ALLCLASSES );
	SP_DEVINFO_DATA devInfoData;
	devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
	
	for (int i = 0; ::SetupDiEnumDeviceInfo(hDevInfo,i,&devInfoData); ++i )
	{	
		deviceId[0]=0;
		BOOL result = ::SetupDiGetDeviceInstanceId(hDevInfo,&devInfoData,deviceId,maxBufferLength,NULL);

		//not TI and/or not CDC
		if( !result || 0 != cdcId.compare( std::string( deviceId).substr( 0, cdcId.size() ) ) )
		{
			continue;
		} 
		else 
		{
			DWORD propertyType = 0;
			 
			RegistryProperty property;
			::SetupDiGetDeviceRegistryProperty(
				hDevInfo, &devInfoData, SPDRP_FRIENDLYNAME,
                &propertyType, &property[0], maxBufferLength, NULL
			);


			std::stringstream sstr;
			for(size_t i = 0; i < property.size(); ++i)
			{
				sstr << property[i];
				if(0 == property[i])
				{
					break;
				}
			}

			const size_t idBegin = sstr.str().find_last_of('(') + 1;
			const size_t idEnd = sstr.str().find_last_of(')');
			assert(idEnd > idBegin);

			const string name = sstr.str().substr(idBegin, idEnd - idBegin);
		
			UsbCdcIoChannel * cdcElem = new UsbCdcIoChannel(name,deviceId);
			portList.insert(std::make_pair(name,cdcElem));
		}
	}
	::SetupDiDestroyDeviceInfoList(hDevInfo);//free resources

#else

	// Implementation based on Debian
	path p(string("/dev/serial/by-id/"));
	if ( exists(p) && is_directory(p) )
	{
		const directory_iterator end;
		for ( directory_iterator it(p); it != end; ++it )
		{
			const string deviceId = it->path().filename().c_str();
			if ( is_symlink(*it) &&
				deviceId.find("usb-Texas_Instruments_Texas_Instruments_MSP430-JTAG_") != string::npos )
			{
				char linkPath[255] = {0};
				const int len = readlink( it->path().string().c_str(), linkPath, sizeof(linkPath) );
				if ( len > 0 )
				{
					string portPath = (linkPath[0] == '/') ? string(linkPath) : p.string() + linkPath;
					portList[portPath] = new UsbCdcIoChannel(portPath, deviceId);
				}
			}
		}
	}
	// Use workaround for distributions without /dev/serial/by-id
	else
	{
		for (int i = 0; i < 16; ++i)
		{
			stringstream portName;
			portName << "ttyACM" << i;
			string portPath = string("/dev/") + portName.str();
			if ( !access(portPath.c_str(), F_OK) )
			{
				portList[portPath] = new UsbCdcIoChannel(portPath, portName.str());
			}
		}
	}
#endif
}

std::string UsbCdcIoChannel::retrieveSerialFromId(const std::string& id)
{
#if defined(_WIN32) || defined(_WIN64)
	std::stringstream sstr(id);

	const size_t idBegin = sstr.str().find_last_of('\\') + 1;

	return sstr.str().substr(idBegin, 16);
#else
	const size_t begin = id.find_last_of('_') + 1;
	const size_t end = id.find_last_of('-');
	return id.substr( begin, end - begin );
#endif
}

long UsbCdcIoChannel::getStatus()
{
 	return status;
}


bool UsbCdcIoChannel::openPort()
{
	const std::string dev = portPrefix + name;
		
	ioService = new boost::asio::io_service;
	port = new boost::asio::serial_port(*ioService);

	if ( boost::system::error_code ec = port->open(dev, ec) )
	{
		int retry = 5;
		while (ec && --retry )
		{
			boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(5));
			ec = port->open(dev, ec);
		}

		if ( ec == boost::system::error_condition(boost::system::errc::permission_denied) )
			status = inUseByAnotherInstance;

		if (ec)
		{
			close();
			return false;
		}
	}
	return true;
}


void UsbCdcIoChannel::retrieveStatus()
{
	status = freeForUse;

	if (!isOpen())
	{
		openPort();
		close();
	}
}


bool UsbCdcIoChannel::open()
{
	if (!isOpen() && !openPort())
	{
		return false;
	}

	status = freeForUse;

	try
	{
		const int baudrate = 460800;

		port->set_option( serial_port::baud_rate( baudrate ) );
		port->set_option( serial_port::flow_control( serial_port::flow_control::none ) );
		port->set_option( serial_port::parity( serial_port::parity::none ) );
		port->set_option( serial_port::stop_bits( serial_port::stop_bits::one ) );
		port->set_option( serial_port::character_size(8) );
	}
	catch (const boost::system::error_code&) {}

	inputBuffer.resize(inputReportSize+3);

	return true;
}

void UsbCdcIoChannel::cleanup () 
{
	if (isOpen())
	{
		boost::system::error_code ec = port->close(ec);
	}
	delete port;
	port = 0;
	delete ioService;
	ioService = 0;
}

bool UsbCdcIoChannel::close () 
{
	cleanup();
	return true;
}

bool UsbCdcIoChannel::isOpen ()
{
	return port && port->is_open();
}

boost::mutex readWriteMutex;

class AsyncTransferHandler
{
public:	
	AsyncTransferHandler(serial_port& port) : 
		port(port), 
		timer(port.get_io_service()), 
		result(RES_NONE), 
		bytesTransfered(0) {}

	int read(unsigned char* buf, size_t bufSize, uint32_t timeout)
	{
		timer.expires_from_now(boost::posix_time::milliseconds(timeout));
		timer.async_wait(boost::bind(&AsyncTransferHandler::onTimeout, this, _1));

		async_read(port, buffer(buf, bufSize),
				 boost::bind(&AsyncTransferHandler::onTransferComplete, this, _1, _2) );

		port.get_io_service().reset();

#if defined(_WIN32) || defined(_WIN64)
		while ( result == RES_NONE )
			port.get_io_service().run_one();

		if (result != RES_COMPLETE)
		{
			readWriteMutex.lock();
			port.cancel();
			readWriteMutex.unlock();
		}

		if (result != RES_TIMEOUT)
			timer.cancel();
#else
		while ( result == RES_NONE )
			port.get_io_service().run();
#endif
		port.get_io_service().stop();

		return (result == RES_COMPLETE) ? static_cast<int>(bytesTransfered) : -1;
	}	

private:
	enum Result { RES_NONE, RES_ERROR, RES_TIMEOUT, RES_COMPLETE };

	serial_port& port;
	deadline_timer timer;
	Result result;
	int bytesTransfered;

	void onTransferComplete(const boost::system::error_code& ec, size_t numBytes)
	{
#ifdef UNIX
		timer.cancel();
#endif
		if ( ec != error::operation_aborted && result == RES_NONE )
			result = ec ? RES_ERROR : RES_COMPLETE;

		bytesTransfered = (result == RES_COMPLETE) ? static_cast<int>(numBytes) : -1;
	}

	void onTimeout(const boost::system::error_code& ec)
	{
#ifdef UNIX
		port.cancel();
#endif
		if( ec != error::operation_aborted && result == RES_NONE )
			result = ec ? RES_ERROR : RES_TIMEOUT;
	}
};

int UsbCdcIoChannel::read(HalResponse& resp, uint32_t timeout)
{
	if (!isOpen())
		return 0;

	if(actSize == 0)
		expSize = 6;

	if (expSize > inputBuffer.size())
		inputBuffer.resize(expSize);

	int bytesRead = -1;
	try
	{
		AsyncTransferHandler readHandler(*port);
		bytesRead = readHandler.read(&inputBuffer[actSize], expSize-actSize, timeout);
	} catch(const std::exception&) {}

	if (bytesRead < 0)
	{
		const std::string dev = portPrefix + name;

		//Important, test port must be gone before close can be called
		boost::system::error_code ec = serial_port(*ioService).open(dev, ec);

		if (ec == boost::system::error_condition(boost::system::errc::no_such_file_or_directory))
		{
			if (comState != ComStateDisconnect)
			{
				this->close();
			}
			comState = ComStateDisconnect;
		}
	}

	if (bytesRead > 0)
	{
		if (actSize == 0)
			expSize = inputBuffer[0] + ( (inputBuffer[0] & 0x01) ? 3 : 4);

		actSize += static_cast<uint16_t>(bytesRead & 0xFFFF);

		//Perform sanity check on message size before trying to read it
		if (expSize < actSize || (expSize % 2) != 0)
		{
			resp.setError(HalResponse::Error_Size);
			expSize = actSize = 0;
			return bytesRead;
		}


		if(expSize && actSize == expSize)
		{
	#ifdef DB_PRINT
			Logging::DefaultLogger().PrintReceiveBuffer(&inputBuffer[0], expSize);
	#endif // DB_PRINT

			const uint16_t msgSize = expSize;
			actSize=0;
			expSize=0;

			resp.setType(inputBuffer[1]);
			resp.setId(inputBuffer[2] & 0x7f); //Don't mask async bit (0x40)
			resp.setIsComplete(inputBuffer[2]);
			
			if(msgSize >= 2)
			{
				resp.append(&inputBuffer[1], inputBuffer[0]);
			}
			
			const uint16_t expectedCRC = createCrc(&inputBuffer[0]);
			const uint16_t actualCRC = inputBuffer[msgSize-2] + (inputBuffer[msgSize-1] << 8);
			if( actualCRC != expectedCRC)
			{
				resp.setError(HalResponse::Error_CRC);
			}

			return bytesRead;
		}
	}
    return 0;
}


enum ComState UsbCdcIoChannel::poll ()
{
	return comState;
}


int UsbCdcIoChannel::write (const uint8_t* payload, size_t len)
{
	if (!isOpen())
		return 0;

	int ret_len = static_cast<int>(len);

	uint8_t report[256] = {0};

	if (payload)
		memcpy(report, payload, len);

	// test for fill byte
	if (!(report[0] & 0x01))
		report[len++] = 0x00;

	//create crc and append it to data
	uint16_t crc = createCrc(report);

	report[len++] = crc & 0x00ff;
	report[len++] = (crc & 0xff00) >> 8;

	size_t n_write = 0;
	uint8_t send_buf[512];

	if (isXoffFlowOn)
	{
		//mask XOFF, XON and MASK in data stream
		size_t j = 0;
		
		for (size_t i = 0; i < len; i++)
		{
			const uint8_t ch = report[i];
			switch (ch)
			{
			case XOFF:
			case XON:
			case XMASK:
				send_buf[j] = XMASK;
				j++;
				send_buf[j] = ch & 0x3;
				break;
			default:
				send_buf[j] = ch;
			}
			j++;
		}
		n_write = j;
	}
	else
	{
		n_write = len;
		memcpy(send_buf, report, n_write);
	}

	readWriteMutex.lock();
	boost::system::error_code ec;
	const size_t nWritten = boost::asio::write(*port, buffer(send_buf, n_write), transfer_all(), ec);
              readWriteMutex.unlock();
	if (nWritten != n_write)
	{
		return 0;
	}

#ifdef DB_PRINT
	Logging::DefaultLogger().PrintSendBuffer(send_buf, n_write);
#endif // DB_PRINT
	return ret_len;
}

const char* UsbCdcIoChannel::getName ()
{
	return name.c_str();
}

const char* UsbCdcIoChannel::getInterfaceName ()
{
	return name.c_str();
}

const std::string UsbCdcIoChannel::getSerial()
{
	return serial;
}
