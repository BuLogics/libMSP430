/*
 * FetHandle.h
 *
 * Interface for fet handle definition.
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
#ifndef DLL430_FetHandle_H
#define DLL430_FetHandle_H

#include <DLL430/VersionInfo.h>
#include <DLL430/ConfigManager.h>
#include <DLL430/MemoryManager.h>
#include <DLL430/DebugManager.h>
#include <DLL430/DeviceHandleManager.h>

#include <DLL430_SYMBOL.h>

#include <boost/function.hpp>

namespace TI
{
	namespace DLL430
	{
		//Commands for HilCommand macro
		enum HIL_COMMAND {
			HIL_CMD_RESET_JTAG_TAP,
			HIL_CMD_OPEN,
			HIL_CMD_CONNECT,
			HIL_CMD_CLOSE,
			HIL_CMD_JTAG_IR,
			HIL_CMD_JTAG_DR,
			HIL_TEST_REG,
			HIL_TEST_REG_3V,
			HIL_CMD_QUERY_JSTATE,
			HIL_CMD_WRITE_JMB,
			HIL_CMD_READ_JMB,
		};

		//Pin definitions for setJtagPin
		enum JTAG_PIN 
		{
			JTAG_PIN_TMS	= 0,
			JTAG_PIN_TDI	= 1,
			JTAG_PIN_TDO	= 2,
			JTAG_PIN_TCK	= 3,
			JTAG_PIN_SELTST = 5,
			JTAG_PIN_RST	= 6,
			JTAG_PIN_TST	= 8
		};

		typedef boost::function4<void, uint32_t, uint32_t, uint32_t, uint32_t> NotifyCallback;

		/** \brief a handle to act on a target
		 *
		 * This class acts as a container for the different managers
		 * that contain the functional parts for target device interactions.
		 */
		class DLL430_SYMBOL FetHandle
		{
		public:
			virtual ~FetHandle () {};
			
			/** \brief the DLL version of this handle
			 *
			 * \return the version information
			 */
			virtual const VersionInfo& getDllVersion () const = 0;

			/** \brief the hardware information as given back from the UIF (emty if LPT)
			 *
			 * \return the version information
			 */
			virtual std::vector<uint8_t> * getHwVersion() =0;

			/** \brief the firmware version information as given back from the UIF (emty if LPT)
			 *
			 * \return the version information
			 */
			virtual std::vector<uint8_t> * getSwVersion() =0;

			/** \brief the configuration manager
			 *
			 * \return pointer to a configuration manager instance
			 */
			virtual ConfigManager* getConfigManager () = 0;

			/** \brief the device handle manager
			 *
			 * \return pointer to a device handle manager instance
			 */			
			virtual DeviceHandleManager* getDeviceHandleManager () = 0;

			/** \brief configure and send the kill command
			 *
			 * \return true on success
			 */	
			virtual bool kill(uint8_t respId) = 0;

			/** \brief pauses a command that is running in a loop in firmware
			 *
			 * \return true on success
			 */	
			virtual bool pauseLoopCmd(unsigned long respId) = 0;

			/** \brief resumes a paused a command loop
			 *
			 * \return true on success
			 */
			virtual bool resumeLoopCmd(unsigned long respId) = 0;

			/** \brief return if there is any communication to UIF
			 *
			 * \return true on success
			 */	
			virtual bool hasCommunication() = 0;
			
			/** \brief install a callback for system messages (USB plug events, etc)
			 *
			 * \param notifyCallback this will be called on enexpected evente
			 */			
			virtual void addSystemNotifyCallback(const NotifyCallback& notifyCallback) = 0;

			/** \brief shutdown communication with Fet
			 *
			 */			
			virtual void shutdown() = 0;

			/** \brief send low level hil commands
			 *
			 * \param command the HIL operation to perform
			 * \param data the instruction or data to shift
			 * \param bitSize the number of bits for data shifts
			 */
            virtual bool sendHilCommand(HIL_COMMAND command, uint32_t data = 0) = 0;
			
			/** \brief send low level jtag shifts
			 *
			 * \param shiftType the type of jtag shift to perform
			 * \param data the instruction or data to shift
			 * \param bitSize the number of bits for data shifts
			 */
			virtual uint64_t sendJtagShift(HIL_COMMAND shiftType, uint64_t data, long bitSize = 16) = 0;

			/** \brief send low level hil commands
			 *
			 * \param pin the pin to set
			 * \param state set the pin high (1) or low(0)
			 */
			virtual bool setJtagPin(JTAG_PIN pin, bool state) = 0;
		};

	};
};

#endif /* DLL430_FetHandle_H */
