/*
 * DebugManager.h 
 *
 * Provides routines for handling a debug session.
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
#ifndef DLL430_DEBUGMANAGER_H
#define DLL430_DEBUGMANAGER_H

#include <inttypes.h>

#include <string>
#include <vector>
#include <map>

#include <DLL430_SYMBOL.h>

namespace TI
{
	namespace DLL430
	{
		enum ClockControlDefaults {
			DefaultClkCntrl = 0x26
		};

		enum ClockControlType {
			GccNone,
			GccStandard,
			GccExtended
		};

		enum DebugControlType {
			FreeRun = 0,
			StopOnTrigger = 0x0020,
			Stopped = 0x0080,
			StorageWrite = 0x0100,
			StorageFull = 0x0200
		};

		enum TriggerCountType {
			RegTrigger,
			Combination
		};

		enum TriggerModeType {
			Options,
			Dma,
            ReadWrite
		};

		enum TraceMode {
			/// Trigger stops the Trace
			TraceHistory = 0,
			/// Trigger starts the Trace
			TraceFuture = 1,
			/// Starts the Trace immediately - stops if buffer is full
			TraceShot = 2,
			/// Collect data only at trigger event
			TraceCollect = 3
		};

		enum TraceAction {
			/// Trace information only on Instruction Fetch cycles
			TraceFetch = 0,
			/// Trace information on all MCLK cycles
			TraceCycle = 1,
		};

		/** \brief debug event target interface */
		class DebugEventTarget
		{
		public:
			enum EventType {
				BreakpointHit,	/**< a breakpoint was reached, device stopped */
				Storage,
				VariableWatch,
				Lpm5Sleep,
				Lpm5Wakeup
			};

			/** \brief called once for each debug event
			 *
			 * \param e the debug event
			 */
			virtual void event(EventType e, uint32_t lParam=0, uint16_t wParam=0) = 0;
		};

		/** \brief manage debug actions on the target device */
		class DLL430_SYMBOL DebugManager
		{
		public:
			/** \brief reestablish JTAG connection after releasing it
			 *
			 * Function will reconnect JTAG and resume any polling 
			 *
			 * \return true if run started successfully, else false
			 */
			virtual bool reconnectJTAG() = 0;

			/** \brief release JTAG control to let the device execute
			 *
			 * The developper must not assume any amount of time until the JTAG pins are
			 * released.
			 *
			 * \param controlType indicates wether waitForEem is activated
			 * \param target call the event method of target (only valid if toBreakpoint is true)
			 * \param eventType bitmask for type of event to react on
			 * \return true if run started successfully, else false
			 */
			virtual bool run (uint16_t controlType, DebugEventTarget* target = 0, bool releaseJTAG=false) = 0;

			/** \brief stop the device
			 *
			 * If a command is running in the FET, this kills it if possible. It also
			 * starts and syncs JTAG.
			 *
			 * \return true if stopped successfully, else false
			 */
			virtual bool stop (bool jtagWasReleased = false) = 0;
			
			/** \brief do a single step
			 *
			 * A SingleStep is performed on the target device. When this function returns,
			 * the device is already stopped. Prior to using this function, the PC must be
			 * set the CPU memory area. The updated value can also be read from there.
			 *
			 * \param cycles the number of device target CPU cycles of the instruction
			 * \return true on successful action, else false
			 */
			virtual bool singleStep (uint32_t* cycles = 0) = 0;

			/** \brief return the clock control level of device
			 *
			 * the returned value depends on the identification of the device
			 * and the database
			 * 
			 * \return level of clock control (1-3)
			 */
			virtual uint8_t getClockControl() const = 0;

			/** \brief return the general clock control setting
			 *
			 * return the 16Bit control value
			 * 
			 * \return control value
			 */
			virtual uint16_t getClockControlSetting() const =0;

			/** \brief set the general clock control setting
			 *
			 * set the 16Bit control value
			 */
			virtual void setClockControlSetting(uint16_t clkcntrl)=0;

			/** \brief return the default clock control module setting
			 *
			 * return the 16Bit control value
			 * 
			 * \return control value
			 */
			virtual uint16_t getClockModuleDefaultSetting() const =0;

			/** \brief return the clock control module setting
			 *
			 * return the 16Bit control value
			 * 
			 * \return control value
			 */
			virtual uint16_t getClockModuleSetting() const =0;

			/** \brief set the clock control module setting
			 *
			 * set the 16Bit control value
			 */
			virtual void setClockModuleSetting(uint16_t modules)=0;

			/** \brief return module strings
			 *
			 * the returned value depends on the identification of the device
			 * and the database
			 * 
			 * \param n pointer to given value filled with number of strings 
			 *
			 * \return pointer to n strings
			 */
			virtual char ** getModuleStrings(uint32_t * n) const = 0;

			/** \brief return clock strings
			 *
			 * the returned value depends on the identification of the device
			 * and the database
			 * 
			 * \param n pointer to given value filled with number of strings 
			 *
			 * \return pointer to n strings
			 */
			virtual char ** getClockStrings(uint32_t * n) const = 0;

			/** \brief write a value directly to an EEM register
			 *
			 * implements the corresponding API call
			 * 
			 * \param reg register to write
			 * \param value value to write into register
			 * 
			 * \return true if successfull, otherwise false
			 */
			virtual bool eemWriteRegister(uint32_t reg, uint32_t value)= 0;

			/** \brief read a value directly from an EEM register
			 *
			 * implements the corresponding API call
			 * 
			 * \param reg register to write
			 * \param value the pointer to a existing variable which is filled with register content
			 * 
			 * \return true if successfull, otherwise false
			 */
			virtual bool eemReadRegister(uint32_t reg, uint32_t* buffer)= 0;

			/** \brief init the EEM-Register
			 *
			 * depending on the target hardware, the existing EEM-register are
			 * set to default (0)
			 * 
			 * \return true if data is successfully written to device, else false
			 */
			virtual bool initEemRegister() = 0;

			/** \brief Enable eem polling loop
			 *
			 * \param mask of events to poll for
			 * 
			 * \return true if successfull, otherwise false
			 */
			virtual bool activatePolling(uint16_t) = 0;

			/** \brief Enable polling of JState register
			 *
			 * \param cb Callback target to handle jstate changes
			 * 
			 * \return true if successfull, otherwise false
			 */
			virtual bool activateJStatePolling(DebugEventTarget * cb) = 0;

			/** \brief Check if device is in low power mode x.5
			 *
			 * Queries the device and returns the current state
			 * 
			 * \return true if device is in LPMx.5
			 */
			virtual bool queryLpm5State() = 0;

			/** \brief Return current low power mode x.5 state
			 *
			 * Returns the last reported state without actively querying
			 * 
			 * \return true if device is in LPMx.5
			 */
			virtual bool isDeviceInLpm5() = 0;

			/** \brief set the opcode parameter
			 *
			 * needed for ID_RestoreContext_ReleaseJtag and set by SET_MDB_BEFORE_RUN
			 * 
			 * \param value opcode to be used by ID_RestoreContext_ReleaseJtag
			 * 
			 * \return true if successfull, otherwise false
			 */
			virtual void setOpcode(uint16_t value)=0;

			/** \brief call macro ID_SyncJtag_Conditional_SaveContext
			 *
			 * calles the macro ID_SyncJtag_Conditional_SaveContext and saves
			 * the returned register values
			 *
			 * \return true on success
			 */
			virtual bool saveContext()=0;

			/** \brief get the current response ID of a loop response (breakpoint, etc.)
			 *
			 * synchronize the call and response of a loop command
			 *
			 * \return true on success
			 */
			virtual uint8_t getRunEemResponseId()=0;
			/** \brief enable/disable low power mode debugging
			 *
			 * synchronize the call and response of a loop command
			 */
			virtual void setLpmDebugging(bool enable)=0;

			/** \brief Check for low power mode debugging
			 *
			 * return true if power mode debugging is enabled
			 */
			virtual bool getLpmDebugging()=0;

			/** \brief Pause all polling loops
			 *
			 *
			 */
			virtual void pausePolling()=0;
			
			/** \brief Resume all polling loops
			 *
			 *
			 */
			virtual void resumePolling()=0;

			/** \brief sync JTAG if an externl wakeup event happends
			 *
			 *
			 */
			virtual bool syncDeviceAfterLPMx5() = 0;

			/** \brief Retrieve current cycle counter value
			 *
			 *
			 */
			virtual uint64_t getCycleCounterValue() = 0;

			/** \brief Reset cycle counter value
			 *
			 *
			 */
			virtual void resetCycleCounterValue() = 0;

			/** \brief Start polling loop for state storage events on UIF
			 *
			 *
			 */
			virtual bool startStoragePolling() = 0;
			
			/** \brief Stop polling loop for state storage events on UIF
			 *
			 *
			 */
			virtual bool stopStoragePolling() = 0;
		};

	};
};

#endif /* DLL430_DEBUGMANAGER_H */
