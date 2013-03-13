/*
 * DeviceInfo.h 
 *
 * Data of device currently under control.
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
#ifndef DLL430_DEVICEINFO_H
#define DLL430_DEVICEINFO_H

#include <inttypes.h>
#include <vector>
#include <boost/array.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include "IoChannel.h"
#include "MemoryAreaBase.h"
#include "FuncletCode.h"

namespace TI
{
	namespace DLL430
	{
		enum ClockSystem { BC_1xx, BC_2xx, FLLPLUS, MOD_OSC };

		struct MemoryCreatorBase
		{
			virtual bool isImplemented() const = 0;
			virtual TI::DLL430::MemoryAreaBase* operator()(
				const std::string&, TI::DLL430::DeviceHandleV3*,
				uint32_t, uint32_t, 
				uint32_t, uint32_t, 
				bool, const bool, MemoryManager*, uint8_t
			) const = 0;
		};
		typedef boost::shared_ptr<TI::DLL430::MemoryCreatorBase> MemoryCreatorPtr;

		template<typename T>
		class Property
		{
		protected:
			T value;
		public:
			Property() : value(T()) {}
			Property(const T& val) : value(val) {}
			inline const T& operator()() const { return value; }
			inline void operator()(const T& newVal) { value=newVal; }
		};

		class DeviceInfo
		{
		public:
			static const size_t nrClockModules = 32;
			static const size_t nrUsedClockModules = 16;
			static const size_t nrClockNames = 16;

			typedef boost::array<std::pair<std::string, uint8_t>, nrClockModules> ClockMapping;
			typedef boost::array<std::string, nrClockNames> ClockNames;

			enum memoryType {
				MEMTYPE_FLASH,
				MEMTYPE_ROM,
				MEMTYPE_RAM,
				MEMTYPE_REGISTER
			};
			enum psaType {
				PSATYPE_REGULAR,
				PSATYPE_ENHANCED
			};

			class MemoryCreatorBase;
			struct memoryInfo {
				std::string name;
				memoryType type;
				uint8_t bits;
				uint32_t size;
				uint32_t offset;
				uint32_t seg_size;
				uint32_t banks;
				std::vector<uint8_t> mask; 
				bool mmapped;
				bool isProtected;
				MemoryCreatorPtr memoryCreatorPtr;
			};

			
			typedef boost::ptr_vector<memoryInfo> memoryInfo_list_type;
			typedef std::map<FuncletCode::Type, FuncletCode> funclet_map_type;
			
			DeviceInfo ();
			~DeviceInfo ();

			void setDescription(const char *);
			const char * getDescription() const;

			void addMemoryInfo (memoryInfo*);
			const memoryInfo_list_type& getMemoryInfo () const;
			
			void setObjectId (size_t);
			size_t getObjectId () const;

			void setPsaType (enum psaType);
			enum psaType getPsaType () const;
			
			Property<uint16_t> minVcc;
			Property<uint16_t> maxVcc;
			Property<uint16_t> minFlashVcc;
			Property<uint16_t> minSecureVcc;
			Property<uint16_t> minSecureVpp;
			Property<uint16_t> maxSecureVpp;
			
			Property<bool> hasTestVpp;
			Property<bool> quickMemRead;
			Property<bool> psach;
			Property<bool> b1377;
			Property<bool> hasFram;
			Property<ClockSystem> clockSystem;

			Property<uint32_t> powerTestRegMask;
			Property<uint32_t> testRegEnableLpmx5;
			Property<uint32_t> testRegDisableLpmx5;
			
			Property<uint16_t> powerTestReg3VMask;
			Property<uint16_t> testReg3VEnableLpmx5;
			Property<uint16_t> testReg3VDisableLpmx5;

			Property<bool> noBsl;

			void addFunctionMapping (unsigned long apiId, uint16_t halId);
			const IoChannel::function_map_type& getMap () const;

			void setFuncletMap(const funclet_map_type& map);
			const funclet_map_type& getFuncletMap() const;

			void setPossibleTrigger (uint8_t, uint8_t);
			uint8_t getPossibleTrigger (uint8_t) const;

			void setClockControl (uint8_t);
			uint8_t getClockControl () const;

			void setClockModDefault (uint16_t);
			uint16_t getClockModDefault () const;

			void setStateStorage (uint8_t);
			uint8_t getStateStorage () const;

			void setCycleCounter (uint8_t);
			uint8_t getCycleCounter () const;

			void setCycleCounterOperations (uint8_t);
			uint8_t getCycleCounterOperations () const;

			void setEmulationLevel(uint8_t);
			uint8_t getEmulationLevel () const;

			void setTriggerOptionsModes(uint8_t);
			uint8_t getTriggerOptionsModes() const;

			void setTriggerDmaModes(uint8_t);
			uint8_t getTriggerDmaModes() const;

			void setTriggerReadWriteModes(uint8_t);
			uint8_t getTriggerReadWriteModes() const;

			void setRegTriggerOperations(uint8_t);
			uint8_t getRegTriggerOperations() const;

			void setTriggerMask(uint8_t);
			uint8_t getTriggerMask() const;

			void setMaxSequencerStates(uint8_t states);
			uint8_t getMaxSequencerStates() const;

			//used to fill clockMapping
			ClockMapping& getWritableClockMapping() {return clockMapping;}
			const ClockMapping& getClockMapping() const {return clockMapping;}

			ClockNames& getClockNames() { return clockNames; }
			const ClockNames& getClockNames() const { return clockNames; }

			void setSFll (uint8_t);
			uint8_t getSFll () const;
			
		private:
			size_t objectId;
			memoryInfo_list_type mem;
			enum psaType type;
			IoChannel::function_map_type map;
			funclet_map_type funcletTable;

			const char * description;

			uint8_t clockControl;
			uint16_t mclkcntrl0;
			uint8_t stateStorage;		/**< depth of available state storage modul. (0 means "not present") */
			uint8_t cycleCounter;		/**< number of available cycle counters. (0 means "not present") */
			uint8_t cycleCounterOperations;

			uint8_t emulationLevel;

			uint8_t triggerMask;
			uint8_t triggerOptionsModes;
			uint8_t triggerDmaModes;
			uint8_t triggerReadWriteModes;
			uint8_t possibleTrigger[3];
			uint8_t regTriggerOperations;

			uint8_t maxSequencerStates;

			ClockMapping clockMapping;
			ClockNames clockNames;

			uint8_t sFll;
		};
		typedef boost::shared_ptr<DeviceInfo> DeviceInfoPtr;
	} //namespace DLL430
	
} //namespace TI

#endif /* DLL430_DEVICEINFO_H */
