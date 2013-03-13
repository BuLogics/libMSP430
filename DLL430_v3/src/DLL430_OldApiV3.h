/*
 * DLL430_OldApi.h
 *
 * Old API interface for IAR - V3 implementation.
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
#ifndef DLL430_OLDAPIV3_H
#define DLL430_OLDAPIV3_H

#include "DLL430_OldApi.h"

#include <DLL430/FetHandleManager.h>

#include <DLL430/FileFunc.h>

#include <Logger.h>

#include "EM/BreakpointManager/IBreakpointManager.h"
#include "EM/TriggerCondition/ITriggerCondition.h"
#include "EM/VariableWatch/IWatchedVariable.h"


#include <map>
#include <vector>
#include <deque>

#include <boost/thread/thread.hpp>

//#define DEVICE_DEFAULT_MCLKCNTRL0	0x60d7
#define DEVICE_DEFAULT_MCLKCNTRL0	0x2407
#define DEFAULT_CLKCNTRL			0x0026


class DLL430_OldApiV3 : public DLL430_OldApi
                      , public TI::DLL430::DebugEventTarget
                      , public TI::DLL430::LogTarget
{
public:
	DLL430_OldApiV3 ();
	~DLL430_OldApiV3 ();

	/* this implements the DebugEventTarget interface */
	void event(TI::DLL430::DebugEventTarget::EventType e,  uint32_t lParam=0, uint16_t wParam=0);

	/* this implements the LogTarget interface */
	void log(TI::DLL430::LogTarget::Severity, unsigned int, const char*);

	bool GetNumberOfUsbIfs(long* Number);
	bool GetNameOfUsbIf(long Idx, char** Name, long* Status);
	bool Initialize(char* port, long* version);
	bool SetSystemNotfyCallback(SYSTEM_NOTIFY_CALLBACK parSystemNotifyCallback);
	bool OpenDevice(char* Device, char* Password, long PwLength, long DeviceCode, long setId);
	bool GetFoundDevice(char* FoundDevice, long count);
	bool Close(long vccOff);
	bool Configure(enum CONFIG_MODE mode, long value);
	long Error_Number(void);
	const char* Error_String(long errorNumber);
	bool GetJtagID(long* JtagId);
	bool Identify(char* buffer, long count, long setId);

	// second version of identify, not part of API (OpenDevice)
	bool Identify(char* buffer, long count, long setId, char* Password, long PwLength, long code);

	bool Device(long localDeviceId, char* buffer, long count);
	bool VCC(long voltage);
	bool GetCurVCCT(long* voltage);
	bool GetExtVoltage(long* voltage, long* state);
	bool Erase(long type, long address, long length);
	bool Memory(long address, uint8_t* buf, long count, long rw);
	bool Secure(void);
	bool ReadOutFile(long wStart, long wLength, char* lpszFileName, long iFileType);
	bool ProgramFile(char* File, long eraseType, long verifyMem);
	bool VerifyFile(char* File);
	bool VerifyMem(long StartAddr, long Length, uint8_t* DataArray);
	bool EraseCheck(long StartAddr, long Length);
	bool Reset(long method, long execute, long releaseJTAG);
	
	bool ExtRegisters(long address, uint8_t * buffer,long count, long rw);

	bool Registers(long* registers, long mask, long rw);
	bool Register(long* reg, long regNb, long rw);
	bool Run(long mode, long releaseJTAG);
	bool State(long* state, long stop, long* pCPUCycles);
	bool CcGetClockNames(long localDeviceId, EemGclkCtrl_t** CcClockNames);
	bool CcGetModuleNames(long localDeviceId, EemMclkCtrl_t** CcModuleNames);
	bool EEM_Init(MSP430_EVENTNOTIFY_FUNC callback, long clientHandle, MessageID_t* pMsgIdBuffer);
	bool EEM_SetBreakpoint(uint16_t* pwBpHandle, BpParameter_t* pBpBuffer);
	bool EEM_GetBreakpoint(uint16_t wBpHandle, BpParameter_t* pBpDestBuffer);
	bool EEM_SetCombineBreakpoint(CbControl_t CbControl, uint16_t wCount, uint16_t* pwCbHandle, uint16_t* pawBpHandle);
	bool EEM_GetCombineBreakpoint(uint16_t wCbHandle, uint16_t* pwCount, uint16_t* pawBpHandle);
	bool EEM_SetTrace(TrParameter_t* pTrBuffer);
	bool EEM_GetTrace(TrParameter_t* pTrDestBuffer);
	bool EEM_ReadTraceBuffer(TraceBuffer_t* pTraceBuffer);
	bool EEM_ReadTraceData(TraceBuffer_t* pTraceBuffer, unsigned long *pulCount);
	bool EEM_RefreshTraceBuffer(void);
	bool EEM_SetVariableWatch(VwEnable_t VwEnable);
	bool EEM_SetVariable(uint16_t* pVwHandle, VwParameter_t* pVwBuffer);
	bool EEM_GetVariableWatch(VwEnable_t* pVwEnable, VwResources_t* paVwDestBuffer);
	bool EEM_SetClockControl(CcParameter_t* pCcBuffer);
	bool EEM_GetClockControl(CcParameter_t* pCcDestBuffer);
	bool EEM_SetSequencer(SeqParameter_t* pSeqBuffer);
	bool EEM_GetSequencer(SeqParameter_t* pSeqDestBuffer);
	bool EEM_ReadSequencerState(SeqState_t* pSeqState);
	bool FET_SelfTest(long count, uint8_t* buffer);
	bool FET_SetSignals(long SigMask, long SigState);
	bool FET_Reset(void);
	bool FET_I2C(long address, uint8_t* buffer, long count, long rw);
	bool FET_EnterBootloader(void);
	bool FET_ExitBootloader(void);
	bool FET_GetFwVersion(long* version);
	bool FET_GetHwVersion(uint8_t** version, long* count);
	bool FET_FwUpdate(char* lpszFileName, DLL430_FET_NOTIFY_FUNC callback, long clientHandle);

	void HIL_ResetJtagTap();
	bool HIL_Open();
	bool HIL_Connect();
	bool HIL_Close(long vccOff);
	bool HIL_TCK(long state);
	bool HIL_TMS(long state);
	bool HIL_TDI(long state);
	bool HIL_RST(long state);
	bool HIL_TST(long state);
	uint64_t HIL_JTAG_IR(long instruction);
	uint64_t HIL_JTAG_DR(int64_t data, long bitSize);

	void execThread();
	void execNotifyCallback(SYSTEM_EVENT_MSP);
	void iNotifyCallback(uint32_t msgId, uint32_t wParam, uint32_t lParam, uint32_t clientHandle);

private:
	template<typename T>
	class TableEntry
	{
	public:
		TableEntry() : inUse_(false) {}

		void set(T ptr) { entry_ = ptr; inUse_ = true; }
		void clear() { entry_.reset(); inUse_ = false; }
		void reserveSlot() { entry_.reset(); inUse_ = true; }
		bool inUse() const { return inUse_; }
		const T& value() const { return entry_; }

	private:
		T entry_;
		bool inUse_;
	};

	std::map<long, TableEntry<TI::DLL430::TriggerConditionPtr> > triggers;
	std::map<long, TableEntry<TI::DLL430::TriggerConditionPtr> > traceTriggers;
	std::map<long, TableEntry<TI::DLL430::BreakpointPtr> > breakpoints;
	std::map<uint16_t, vector<uint16_t> > triggerCombinations;
	std::map<uint16_t, TI::DLL430::WatchedVariablePtr> watchedVariables;
	boost::mutex watchedVariablesMutex;
	VwEnable_t varWatch_state;

	TI::DLL430::TriggerConditionPtr triggerConditionFromBpParameter(TI::DLL430::EmulationManagerPtr emuManager, BpParameter_t* bpBuffer);
	void addBreakpointsAndStorage(TI::DLL430::EmulationManagerPtr emuManager, TI::DLL430::TriggerConditionPtr triggerCondition, BpAction_t reactions, uint16_t handle);
	void updateStorageConfiguration(TI::DLL430::EmulationManagerPtr emuManager);

	bool writeToExternalMemory();
	bool lockMemory(const std::string& memoryName, bool lock);

	// Contains all the config data
	std::map<enum CONFIG_MODE, long> config_settings;

    // Container to keep persistent list of port names to be referenced from IDE
	// Pointers to strings in table get invalidated when initializing
	struct port_name {
		char name[64];
		port_name() { memset(name, 0, sizeof(name)); }
	};
	typedef std::deque<port_name> port_names_list_type;
	port_names_list_type port_names;
	
	CcParameter_t clock_control;
	EemMclkCtrl_t moduleNameBuffer;

	std::map<uint16_t, BpParameter_t> bp_storage;
	std::map<uint16_t, VwResources_t> vw_storage;
	TrParameter_t trace_storage;	// trace API parameter buffer
	SeqParameter_t sequencer_control;

	TI::DLL430::FetHandleManager* manager;
	TI::DLL430::FetHandle* handle;

	unsigned int errNum;
	bool v2initCalled;

	std::vector<uint32_t> v_tmp;

	// only 1 FET with one device 
	TI::DLL430::DeviceHandle * singleDevice;
	long selectedJtagMode;

	bool devInfoFilled;
	DEVICE_T devInfo;
	uint32_t devCode;

	struct {
		enum STATE_MODES state;
		bool jtagReleased;
		struct {
			MSP430_EVENTNOTIFY_FUNC func;
			long clientHandle;
			MessageID_t ids;
		} cb;
	} debug;

	SYSTEM_NOTIFY_CALLBACK notifyCallback;
	string lastUsedPort;
};

#endif /* DLL430_OLDAPIV3_H */
