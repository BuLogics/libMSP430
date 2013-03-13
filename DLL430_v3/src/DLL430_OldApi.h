/*
 * DLL430_OldApi.h
 *
 * Old API interface for IAR.
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
#ifndef DLL430_OLDAPI_H
#define DLL430_OLDAPI_H

#include "MSP430.h"				// changed public path to local path
#include "MSP430_Debug.h"		// avoid mix of v2 and v3 API header
#include "MSP430_EEM.h"
#include "MSP430_FET.h"

#include <inttypes.h>
#include <string.h>

class DLL430_OldApi
{
public:
	virtual ~DLL430_OldApi () {};
	
	virtual bool GetNumberOfUsbIfs(long* Number) = 0;
	virtual bool GetNameOfUsbIf(long Idx, char** Name, long* Status) = 0;
	virtual bool Initialize(char* port, long* version) = 0;
	virtual bool SetSystemNotfyCallback(SYSTEM_NOTIFY_CALLBACK parSystemNotifyCallback) = 0;
	virtual bool OpenDevice(char* Device, char* Password, long PwLength, long DeviceCode, long setId) = 0;
	virtual bool GetFoundDevice(char* FoundDevice, long count) = 0;
	virtual bool Close(long vccOff) = 0;
	virtual bool Configure(enum CONFIG_MODE mode, long value) = 0;
	virtual long Error_Number(void) = 0;
	virtual const char* Error_String(long errorNumber) = 0;
	virtual bool GetJtagID(long* JtagId) = 0;
	virtual bool Identify(char* buffer, long count, long setId) = 0;
	virtual bool Device(long localDeviceId, char* buffer, long count) = 0;
	virtual bool VCC(long voltage) = 0;
	virtual bool GetCurVCCT(long* voltage) = 0;
	virtual bool GetExtVoltage(long* voltage, long* state) = 0;
	virtual bool Erase(long type, long address, long length) = 0;
	virtual bool Memory(long address, uint8_t* buf, long count, long rw) = 0;
	virtual bool Secure(void) = 0;
	virtual bool ReadOutFile(long wStart, long wLength, char* lpszFileName, long iFileType) = 0;
	virtual bool ProgramFile(char* File, long eraseType, long verifyMem) = 0;
	virtual bool VerifyFile(char* File) = 0;
	virtual bool VerifyMem(long StartAddr, long Length, uint8_t* DataArray) = 0;
	virtual bool EraseCheck(long StartAddr, long Length) = 0;
	virtual bool Reset(long method, long execute, long releaseJTAG) = 0;
	virtual bool Registers(long* registers, long mask, long rw) = 0;

	virtual bool ExtRegisters(long address, uint8_t * buffer,long count, long rw) = 0;
	
	virtual bool Register(long* reg, long regNb, long rw) = 0;
	virtual bool Run(long mode, long releaseJTAG) = 0;
	virtual bool State(long* state, long stop, long* pCPUCycles) = 0;

	virtual bool CcGetClockNames(long localDeviceId, EemGclkCtrl_t** CcClockNames) = 0;
	virtual bool CcGetModuleNames(long localDeviceId, EemMclkCtrl_t** CcModuleNames) = 0;
	virtual bool EEM_Init(MSP430_EVENTNOTIFY_FUNC callback, long clientHandle, MessageID_t* pMsgIdBuffer) = 0;
	virtual bool EEM_SetBreakpoint(uint16_t* pwBpHandle, BpParameter_t* pBpBuffer) = 0;
	virtual bool EEM_GetBreakpoint(uint16_t wBpHandle, BpParameter_t* pBpDestBuffer) = 0;
	virtual bool EEM_SetCombineBreakpoint(CbControl_t CbControl, uint16_t wCount, uint16_t* pwCbHandle, uint16_t* pawBpHandle) = 0;
	virtual bool EEM_GetCombineBreakpoint(uint16_t wCbHandle, uint16_t* pwCount, uint16_t* pawBpHandle) = 0;
	virtual bool EEM_SetTrace(TrParameter_t* pTrBuffer) = 0;
	virtual bool EEM_GetTrace(TrParameter_t* pTrDestBuffer) = 0;
	virtual bool EEM_ReadTraceBuffer(TraceBuffer_t* pTraceBuffer) = 0;
	virtual bool EEM_ReadTraceData(TraceBuffer_t* pTraceBuffer, unsigned long *pulCount) = 0;
	virtual bool EEM_RefreshTraceBuffer(void) = 0;
	virtual bool EEM_SetVariableWatch(VwEnable_t VwEnable) = 0;
	virtual bool EEM_SetVariable(uint16_t* pVwHandle, VwParameter_t* pVwBuffer) = 0;
	virtual bool EEM_GetVariableWatch(VwEnable_t* pVwEnable, VwResources_t* paVwDestBuffer) = 0;
	virtual bool EEM_SetClockControl(CcParameter_t* pCcBuffer) = 0;
	virtual bool EEM_GetClockControl(CcParameter_t* pCcDestBuffer) = 0;
	virtual bool EEM_SetSequencer(SeqParameter_t* pSeqBuffer) = 0;
	virtual bool EEM_GetSequencer(SeqParameter_t* pSeqDestBuffer) = 0;
	virtual bool EEM_ReadSequencerState(SeqState_t* pSeqState) = 0;
	virtual bool FET_SelfTest(long count, uint8_t* buffer) = 0;
	virtual bool FET_SetSignals(long SigMask, long SigState) = 0;
	virtual bool FET_Reset(void) = 0;
	virtual bool FET_I2C(long address, uint8_t* buffer, long count, long rw) = 0;
	virtual bool FET_EnterBootloader(void) = 0;
	virtual bool FET_ExitBootloader(void) = 0;
	virtual bool FET_GetFwVersion(long* version) = 0;
	virtual bool FET_GetHwVersion(uint8_t** version, long* count) = 0;
	virtual bool FET_FwUpdate(char* lpszFileName, DLL430_FET_NOTIFY_FUNC callback, long clientHandle) = 0;

	virtual void HIL_ResetJtagTap() = 0;
	virtual bool HIL_Open() = 0;
	virtual bool HIL_Connect() = 0;
	virtual bool HIL_Close(long vccOff) = 0;
	virtual bool HIL_TCK(long state) = 0;
	virtual bool HIL_TMS(long state) = 0;
	virtual bool HIL_TDI(long state) = 0;
	virtual bool HIL_RST(long state) = 0;
	virtual bool HIL_TST(long state) = 0;
	virtual uint64_t HIL_JTAG_IR(long instruction) = 0;
	virtual uint64_t HIL_JTAG_DR(int64_t data, long bits) = 0;
};

#endif /* DLL430_OLDAPI_H */
