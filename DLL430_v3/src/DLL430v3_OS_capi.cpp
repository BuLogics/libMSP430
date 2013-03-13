/*
 * DLL430v3_OS_capi.cpp
 *
 * C API implementation for open source edition.
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

#include "DLL430_OldApiV3.h"
#include <cstring>

DLL430_OldApi* DLL430_CurrentInstance = 0;
#if defined(_WIN32) || defined (_WIN64)
#define ENTRY_FUNCTION bool
#define EXIT_FUNCTION void
#elif defined (UNIX)
#define ENTRY_FUNCTION bool __attribute__ ((constructor))
#define EXIT_FUNCTION void __attribute__ ((destructor))
#endif

ENTRY_FUNCTION oldapi_INIT()
{
	DLL430_CurrentInstance = 0;
	return true;
}

EXIT_FUNCTION oldapi_EXIT()
{
	if (DLL430_CurrentInstance != 0)
		delete DLL430_CurrentInstance;
}

static STATUS_T toStatus (bool b)
{
	return (b? STATUS_OK: STATUS_ERROR);
}

static void createInstance ()
{
	if (DLL430_CurrentInstance == 0)
		DLL430_CurrentInstance = new DLL430_OldApiV3;
}

STATUS_T WINAPI MSP430_GetNumberOfUsbIfs(long* Number)
{
	createInstance();
	long count;
	
	if (!DLL430_CurrentInstance->GetNumberOfUsbIfs(&count))
		return toStatus(false);
	
	if(Number)
	{
		*Number = count;
	}
	return toStatus(true);
}

STATUS_T WINAPI MSP430_GetNameOfUsbIf(long Idx, char** Name, long* Status)
{
	createInstance();
	
	return toStatus(DLL430_CurrentInstance->GetNameOfUsbIf(Idx, Name, Status));
}

STATUS_T WINAPI MSP430_Initialize(char* port, long* version)
{
	createInstance();

	if((version==NULL)||(port==NULL))
		return toStatus(false);

	return toStatus(DLL430_CurrentInstance->Initialize(port, version));
}

STATUS_T WINAPI MSP430_SET_SYSTEM_NOTIFY_CALLBACK(SYSTEM_NOTIFY_CALLBACK parSystemNotifyCallback)
{
	if(!DLL430_CurrentInstance)
		return STATUS_OK;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SetSystemNotfyCallback(parSystemNotifyCallback));
}

STATUS_T WINAPI MSP430_OpenDevice(CHAR* Device, CHAR* Password, LONG PwLength, LONG DeviceCode, LONG setId)
{
	if(!DLL430_CurrentInstance)
		return STATUS_OK;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->OpenDevice(Device,Password,PwLength,DeviceCode,setId));
}

STATUS_T WINAPI MSP430_GetFoundDevice(CHAR* FoundDevice, LONG count)
{
	if(!DLL430_CurrentInstance)
		return STATUS_OK;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->GetFoundDevice(FoundDevice,count));
}

STATUS_T WINAPI MSP430_Close(long vccOff)
{
	if(!DLL430_CurrentInstance)
		return STATUS_OK;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Close(vccOff));
}

STATUS_T WINAPI MSP430_Configure(long mode, long value)
{
	enum CONFIG_MODE m = (enum CONFIG_MODE)mode;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Configure(m, value));
}

long WINAPI MSP430_Error_Number(void)
{
	if (!DLL430_CurrentInstance)
		return 0;
	return DLL430_CurrentInstance->Error_Number();
}

const char* WINAPI MSP430_Error_String(long errorNumber)
{
	if (!DLL430_CurrentInstance)
		return 0;
	return DLL430_CurrentInstance->Error_String(errorNumber);
}

STATUS_T WINAPI MSP430_GetJtagID(long* JtagId)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->GetJtagID(JtagId));
}

STATUS_T WINAPI MSP430_Device(long localDeviceId, char* buffer, long count)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Device(localDeviceId, buffer, count));
}

STATUS_T WINAPI MSP430_VCC(long voltage)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->VCC(voltage));
}

STATUS_T WINAPI MSP430_GetCurVCCT(long* voltage)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->GetCurVCCT(voltage));
}

STATUS_T WINAPI MSP430_GetExtVoltage(long* voltage, long* state)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->GetExtVoltage(voltage, state));
}

STATUS_T WINAPI MSP430_Erase(long type, long address, long length)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Erase(type, address, length));
}

STATUS_T WINAPI MSP430_Memory(long address, char* buffer, long count, long rw)
{
	uint8_t* buf = reinterpret_cast<uint8_t*>(buffer);

	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Memory(address, buf, count, rw));
}

STATUS_T WINAPI MSP430_Secure(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Secure());
}

STATUS_T WINAPI MSP430_ReadOutFile(long wStart, long wLength, char* lpszFileName, long iFileType)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->ReadOutFile(wStart, wLength, lpszFileName, iFileType));
}

STATUS_T WINAPI MSP430_ProgramFile(char* File, long eraseType, long verifyMem)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->ProgramFile(File, eraseType, verifyMem));
}

STATUS_T WINAPI MSP430_VerifyFile(char* File)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->VerifyFile(File));
}

STATUS_T WINAPI MSP430_VerifyMem(long StartAddr, long Length, char* DataArray)
{
	uint8_t* data = reinterpret_cast<uint8_t*>(DataArray);
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->VerifyMem(StartAddr, Length, data));
}

STATUS_T WINAPI MSP430_EraseCheck(long StartAddr, long Length)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EraseCheck(StartAddr, Length));
}

STATUS_T WINAPI MSP430_Reset(long method, long execute, long releaseJTAG)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Reset(method, execute, releaseJTAG));
}

STATUS_T WINAPI MSP430_ExtRegisters(long address, char * buffer, long count, long rw)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->ExtRegisters(address,(uint8_t*)buffer,count,rw));
}

STATUS_T WINAPI MSP430_Registers(long* registers, long mask, long rw)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Registers(registers, mask, rw));
}

STATUS_T WINAPI MSP430_Register(long* reg, long regNb, long rw)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Register(reg, regNb, rw));
}

STATUS_T WINAPI MSP430_Run(long mode, long releaseJTAG)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Run(mode, releaseJTAG));
}

STATUS_T WINAPI MSP430_State(long* state, long stop, long* pCPUCycles)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->State(state, stop, pCPUCycles));
}

STATUS_T WINAPI MSP430_CcGetClockNames(long localDeviceId, EemGclkCtrl_t** CcClockNames)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->CcGetClockNames(localDeviceId, CcClockNames));
}

STATUS_T WINAPI MSP430_CcGetModuleNames(long localDeviceId, EemMclkCtrl_t** CcModuleNames)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->CcGetModuleNames(localDeviceId, CcModuleNames));
}

STATUS_T WINAPI MSP430_EEM_Init(
	MSP430_EVENTNOTIFY_FUNC callback,
	long clientHandle,
	MessageID_t* pMsgIdBuffer
)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_Init(callback, clientHandle, pMsgIdBuffer));
}

STATUS_T WINAPI MSP430_EEM_SetBreakpoint(uint16_t* pwBpHandle, BpParameter_t* pBpBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_SetBreakpoint(pwBpHandle, pBpBuffer));
}

STATUS_T WINAPI MSP430_EEM_GetBreakpoint(uint16_t wBpHandle, BpParameter_t* pBpDestBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetBreakpoint(wBpHandle, pBpDestBuffer));
}

STATUS_T WINAPI MSP430_EEM_SetCombineBreakpoint(
	CbControl_t CbControl,
	uint16_t wCount,
	uint16_t* pwCbHandle,
	uint16_t* pawBpHandle
)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_SetCombineBreakpoint(CbControl, wCount, pwCbHandle, pawBpHandle));
}

STATUS_T WINAPI MSP430_EEM_GetCombineBreakpoint(
	uint16_t wCbHandle,
	uint16_t* pwCount,
	uint16_t* pawBpHandle
)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetCombineBreakpoint(wCbHandle, pwCount, pawBpHandle));
}

STATUS_T WINAPI MSP430_EEM_SetTrace(TrParameter_t* pTrBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_SetTrace(pTrBuffer));
}

STATUS_T WINAPI MSP430_EEM_GetTrace(TrParameter_t* pTrDestBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetTrace(pTrDestBuffer));
}

STATUS_T WINAPI MSP430_EEM_ReadTraceBuffer(TraceBuffer_t* pTraceBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_ReadTraceBuffer(pTraceBuffer));
}

DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_ReadTraceData(TraceBuffer_t* pTraceBuffer, ULONG* pulCount)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_ReadTraceData(pTraceBuffer, pulCount));
}

STATUS_T WINAPI MSP430_EEM_RefreshTraceBuffer(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_RefreshTraceBuffer());
}

STATUS_T WINAPI MSP430_EEM_SetVariableWatch(VwEnable_t VwEnable)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_SetVariableWatch(VwEnable));
}

STATUS_T WINAPI MSP430_EEM_SetVariable(uint16_t* pVwHandle, VwParameter_t* pVwBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_SetVariable(pVwHandle, pVwBuffer));
}

STATUS_T WINAPI MSP430_EEM_GetVariableWatch(VwEnable_t* pVwEnable, VwResources_t* paVwDestBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetVariableWatch(pVwEnable, paVwDestBuffer));
}

STATUS_T WINAPI MSP430_EEM_SetClockControl(CcParameter_t* pCcBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_SetClockControl(pCcBuffer));
}

STATUS_T WINAPI MSP430_EEM_GetClockControl(CcParameter_t* pCcDestBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetClockControl(pCcDestBuffer));
}

STATUS_T WINAPI MSP430_EEM_SetSequencer(SeqParameter_t* pSeqBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_SetSequencer(pSeqBuffer));
}

STATUS_T WINAPI MSP430_EEM_GetSequencer(SeqParameter_t* pSeqDestBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetSequencer(pSeqDestBuffer));
}

STATUS_T WINAPI MSP430_EEM_ReadSequencerState(SeqState_t* pSeqState)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_ReadSequencerState(pSeqState));
}

STATUS_T WINAPI MSP430_FET_SelfTest(long count, uint8_t* buffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_SelfTest(count, buffer));
}

STATUS_T WINAPI MSP430_FET_SetSignals(long SigMask, long SigState)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_SetSignals(SigMask, SigState));
}

STATUS_T WINAPI MSP430_FET_Reset(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_Reset());
}

STATUS_T WINAPI MSP430_FET_I2C(long address, char* buffer, long count, long rw)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_I2C(address, reinterpret_cast<uint8_t*>(buffer), count, rw));
}

STATUS_T WINAPI MSP430_FET_EnterBootloader(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_EnterBootloader());
}

STATUS_T WINAPI MSP430_FET_ExitBootloader(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_ExitBootloader());
}

STATUS_T WINAPI MSP430_FET_GetFwVersion(long* version)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_GetFwVersion(version));
}

STATUS_T WINAPI MSP430_FET_GetHwVersion(uint8_t** version, long* count)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_GetHwVersion(version, count));
}

STATUS_T WINAPI MSP430_FET_FwUpdate(
	char* lpszFileName,
	DLL430_FET_NOTIFY_FUNC callback,
	long clientHandle
)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_FwUpdate(lpszFileName, callback, clientHandle));
}

void WINAPI MSP430_HIL_ResetJtagTap(void)
{
	 if (DLL430_CurrentInstance)
		 DLL430_CurrentInstance->HIL_ResetJtagTap();
}

STATUS_T WINAPI MSP430_HIL_Open(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_Open());
}

STATUS_T WINAPI MSP430_HIL_Connect(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_Connect());
}

STATUS_T WINAPI MSP430_HIL_Close(LONG vccOff)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_Close(vccOff));
}

LONG WINAPI MSP430_HIL_JTAG_IR(LONG instruction)
{
	uint64_t retValue = (uint64_t)-1;
	if (DLL430_CurrentInstance)
		retValue = DLL430_CurrentInstance->HIL_JTAG_IR(instruction);

	return (LONG)retValue;
}

LONG WINAPI MSP430_HIL_JTAG_DR(LONG data, LONG bits)
{
	uint64_t retValue = (uint64_t)-1;
	if (DLL430_CurrentInstance)
		retValue = DLL430_CurrentInstance->HIL_JTAG_DR(data, bits);

	return (LONG)retValue;
}

LONGLONG WINAPI MSP430_HIL_JTAG_DRX(LONGLONG data, LONG bits)
{
	uint64_t retValue = (uint64_t)-1;
	if (DLL430_CurrentInstance)
		retValue = DLL430_CurrentInstance->HIL_JTAG_DR(data, bits);

	return (LONGLONG)retValue;
}

STATUS_T WINAPI MSP430_HIL_TCK(LONG state)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_TCK(state));
}

STATUS_T WINAPI MSP430_HIL_TMS(LONG state)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_TMS(state));
}

STATUS_T WINAPI MSP430_HIL_TDI(LONG state)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_TDI(state));
}

STATUS_T WINAPI MSP430_HIL_RST(LONG state)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_RST(state));
}

STATUS_T WINAPI MSP430_HIL_TST(LONG state)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_TST(state));
}
