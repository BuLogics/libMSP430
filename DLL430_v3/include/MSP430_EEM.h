/*
 * MSP430_EEM.h
 *
 * API for accessing EEM functionality of MSP430 library.
 *
 * Copyright (C) 2004 - 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

/** \file MSP430_EEM.h
 
 \brief       This file contains the Application Programming Interface (API)
              to access the Enhanced Emulation Module (EEM) of a MSP430 device
              using the MSP430.DLL.
 
              The DLL provides the following functionalities
              - EEM Hardware Resource Management
              - Caller notifying mechanism using Windows Message Queues
                (Replaced by alternate callback mechanism since version 2.1.2.0)
              - Autonom detection of state changes (e.g. breakpoint hit, single step complete)
 
 \attention   Since the DLL manages all the available HW resources itself it is
              not permitted to mix the old EEM API (EEM_Open(), EEM_Read(),
              EEM_Write()) with the functions provided via this header
              file. The old EEM API functions are only available for compatibility
              reasons. It is up to the caller to make use of the new or the old API.
              As soon as the new EEM API is initialized by MSP430_EEM_Init() functions
              of the old API are executed but will post a warning message as unexpected
              exceptions might occure.
              Integration support from Texas Instruments will only be given for the
              new API provided via this header file.
              Following functions MUST NOT be called any longer when EEM API is used:
              - MSP430_Configure() with parameter 'mode' set to CLK_CNTRL_MODE
              - MSP430_Configure() with parameter 'mode' set to MCLK_CNTRL_MODE
              - MSP430_State() with parameter 'stop' set to FALSE
              - MSP430_Breakpoint()
              - MSP430_EEM_Open()
              - MSP430_EEM_Read_Register()
              - MSP430_EEM_Read_Register_Test()
              - MSP430_EEM_Write_Register()
              - MSP430_EEM_Close()
 
 \par         Project:
              MSP430 Enhanced Emulation Module (EEM) API
 
 \par         Developed using:
              MS Visual C++ 2003/2010
 
 \par         Supported API calls:
              - MSP430_EEM_Init()
              - MSP430_EEM_SetBreakpoint()
              - MSP430_EEM_GetBreakpoint()
              - MSP430_EEM_SetCombineBreakpoint()
              - MSP430_EEM_GetCombineBreakpoint()
              - MSP430_EEM_SetTrace()
              - MSP430_EEM_GetTrace()
              - MSP430_EEM_ReadTraceBuffer()
              - MSP430_EEM_RefreshTraceBuffer()
              - MSP430_EEM_SetVariableWatch()
              - MSP430_EEM_SetVariable()
              - MSP430_EEM_GetVariableWatch()
              - MSP430_EEM_SetClockControl()
              - MSP430_EEM_GetClockControl()
              - MSP430_EEM_SetSequencer()
              - MSP430_EEM_GetSequencer()
              - MSP430_EEM_ReadSequencerState()
*/


#ifndef MSP430_EEM_H
#define MSP430_EEM_H


#include <MSP430_Debug.h>

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef MSP430_EEM_TYPES
#define MSP430_EEM_TYPES

/**
\brief	The definition of MAXHANDLE is twice of the number of Memory-Bus and
		Register-Write triggers. More handles are impossible.
*/
#define MAXHANDLE 20

/**
\brief	The definition of MAXTRIGGER is the number of Memory-Bus triggers.
*/
#define MAXTRIGGER 8

/**
\brief	The definition of N_TRACE_POS is the number of positions in the trace buffer
*/
#define N_TRACE_POS	8

/**
\brief	The definition of MAX_SEQ_TRIGGER is the number of available triggers used by the sequencer
*/
#define MAX_SEQ_TRIGGER	4

/**
\brief	The definition of MAX_SEQ_STATE is the number of available states of the sequencer
*/
#define MAX_SEQ_STATE 4

/**
 \brief     Event message identification structure:
            This structure contains the message identifications for the
            different event messages sent by the DLL. Events are sent by the
            DLL to inform the caller of a change of state (e.g. breakpoint hit)
            or to provide data to the caller
*/
typedef struct MESSAGE_ID {
   /// Message identification for "Single step complete" event
   ULONG uiMsgIdSingleStep;
   /// Message identification for "Breakpoint hit" event
   ULONG uiMsgIdBreakpoint;
   /// Message identification for "Storage on trace buffer" event
   ULONG uiMsgIdStorage;
   /// Message identification for "Change in new state of the sequencer" event
   ULONG uiMsgIdState;
   /// Message identification for "Warning" event
   ULONG uiMsgIdWarning;
   /// Message identification for "Device CPU stopped" event
   ULONG uiMsgIdCPUStopped;
}
/**
 \brief     The type MessageID_t enables the caller of the function
            MSP430_EEM_Init(LONG lhWnd, MessageID_t* pMsgIdBuffer) to set
            dedicated message IDs for the different events. Using Windows
            user events normaly start at WM_USER (please refer to MSDN help)
*/
MessageID_t;


/// WarCode_t contains the warning codes that are sent as an event
typedef enum WarningCodes {
   /// Combination removed
   WAR_CLR_COMBINE = 0,
   /// Breakpoint removed from combination
   WAR_CLR_BP_COMBINE = 1,
   /// Properties of combination changed
   WAR_MOD_COMBINE = 2,
   /// Reset device
   WAR_RESET = 3,
   /// Trace trigger action is disabled and stored
   WAR_DIS_TR_TRIGGER = 4,
   /// Stored trace trigger action is enabled
   WAR_EN_TR_TRIGGER = 5,
   /// Polling thread is active - function call not allowed at the moment
   WAR_EEM_THREAD_ACTIVE = 6,
   /// forbidden old API call
   WAR_EEM_CONFLICT = 7
} WarCode_t;


/// BpMode_t gives the supported modes for a breakpoint. Used inside the BREAKPOINT structure
typedef enum BpMode {
   /// Clear breakpoint
   BP_CLEAR = 0,
   /// Set code breakpoint
   BP_CODE = 1,
   /// Set range breakpoint
   BP_RANGE = 2,
   /// Set complex breakpoint
   BP_COMPLEX = 3
} BpMode_t;


/// BpType_t gives the supported types for a breakpoint. Used inside the BREAKPOINT structure
typedef enum BpType {
   /// Set MAB breakpoint
   BP_MAB = 0,
   /// Set MDB breakpoint
   BP_MDB = 1,
   /// Set register breakpoint
   BP_REGISTER = 2
} BpType_t;

/**
 \brief     BpAccess_t gives the supported access modes for a breakpoint.
            Used inside the BREAKPOINT structure.
 
            The following table shows the relation of the access mode to the
            signals Fetch, R/W and DMA.

 \code 
                                                                                    Fetch | R/W | DMA
               BP_FETCH:                Instuction fetch                              1   | (R) | (0)
               BP_FETCH_HOLD:           Instruction fetch hold                        1   | (R) | (0)
               BP_NO_FETCH:             No instruction fetch                          0   |  X  |  X
                                                                                      1   |  X  |  1
               BP_DONT_CARE:            Don't care                                    X   |  X  |  X
               BP_NO_FETCH_READ:        No intruction fetch & read                    0   |  R  |  X
                                                                                      1   |  R  |  1
               BP_NO_FETCH_WRITE:       No instruction fetch & write                  X   |  W  |  X
               BP_READ:                 Read                                          X   |  R  |  X
               BP_WRITE:                Write                                         X   |  W  |  X
               BP_NO_FETCH_NO_DMA:      No intruction fetch & no DMA access           0   |  X  |  0
               BP_DMA:                  DMA access (read or write)                    X   |  X  |  1
               BP_NO_DMA:               No DMA access                                 X   |  X  |  0
               BP_WRITE_NO_DMA:         Write & no DMA access                         X   |  W  |  0
               BP_NO_FETCH_READ_NO_DMA: No instruction fetch & read & no DMA access   0   |  R  |  0
               BP_READ_NO_DMA:          Read & no DMA access                          X   |  R  |  0
               BP_READ_DMA:             Read & DMA access                             X   |  R  |  1
               BP_WRITE_DMA:            Write & DMA access                            X   |  W  |  1
 \endcode
*/
typedef enum BpAccess {
   /// Instuction fetch
   BP_FETCH = 0,
   /// Instruction fetch & hold trigger
   BP_FETCH_HOLD = 1,
   /// No instruction fetch
   BP_NO_FETCH = 2,
   /// Don't care
   BP_DONT_CARE = 3,
   /// No intruction fetch & read
   BP_NO_FETCH_READ = 4,
   /// No instruction fetch & write
   BP_NO_FETCH_WRITE = 5,
   /// Read
   BP_READ = 6,
   /// Write
   BP_WRITE = 7,
   /// No intruction fetch & no DMA access
   BP_NO_FETCH_NO_DMA = 8,
   /// DMA access (read or write)
   BP_DMA = 9,
   /// No DMA access
   BP_NO_DMA = 10,
   /// Write & no DMA access
   BP_WRITE_NO_DMA = 11,
   /// No instruction fetch & read & no DMA access
   BP_NO_FETCH_READ_NO_DMA = 12,
   /// Read & no DMA access
   BP_READ_NO_DMA = 13,
   /// Read & DMA access
   BP_READ_DMA = 14,
   /// Write & DMA access
   BP_WRITE_DMA = 15
} BpAccess_t;


/// BpAction_t gives the supported actions for a breakpoint. Used inside the BREAKPOINT structure
typedef enum BpAction {
   /// No action on trigger (necessary for sequencer mechanism)
   BP_NONE = 0,
   /// Break on trigger
   BP_BRK = 1,
   /// Trigger state storage (trace mechnism) on trigger
   BP_STO = 2,
   /// Break and trigger state storage on trigger
   BP_BRK_STO = 3,
} BpAction_t;


/**
 \brief     BpOperat_t gives the supported comparison operators for a breakpoint.
            Used inside the BREAKPOINT structure
*/
typedef enum BpOperat {
   /// Address/value equal MAB/MDB
   BP_EQUAL = 0,
   /// Address/value greater MAB/MDB
   BP_GREATER = 1,
   /// Address/value lower MAB/MDB
   BP_LOWER = 2,
   /// Address/value unequal MAB/MDB
   BP_UNEQUAL = 3,
} BpOperat_t;


/**
 \brief     BpRangeAction_t gives the supported range control for a range breakpoint.
            Used inside the BREAKPOINT structure
*/
typedef enum BpRangeAction {
   /// Inside range
   BP_INSIDE = 0,
   /// Outside range
   BP_OUTSIDE = 1,
} BpRangeAction_t;


/**
 \brief     BpCondition_t gives the exist condition for a complex breakpoint.
            Used inside the BREAKPOINT structure
*/
typedef enum BpCondition {
	/// No condition available
	BP_NO_COND = 0,
	/// Condition available
	BP_COND = 1,
} BpCondition_t;

/**
 \brief  The breakpoint structure contains the settings which are required to set,
         modify or clear a breakpoint.
*/
typedef struct BREAKPOINT {
  /// Breakpoint modes
  BpMode_t          bpMode;
   /// Breakpoint address/value (ignored for clear breakpoint)
  LONG				lAddrVal;
  /// Breakpoint type (used for range and complex breakpoints)
  BpType_t          bpType;
  /// Breakpoint register (used for complex breakpoints with register-write trigger)
  LONG              lReg;
  /// Breakpoint access (used only for range and complex breakpoints)
  BpAccess_t        bpAccess;
  /// Breakpoint action (break/storage) (used for range and complex breakpoints)
  BpAction_t        bpAction;
  /// Breakpoint operator (used for complex breakpoints)
  BpOperat_t        bpOperat;
  /// Breakpoint mask (used for complex breakpoints)
  LONG				lMask;
  /// Range breakpoint end address (used for range breakpoints)
  LONG				lRangeEndAdVa;
  /// Range breakpoint action (inside/outside) (used for range breakpoints)
  BpRangeAction_t   bpRangeAction;
  /// Complex breakpoint: Condition available
  BpCondition_t     bpCondition;
  /// Complex breakpoint: MDB value (used for complex breakpoints)
  ULONG				lCondMdbVal;
  /// Complex breakpoint: Access (used for complex breakpoints)
  BpAccess_t        bpCondAccess;
  /// Complex breakpoint: Mask Value(used for complex breakpoints)
  LONG				lCondMask;
  /// Complex breakpoint: Operator (used for complex breakpoints)
  BpOperat_t        bpCondOperat;
  /// Combine breakpoint: Reference of a combination handle
  WORD			    wExtCombine;
}
/**
 \brief  The type BpParameter_t is used as a source parameter to the function
         MSP430_EEM_SetBreakpoint(WORD* pwBpHandle, BpParameter_t* pBpBuffer)
         and as a destination parameter to the function
         MSP430_EEM_GetBreakpoint(WORD wBpHandle, BpParameter_t* pBpDestBuffer)
*/
BpParameter_t;


/**
 \brief  CbControl_t gives the supported control options for a combined breakpoint.
         Used in MSP430_EEM_SetCombineBreakpoint()
*/
typedef enum CbControl {
   /// Combines two or several available breakpoints
   CB_SET = 0,
   /// Clears existing combination of two or several breakpoints
   CB_CLEAR = 1,
} CbControl_t;




/// Trace: Control
typedef enum TrControl {
   /// Enable state storage
   TR_ENABLE = 0,
   /// Disable state storage
   TR_DISABLE = 1,
   /// Reset state storage
   TR_RESET = 2,
} TrControl_t;

/// Trace: Mode
typedef enum TrMode {
   /// Trigger stops the Trace
   TR_HISTORY = 0,
   /// Trigger starts the Trace - stops if buffer is full
   TR_FUTURE = 1,
   /// Start immediately - stops if buffer is full
   TR_SHOT = 2,
   /// Collect data only at trigger event - stops if buffer is full
   TR_COLLECT = 3,
} TrMode_t;

/// Trace: Action (ignored for collect data mode)
typedef enum TrAction {
   /// Trace information only at instruction Fetch
   TR_FETCH = 0,
   /// Trace information on all MCLK clocks
   TR_ALL_CYCLE = 1,
} TrAction_t;

/**
 \brief  Trace parameter structure:
         The data structure contains the configuration settings of the EEM trace function.
*/
typedef struct TRACE_CTRL {
   /// Enable/disable/reset trace buffer (see enumerations of TR_Control)
   TrControl_t	trControl;
   /// Stores history, future, one snap shot or collect data on trigger (see enumerations of TR_Mode) (only if trControl = ST_ENABLE, else ignored)
   TrMode_t		trMode;
   /// Store on instruction fetch or on all cycles (see enumerations of TR_Action) (only trControl = ST_ENABLE and trMode != TR_COLLECT, else ignored)
   TrAction_t   trAction;
}

/**
 \brief  The type TrParameter_t is used by the functions:
         - MSP430_EEM_SetTrace(TrParameter_t* pTrBuffer) as a source buffer
         - MSP430_EEM_SetTrace(TrParameter_t* pTrDestBuffer) as a destination buffer 
         for the trace settings.
*/
TrParameter_t;

/**
 \brief  Trace buffer readout structure:
         The data structure is a copy of one position of the hardware trace buffer.
         They consist of data in a 40 bit buffer. The 40 bits are divided in 16 bit MAB,
         16 bit MDB and 8 bit control signals.
*/
typedef struct TRACE_BUFFER {
   /// Trace buffer MAB
   LONG    lTrBufMAB;
   /// Trace buffer MDB
   LONG    lTrBufMDB;
   /// Trace buffer control signals
   WORD   wTrBufCNTRL;
}

/**
 \brief  The type TraceBuffer_t is used by the function:
         - MSP430_EEM_ReadTraceBuffer(TraceBuffer_t* pTraceBuffer) as a destination buffer 
         for the read out data.
*/
TraceBuffer_t;




/// Variable watch: Enable
typedef enum VwEnable {
   /// Enable the variable watch function
   VW_ENABLE = 0,
   /// Disable the variable watch function
   VW_DISABLE = 1,
} VwEnable_t;

/// Variable watch: Control
typedef enum VwControl {
   /// Set a variable to watch
   VW_SET = 0,
   /// Clear a watched variable
   VW_CLEAR = 1,
} VwControl_t;

/// Variable watch: Data type of the variable (ignored for VW_CLEAR)
typedef enum VwDataType {
   /// Byte
   VW_8 = 0,
   /// Word
   VW_16 = 1,
   /// Long
   VW_32 = 2,
} VwDataType_t;

/**
 \brief  Variable watch parameter structure:
         The data structure contains the settings of one variable.
*/
typedef struct VARIABLE_WATCH {
   /// Set/clear variable
   VwControl_t	vwControl;
   /// Address of the watched variable (ignored for VW_CLEAR)
   ULONG     lAddr;
   /// Data type of the variable (ignored for VW_CLEAR)
   VwDataType_t	vwDataType;
}
/**
 \brief  The type VwParameter_t is used by the function:
         - MSP430_EEM_SetVariable(WORD* pVwHandle, VwParameter_t* pVwBuffer) as a source buffer
         for the variable watch settings.
*/
VwParameter_t;


/**
 \brief  Variable watch resource structure:
         The data structure contains the resources of one variable trigger.
*/
typedef struct VAR_WATCH_RESOURCES {
   /// Handle of the variable trigger
   WORD		vwHandle;
   /// Address of the watched variable
   ULONG     lAddr;
   /// Data type of the variable
   VwDataType_t	vwDataType;
}
/**
 \brief  The type VwResources_t is used as a array of 8 by the function:
         - MSP430_EEM_GetVariableWatch(BOOL* pbVwEnable, VwResouces_t* paVwDestBuffer)
         as a destination buffer for the variable watch settings.
*/
VwResources_t;


/// Clock control: Extended emulation
typedef enum CcControl {
   /// Disable
   CC_DISABLE = 0,
   /// Enable
   CC_ENABLE = 1,
} CcControl_t;

/// Clock control: Clock for selected modules switch off (logic AND operation) (only for extended clock control, else ignored)
typedef enum CcModule {
   /// All module clocks are running on emualtion halt
   CC_ALLRUN        = 0,
   /// Stop clock for Watch Dog Timer on emualtion halt
   CC_WDT           = (1 << 1),
   /// Stop clock for TimerA on emualtion halt
   CC_TIMER_A       = (1 << 2),
   /// Stop clock for TimerB on emualtion halt
   CC_TIMER_B       = (1 << 3),
   /// Stop clock for Basic Timer on emualtion halt
   CC_BASIC_TIMER   = (1 << 4),
   /// Stop clock for LCD frequency on emualtion halt
   CC_LCD_FREQ      = (1 << 5),
   /// Stop clock for 8 bit Timer/Counter on emualtion halt
   CC_TIMER_COUNTER	= (1 << 6),
   /// Stop clock for Timer Port on emualtion halt
   CC_TIMER_PORT    = (1 << 7),
   /// Stop clock for USART0 on emualtion halt
   CC_USART0        = (1 << 8),
   /// Stop clock for USART1 on emualtion halt
   CC_USART1        = (1 << 9),
   /// Stop clock for Flash Control on emualtion halt
   CC_FLASH_CNTRL   = (1 << 10),
   /// Stop clock for ADC on emualtion halt
   CC_ADC           = (1 << 11),
   /// Stop ACLK on extern pin on emualtion halt
   CC_ACLK          = (1 << 12),
   /// Stop SMCLK on extern pin on emualtion halt
   CC_SMCLK         = (1 << 13),
   /// Stop MCLK on extern pin on emualtion halt
   CC_MCLK          = (1 << 14),
} CcModule_t;


/**
\brief  Clock control: Switch general clock off (logic AND operation)
\note   This function does influence the module clock control.\n 
        If the general clock is not stop it could not be stopped at the module
*/
typedef enum CcGeneralCLK {
   /// All general clocks running on emulation halt
   CC_STP_NONE		= 0,
   /// Stop ACLK on emulation halt
   CC_STP_ACLK      = (1 << 1),
   /// Stop SMCLK on emulation halt
   CC_STP_SMCLK		= (1 << 2),
   /// Stop MCLK on emulation halt (not for extended clock control)
   CC_STP_MCLK      = (1 << 3),
   /// Stop TACLK on emulation halt (only for standard clock control)
   CC_STP_TACLK		= (1 << 5),
} CcGeneralCLK_t;

/**
\brief   Clock control parameter structure:
         The data structure contains the settings of the clock control features.
*/
typedef struct CLOCK_CONTROL {
   /// Extended emulation clock control (enable/disable clock control)
   CcControl_t	ccControl;
   /// Switch clock for modules off (1 bit per clock module, 1: stop clock module while CPU halted)
   WORD		ccModule;
   /// Switch general clock off (1 bit per clock, 1: stop clock while CPU halted)
   WORD		ccGeneralCLK;
}
/**
 \brief  The type CcParameter_t is used by the functions:
         - MSP430_EEM_SetClockControl(CcParameter_t* pCcBuffer) as a source buffer
         - MSP430_EEM_GetClockControl(CcParameter_t* pCcDestBuffer) as a destination buffer 
         for the clock control settings.
*/
CcParameter_t;




/// Sequencer: Control
typedef enum SeqControl {
   /// Disable sequencer state machine
   SEQ_DISABLE = 0,
   /// Enable sequencer state machine
   SEQ_ENABLE = 1,
} SeqControl_t;

/// Sequencer: Select next state when selected trigger occurs
typedef enum SeqState {
   /// Switch in state 0
   SEQ_STATE0 = 0,
   /// Switch in state 1
   SEQ_STATE1 = 1,
   /// Switch in state 2
   SEQ_STATE2 = 2,
   /// Switch in state 3
   SEQ_STATE3 = 3,
} SeqState_t;

/**
\brief   Sequencer parameter structure:
         The data structure contains the configuration settings of the sequencer.
		 To select no trigger provide zero as handle.
*/
typedef struct SEQUENCER {
   /// Trigger sequencer Control (enable/disable)
   SeqControl_t	seqControl;
   /// Select breakpoint as a reset trigger to set start state 0 (0 = off)
   WORD		wHandleRstTrig;
   /// Select action on entering final state
   BpAction_t	bpAction;
   //  State X:
   /// Select next state x that followed of state X
   SeqState_t   seqNextStateX[MAX_SEQ_STATE];
   /// Select breakpoint as a trigger for switching from state x into next state: x path
   WORD     wHandleStateX[MAX_SEQ_STATE];
   /// Select next state y that followed of state X
   SeqState_t   seqNextStateY[MAX_SEQ_STATE];
   /// Select breakpoint as a trigger for switching from state x into next state: y path
   WORD     wHandleStateY[MAX_SEQ_STATE];
}
/**
\brief   The type SeqParameter_t is used by the functions:
         - MSP430_EEM_SetSequencer(SeqParameter_t* pSeqBuffer) as a source buffer
         - MSP430_EEM_GetSequencer(SeqParameter_t* pSeqDestBuffer) as a destination buffer
*/
SeqParameter_t;



//==============================================================================
//==============================================================================

/**
\brief   Type definition for a callback function which must be available
         in the application which calls the MSP430.dll. The callback function
		 handles notify events which are sent from the DLL to the calling application.
		 A handle to the callback function is passed to the DLL by calling
		 MSP430_EEM_Init().
*/
typedef void (* MSP430_EVENTNOTIFY_FUNC) (UINT MsgId, UINT wParam, LONG lParam, LONG clientHandle);

//==============================================================================
#endif /* MSP430_EEM_TYPES */

/**
\fn    STATUS_T WINAPI MSP430_EEM_Init(MSP430_EVENTNOTIFY_FUNC callback, LONG clientHandle, MessageID_t* pMsgIdBuffer);

\brief   Initialisation to enable and use the functionality of the
         Enhanced Emulation Module (EEM). This function resets and
         intitializes the EEM.

\note    1. MSP430_OpenDevice() has to be called prior to this function  
\note    2. This function initializes the EEM API. By calling this function
            you are no longer allowed to call the following functions of the old EEM API:
			- MSP430_Breakpoint()
			- MSP430_EEM_Open()
			- MSP430_EEM_Read_Register()
			- MSP430_EEM_Read_Register_Test()
			- MSP430_EEM_Write_Register()\n
			The following functions can still be used with restrictions (see function documentation for details):
			- MSP430_State()
			- MSP430_Configure()\n
			All of these functions will post an according warning message.
 
\param   callback:     Pointer to the callback function of the calling application.
                       (see type definition MSP430_EVENTNOTIFY_FUNC)
\param   clientHandle: A handle to the calling application.
\param   pMsgIdBuffer: Pointer to buffer of event message identifications
                       (MessageID_t).

\code
          Sample Client Code:
          a) Client’s callback function 

             static LONG my_handle;

             void MY_MSP430_EVENTNOTIFY_FUNC(
			                 UINT MsgId,
							 UINT wParam,
							 LONG lParam,
							 LONG clientHandle)
             {
                if(clientHandle == my_handle)
                {
				   switch(MsgId)
				   {
                      // client’s handling code
				   }
                }
             }

          b) Client’s main 
          
			 int main(int argc, char** argv)
             {
                // do msp430 init

                MessageID_t MsgIds;

                pMSP430DLL->MSP430_EEM_Init(
				                  MY_MSP430_EVENTNOTIFY_FUNC,
								  my_handle,
								  &MsgIds);

                // do msp430 operations

                return 0;
             }
\endcode

\return  STATUS_OK:    EEM initialized
\return  STATUS_ERROR: EEM not initialized

\par     Error codes:
         DEVICE_UNKNOWN_ERR
\n		 NO_DEVICE_ERR
\n       EEM_INIT_ERR
\n       THREAD_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_Init(MSP430_EVENTNOTIFY_FUNC callback, LONG clientHandle, MessageID_t* pMsgIdBuffer);

/**
\fn    STATUS_T WINAPI MSP430_EEM_SetBreakpoint(WORD* pwBpHandle, BpParameter_t* pBpBuffer);

\brief   This function is used to set, modify or clear breakpoints.
         A breakpoint handle is returned via the pointer pBpHandle.
         Is a cleared or modified breakpoint combined the combinations are
		 updated and a warning message with the warning code and the
		 combination handle will be sent.

\note    MSP430_EEM_Init() have to be called prior to this function
\note	 A breakpoint with instruction fetch access set on odd address N will be set on even address N-1.

\param   pwBpHandle:   Pointer to the assigned breakpoint handle (return parameter).
                       The assigned handle is an arbitrary selected name of the breakpoint.
					   To set a combination the value pointed to (breakpoint handle) must be zero.
					   In case of clearing or modifying a breakpoint the handle of the breakpoint has
                       to be provided here. If a cleared or modified breakpoint is combined with another
					   breakpoint the combination will be updated or removed.
\param   pBpBuffer:    Pointer to breakpoint parameters (BREAKPOINT).

\return  STATUS_OK:    breakpoint configured or cleared
\return  STATUS_ERROR: breakpoint configured or cleared error

\par     Error codes:
         DEVICE_UNKNOWN_ERR
\n		 NO_DEVICE_ERR
\n       PARAMETER_ERR
\n		 BREAKPOINT_ERR
\n       CLR_SEQ_TRIGGER
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_SetBreakpoint(WORD* pwBpHandle, BpParameter_t* pBpBuffer);




/**
\fn    STATUS_T WINAPI MSP430_EEM_GetBreakpoint(WORD wBpHandle, BpParameter_t* pBpDestBuffer);

\brief   This function reads back the settings of a breakpoint.
         No change or other action on the breakpoint is performed.

\note    Observe the size of the destination buffer as a type of the structure BREAKPOINT
\note    MSP430_EEM_Init() must have been called prior to this function

\param   wBpHandle:      Handle of the breakpoint to read back
\param   pBpDestBuffer:  Pointer of the destination structure (BREAKPOINT)

\return  STATUS_OK:      breakpoint settings provided
\return  STATUS_ERROR:   breakpoint settings not provided.
  
\par     Error codes:
         PARAMETER_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_GetBreakpoint(WORD wBpHandle, BpParameter_t* pBpDestBuffer);




/**
\fn    STATUS_T WINAPI MSP430_EEM_SetCombineBreakpoint(CbControl_t CbControl, WORD wCount, WORD* pwCbHandle, WORD* pawBpHandle);

\brief   This function sets or clears combinations of breakpoints.
         The parameter wCount includes the number of breakpoints to combine or clear.
         The breakpoint handles are delivered via the pointer pwBpHandle.

\note    MSP430_EEM_Init() must have been called prior to this function

\param   CbControl:    Set/clear breakpoint combination (see enumeration CbControl)
\param   wCount:       Number of breakpoints to combine (ignored for clear combination)
\param   pwCbHandle:   Pointer to the assigned combination handle (return parameter).
                       The assigned handle is an arbitrary selected name of the combination.
					   To set a combination the value pointed to (combine handle) must be zero.
					   In case of clearing or modifying a combination the handle of the combination has
                       to be provided here.
\param   pawBpHandle:  Pointer to array that includes the breakpoint handles to set a combination
                       and in case of clearing a combination the handles of the combined breakpoints will
                       be returned here. To set a breakpoint combination the hardware resources
					   and with it the action of the first breakpoint handle in the list on pwBpHandle
					   are used. So this breakpoint can't be used as a separate and independent
					   breakpoint unlike the following breakpoints that are provided in the list on pwBpHandle.

\return  STATUS_OK:    breakpoints combined/combination cleared
\return  STATUS_ERROR: breakpoints combined/combination cleared error

\par     Error codes:
         DEVICE_UNKNOWN_ERR
\n		 NO_DEVICE_ERR
\n       PARAMETER_ERR
\n       CLR_SEQ_TRIGGER
\n       SET_SEQ_TRIGGER
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_SetCombineBreakpoint(CbControl_t CbControl, WORD wCount, WORD* pwCbHandle, WORD* pawBpHandle);




/**
\fn    STATUS_T WINAPI MSP430_EEM_GetCombineBreakpoint(WORD wCbHandle, WORD* pwCount, WORD* pawBpHandle);

\brief   This function reads back the number and list of combined breakpoints

\note    Observe the size of the destination buffers
\note    MSP430_EEM_Init() has to be called prior to this function

\param   wCbHandle:		Handle of the combined breakpoint to read back
\param   pwCount:		Pointer to the number of breakpoints that are combined
						with the breakpoint
\param   pawBpHandle:	The address of the dynamic destination buffer
						in which the handle list of combined breakpoints will be stored.

\return  STATUS_OK:     combine configuration read out
\return  STATUS_ERROR:  combine configuration read out error

\par     Error codes:
         PARAMETER_ERR
\n		 NO_DEVICE_ERR
\n		 INTERNAL_ERR

*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_GetCombineBreakpoint(WORD wCbHandle, WORD* pwCount, WORD* pawBpHandle);




/**
\fn    STATUS_T WINAPI MSP430_EEM_SetTrace(TrParameter_t* pTrBuffer);

\brief   This function configures the EEM state storage feature to trace the selected information
         into the internal Trace buffer which has a depth of 8x40 bit (F449)

\note    MSP430_EEM_Init() must have been called prior to this function

\param   pTrBuffer:   Pointer to the source buffer (Type of structure TRACE).

\return  STATUS_OK:     The state storage control register has been configured.
\return  STATUS_ERROR:  The state storage control register has not been configured.

\par     Error codes:
         DEVICE_UNKNOWN_ERR
\n		 NO_DEVICE_ERR
\n       PARAMETER_ERR
\n       RESOURCE_ERR
\n		 INTERNAL_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_SetTrace(TrParameter_t* pTrBuffer);




/**
\fn    STATUS_T WINAPI MSP430_EEM_GetTrace(TrParameter_t* pTrDestBuffer);

\brief   This function reads back the settings of a Trace Configuration Register.
         No change or other action on the trace is performed.
      
\note    Observe the size of the destination buffer as a type of the structure TRACE
\note    MSP430_EEM_Init() must have been called prior to this function

\param   pTrDestBuffer: Pointer to the destination structure (Type of structure TRACE).

\return  STATUS_OK:     The state storage settings have been returned.
\return  STATUS_ERROR:  The state storage settings have not been returned,

\par     Error codes:
         PARAMETER_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_GetTrace(TrParameter_t* pTrDestBuffer);




/**
\fn    STATUS_T WINAPI MSP430_EEM_ReadTraceBuffer(TraceBuffer_t* pTraceBuffer);

\brief   This function reads the content of the Trace Buffer of the EEM.
         No change or other action on the trace is performed.

\note    Observe the size of the destination buffer as a type of the structure TRACE_BUFFER
\note    MSP430_EEM_Init() must have been called prior to this function

\param   pTraceBuffer:  Pointer to the destination buffer (8 x 40 bit) (Type of structure TRACE_BUFFER).

\return  STATUS_OK:     The trace buffer was read out.
\return  STATUS_ERROR:  The trace buffer was not read out.

\par     Error codes:
         PARAMETER_ERR
\n		 NO_DEVICE_ERR
\n		 STATE_STOR_ERR
\n       READ_TRACE_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_ReadTraceBuffer(TraceBuffer_t* pTraceBuffer);



/**
\fn    STATUS_T WINAPI MSP430_EEM_ReadTraceData(TraceBuffer_t* pTraceBuffer, ULONG* pulCount);

\brief   This function reads the content of the Trace Buffer of the EEM.
         No change or other action on the trace is performed.

\note    MSP430_EEM_Init() must have been called prior to this function
\note	 The DLL v2 will always write 8 entries and expect pulCount to be >= 8

\param   pTraceBuffer:  Pointer to the destination buffer (array of structure TRACE_BUFFER).
\param   pulCount:		In: number of allocated elements in the passed buffer
						Out: number of actually written elements

\return  STATUS_OK:     The trace buffer was read out.
\return  STATUS_ERROR:  The trace buffer was not read out.

\par     Error codes:
         PARAMETER_ERR
\n		 NO_DEVICE_ERR
\n		 STATE_STOR_ERR
\n       READ_TRACE_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_ReadTraceData(TraceBuffer_t* pTraceBuffer, ULONG* pulCount);



/**
\fn    STATUS_T WINAPI MSP430_EEM_RefreshTraceBuffer(void);

\brief   This function refreshes the content of the Trace Buffer of the EEM.
         No change or other action on the trace is performed.

\note    MSP430_EEM_Init() must have been called prior to this function
\note    The trace function must have been enabled prior to this function

\return  STATUS_OK:     The trace buffer was refreshed.
\return  STATUS_ERROR:  The trace buffer was not refreshed.
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_RefreshTraceBuffer(void);




/**
\fn    STATUS_T WINAPI MSP430_EEM_SetVariableWatch(VwEnable_t VwEnable);

\brief   This function configures the EEM State Storage Module to work as a Real Time Monitor for
         a variable.
         
\note    MSP430_EEM_Init() must have been called prior to this function

\param   VwEnable:      Enable the variable watch function if TRUE and the
						trace function is disabled, else disable.

\return  STATUS_OK:     The variable watch function has been enabled/disabled.
\return  STATUS_ERROR:  The variable watch function has not been enabled/disabled.

\par     Error codes:
         DEVICE_UNKNOWN_ERR
\n		 NO_DEVICE_ERR
\n		 STATE_STOR_ERR
\n       PARAMETER_ERR
\n       RESOURCE_ERR
\n		 VAR_WATCH_EN_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_SetVariableWatch(VwEnable_t VwEnable);




/**
\fn    STATUS_T WINAPI MSP430_EEM_SetVariable(WORD* pVwHandle, VwParameter_t* pVwBuffer);

\brief   This function sets one variable to watch.
         
\note    MSP430_EEM_Init() must have been called prior to this function
\note    The variable watch function must be enabled.

\param   pVwHandle:     Address of the assigned variable trigger handle if the variable will be set.
						To clear the variable trigger handle must be provided.
\param   pVwBuffer:     Pointer to the source buffer (Type of structure VARIABLE_WATCH).

\return  STATUS_OK:     The variable trigger has been set/clear.
\return  STATUS_ERROR:  The variable trigger has not been set/clear.

\par     Error codes:
         PARAMETER_ERR
\n		 NO_DEVICE_ERR
\n       VAR_WATCH_EN_ERR
\n		 INTERNAL_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_SetVariable(WORD* pVwHandle, VwParameter_t* pVwBuffer);




/**
\fn    STATUS_T WINAPI MSP430_EEM_GetVariableWatch(VwEnable_t* pVwEnable, VwResources_t* paVwDestBuffer);

\brief   This function reads back the settings and the resources of the variable watch configuration.

\note    Observe the size of the destination buffer as an array of the number of MAB/MDB triggerblocks
		 from type of the structure VAR_WATCH_RESOURCES
\note    MSP430_EEM_Init() must have been called prior to this function

\param   pVwEnable:      Pointer to the destination buffer that includes the state
		                 of the variable watch function.
\param   paVwDestBuffer: Pointer to an array of the destination buffer that includes the
						 resources of the variable triggers (Type of structure VAR_WATCH_RESOURCES).

\return  STATUS_OK:      The configuration of the variable watch function has been returned.
\return  STATUS_ERROR:   The configuration of the variable watch function has not been returned.

\par     Error codes:
         PARAMETER_ERR
\n		 NO_DEVICE_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_GetVariableWatch(VwEnable_t* pVwEnable, VwResources_t* paVwDestBuffer);




/**
\fn    STATUS_T WINAPI MSP430_EEM_SetClockControl(CcParameter_t* pCcBuffer);

\brief   This function configures the Clock Control settings of the EEM at emulation stop.
         It is possible to control the general clock signals (MCLK, SMCLK, ACLK) and
         the clock signals on extern pins.
		 For devices with the extended clock control it is possible to control the clock
		 for each module.

\note    MSP430_EEM_Init() must have been called prior to this function
\note	 For devices with standard clock control the parameter pCcBuffer->ccModule must be zero
\note	 The device will be reset after applying the new settings

\param   pCcBuffer:     Pointer to the source buffer (Type of structure CLOCK_CONTROL).

\return  STATUS_OK:     The clock signals have been configured.
\return  STATUS_ERROR:  The clock signals have not been configured.

\par     Error codes:
		 DEVICE_UNKNOWN_ERR
\n		 NO_DEVICE_ERR
\n       PARAMETER_ERR
\n		 INTERNAL_ERR
\n		 RESET_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_SetClockControl(CcParameter_t* pCcBuffer);




/**
\fn    STATUS_T WINAPI MSP430_EEM_GetClockControl(CcParameter_t* pCcDestBuffer);

\brief   This function reads back the settings of the clock control.
         No change or other action on the settings is performed.

\note    Observe the size of the destination buffer as a type of the structure CLOCK_CONTROL
\note    MSP430_EEM_Init() must have been called prior to this function

\param   pCcDestBuffer: Pointer to the destination buffer (Type of structure CLOCK_CONTROL).

\return  STATUS_OK:     The clock control settings have been returned.
\return  STATUS_ERROR:  The clock control settings have not been returned.

\par     Error codes:
         PARAMETER_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_GetClockControl(CcParameter_t* pCcDestBuffer);




/**
\fn    STATUS_T WINAPI MSP430_EEM_SetSequencer(SeqParameter_t* pSeqBuffer);
                                                                
\brief   This function will be used to configure the trigger sequencer feature.
         When disabling the sequencer all further parameters will be ignored.
         For each state two of four different sequencer triggers can be set as condition
		 to switch to two selectable states. To select no trigger provide zero as handle.
		 The action trigger must be provided.

\note    MSP430_EEM_Init() must have been called prior to this function
\note    MSP430_EEM_Open() will be called if required.
\note    Use only 4 different breakpoints as trigger for switching in the next state. 
         More resources not to be available.

\param   pSeqBuffer:    Pointer to the source buffer (Type of structure SEQUENCER).

\return  STATUS_OK:     The sequencer has been configured.
\return  STATUS_ERROR:  The sequencer has not been configured.

\par     Error codes:
         DEVICE_UNKNOWN_ERR
\n		 NO_DEVICE_ERR
\n       PARAMETER_ERR
\n       RESOURCE_ERR
\n		 SEQUENCER_ERR
\n		 INTERNAL_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_SetSequencer(SeqParameter_t* pSeqBuffer);




/**
\fn    STATUS_T WINAPI MSP430_EEM_GetSequencer(SeqParameter_t* pSeqDestBuffer);

\brief   This function reads back the settings of the sequencer configuration.
         No change or other action on the sequencer is performed.

\note    Observe the size of the destination buffer as a type of the structure SEQUENCER
\note    MSP430_EEM_Init() must have been called prior to this function

\param   pSeqDestBuffer: Pointer to the destination buffer (Type of structure SEQUENCER).

\return  STATUS_OK:     The sequencer settings have been returned.
\return  STATUS_ERROR:  The sequencer settings have not been returned.

\par     Error codes:
         PARAMETER_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_GetSequencer(SeqParameter_t* pSeqDestBuffer);




/**
\fn    STATUS_T WINAPI MSP430_EEM_ReadSequencerState(SeqState_t* pSeqState);

\brief   Read out the state of the sequencer and store the result in the passed destination buffer.

\note    MSP430_EEM_Init() must have been called prior to this function
\note    MSP430_EEM_Open() will be called if required.

\param   pSeqState:     Pointer to the destination buffer (see enumeration of SeqState).

\return  STATUS_OK:     The state of the sequencer was read out.
\return  STATUS_ERROR:  The state of the sequencer was not read out.

\par     Error codes:
         PARAMETER_ERR
\n		 NO_DEVICE_ERR
\n       SEQ_ENABLE_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_ReadSequencerState(SeqState_t* pSeqState);


#if defined(__cplusplus)
}
#endif

#endif // MSP430_EEM_H
