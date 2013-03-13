/*
 * MSP430_FET.h
 *
 * API for maintaing MSP-FET430UIF (TI USB FET) hardware and firmware.
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

/**            
 \file MSP430_FET.h
 
 \brief       This file contains the DLL function headers and definitions
              to maintain the MSP-FET430UIF (TI USB FET) hardware and firmware.
 
 \par         Project:
              MSP-FET430UIF (TI USB FET) Maintenance API
 
 \par         Developed using:
              MS Visual C++ 2003/2010
 
 \par         Supported API calls:
              - MSP430_FET_SelfTest()
              - MSP430_FET_SetSignals()
              - MSP430_FET_Reset()
              - MSP430_FET_I2C()
              - MSP430_FET_EnterBootloader()
              - MSP430_FET_ExitBootloader()
              - MSP430_FET_GetFwVersion()
              - MSP430_FET_FwUpdate()
              - MSP430_FET_GetHwVersion()
*/

#ifndef MSP430_FET_H
#define MSP430_FET_H

#include "MSP430.h"

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef MSP430_FET_TYPES
#define MSP430_FET_TYPES

/**
\brief   Type definition for a callback function which could handle notify
         messages sent by the DLL to the calling application during USB FET
		 firmware update.
		 A handle to the callback function is passed to the DLL by calling
		 MSP430_FET_FwUpdate().
		 See enumeration UPDATE_STATUS_MESSAGES for details on possible
		 notify messages.
*/
typedef void (* DLL430_FET_NOTIFY_FUNC) (UINT MsgId,
                                         ULONG wParam,
                                         ULONG lParam,
                                         LONG clientHandle);

/// Status messages during USB FET firmware update
typedef enum UPDATE_STATUS_MESSAGES {
	BL_INIT = 0, /**< Initializing Update Bootloader */
	BL_ERASE_INT_VECTORS = 1, /**< Erasing mapped interrupt vectors */
	BL_ERASE_FIRMWARE = 2, /**< Erasing firmware memory section */
	BL_PROGRAM_FIRMWARE = 3, /**< Program new firmware */
	BL_DATA_BLOCK_PROGRAMMED = 4, /**< One data block of the new firmware was successfully programmed */
	BL_EXIT = 5, /**< Exit Update Bootlader and reboot firmware */
	BL_UPDATE_DONE = 6, /**< Update was successfully finished */
	BL_UPDATE_ERROR = 7, /**< An error occured during firmware update */
	BL_WAIT_FOR_TIMEOUT = 8 /**< An error occured during firmware update */
} UPDATE_STATUS_MESSAGES_t;

#endif /* MSP430_FET_TYPES */


/**
\fn   STATUS_T MSP430_FET_GetFwVersion(LONG* version);

\brief   Returns the version number of the MSP430-FET430UIF USB FET firmware.

\note    1. MSP430_Initialize() must have been called prior to calling this function.

\param   version: The version number of the MSP430-FET430UIF firmware is returned.

\return  STATUS_OK:    Firmware version was returned.
\n       STATUS_ERROR: Firmware version was NOT returned.

\par     Error codes:
         COMM_ERR
\n       INTERFACE_SUPPORT_ERR
\n		 INTERNAL_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_FET_GetFwVersion(LONG* version);

/**
\fn   STATUS_T WINAPI MSP430_FET_GetHwVersion(BYTE** version, LONG* count);

\brief   Returns the version number of the MSP430-FET430UIF USB FET hardware.

\note    1. MSP430_Initialize() must have been called prior to calling this function.

\param   version: Pointer to version number buffer of the MSP430-FET430UIF is returned.
\param   count:   Number of valid bytes in the version number buffer is returned.

\code    Desription of the Dll's internal hardware version buffer.
         The hardware revision of the MSP430-FET430UIF is retrieved internally
         by calling MSP430_Initialize(). MSP430 DLL stores the hardware revision
         number in a byte array. On calling MSP430_FET_GetHwVersion() a pointer
         to the internal buffer as well as the number of valid bytes in the buffer
         are returned.

         Current buffer implementation:
         [0] // designator letter: 'U' for USB FET, 'E' for eZ430 demo tool
         [1] // unused
         [2] // minor version number
         [3] // major version number
\endcode

\return  STATUS_OK:    Hardware version was returned.
\n       STATUS_ERROR: Hardware version was NOT returned.

\par     Error codes:
         COMM_ERR
\n		 PARAMETER_ERR
\n		 INTERNAL_ERR
\n       INTERFACE_SUPPORT_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_FET_GetHwVersion(BYTE** version, LONG* count);

/**
\fn   STATUS_T MSP430_FET_FwUpdate(CHAR* lpszFileName,
					DLL430_FET_NOTIFY_FUNC callback,
					LONG clientHandle);

\brief   Synchronizes the firmware version of the MSP-FET430UIF (TI USB FET)
         with the version of the Dll.

\note    1. MSP430_Initialize() must have been called prior to calling this function.
\note	 2. When calling MSP430_FET_FwUpdate() in response the MSP430_Initialize() 
			returning a version of -3, the file CDC.log must exist
			in the directory of the executed binary and the content must be the string "True" 
			without a newline. This file signals that a CDC driver is installed and prevents
			the update from making the MSP-FET430UIF unusable.
\note    3. The actual firmware update of the MSP-FET430UIF might require a long time
			to finish. Therefore a callback mechanism was implemented to report
            back status information to the calling application during update operation.
\note    4. Please have a look at the provided example application (UifUpdate) on how
            to apply this function!!!

\param   lpszFileName: Pointer to a file of TI TXT or INTEL HEX format.
                       It is recommended to call MSP430_FET_FwUpdate() with paramter
                       lpszFileName set to NULL. In that case an internally stored firmware
                       image will be used by MSP430.dll to perform the update. By following
                       this recommondation you make sure that the MSP-FET430UIF interface gets
                       updated with the correct firmware matching version number and features
                       of the currently used MSP430.dll Library.
\param   callback:     Pointer to a callback function of the type DLL430_FET_NOTIFY_FUNC
                       that might handle notify messages send by the DLL during firmware
                       update. This parameter can be set to NULL if one does not want to use
                       the callback mechanism.
\param   clientHandle: Handle of the calling application. Can be set to FALSE in case the
                       callback mechanism is not applied and parameter callback is set to NULL.

\return  STATUS_OK:    Firmware update was done successfully.
\n       STATUS_ERROR: Errors occured during firmware update.

\par     Error codes:
         COMM_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_FET_FwUpdate(CHAR* lpszFileName,
                                           DLL430_FET_NOTIFY_FUNC callback,
                                           LONG clientHandle);

#if defined(__cplusplus)
}
#endif

#endif // MSP430_FET_H
