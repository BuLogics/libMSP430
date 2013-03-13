/**
* \ingroup MODULBIOS
*
* \file stream.h
*
* \brief <FILEBRIEF>
*
*/
/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
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

#ifndef _STREAM_H_
#define _STREAM_H_

#include "../include/protocol.h"
#define MESSAGE_NEW_MSG 0x0001
#define MESSAGE_LAST_MSG 0x0002
#define MESSAGE_OUT_TO_DLL (unsigned char*)0x0001
#define MESSAGE_NO_OUT (unsigned char*)0x0000

struct _StreamSafe_
{
    unsigned char rx[12];
    unsigned short *ext_data;
    unsigned short ext_size;
};
typedef struct _StreamSafe_ StreamSafe;

struct stream_funcs
{
    short (*out_init)(unsigned char, unsigned char);
    short (*flush)(void);
    short (*put_byte)(unsigned char);
    short (*put_bytes)(void*, unsigned short);
    short (*put_word)(unsigned short);
    short (*put_long)(unsigned long);
    short (*in_init)(void *, unsigned char);
    short (*internal_stream)(unsigned char *, unsigned short, unsigned char *, unsigned short, StreamSafe *);
    short (*external_stream)(StreamSafe *);
    unsigned char (*change_type)(unsigned char);
    short (*get_byte)(unsigned char *);
    short (*get_word)(unsigned short *);
    short (*get_long)(unsigned long *);
    short (*discard_bytes)(unsigned short);
    short (*memcpy)(unsigned char*, unsigned char*, unsigned short);
    short (*biosLedOn)(unsigned char);
    short (*biosLedOff)(unsigned char);
    short (*biosLedBlink)(unsigned char, unsigned short);
    short (*biosLedFlash)(unsigned char, unsigned short);
    short (*biosLedAlternate)(unsigned short);
};


#ifdef HAL_STREAM

extern struct stream_funcs * _stream_Funcs;

// add definitions for external use
// functions are located in core module 

/**
 * STREAM Function Pointer - these are the STREAM Exports
 */
#define STREAM_out_init         (*_stream_Funcs->out_init)
#define STREAM_flush            (*_stream_Funcs->flush)
#define STREAM_put_byte         (*_stream_Funcs->put_byte)
#define STREAM_put_bytes        (*_stream_Funcs->put_bytes)
#define STREAM_put_word         (*_stream_Funcs->put_word)
#define STREAM_put_long         (*_stream_Funcs->put_long)
#define STREAM_in_init          (*_stream_Funcs->in_init)
#define STREAM_internal_stream  (*_stream_Funcs->internal_stream)
#define STREAM_external_stream  (*_stream_Funcs->external_stream)
#define STREAM_out_change_type  (*_stream_Funcs->change_type)
#define STREAM_get_byte         (*_stream_Funcs->get_byte)
#define STREAM_get_word         (*_stream_Funcs->get_word)
#define STREAM_get_long         (*_stream_Funcs->get_long)
#define STREAM_discard_bytes    (*_stream_Funcs->discard_bytes)
#define STREAM_memcpy           (*_stream_Funcs->memcpy)
#define STREAM_biosLedOn        (*_stream_Funcs->biosLedOn)
#define STREAM_biosLedOff       (*_stream_Funcs->biosLedOff)
#define STREAM_biosLedBlink     (*_stream_Funcs->biosLedBlink)
#define STREAM_biosLedFlash     (*_stream_Funcs->biosLedFlash)
#define STREAM_biosLedAlternate (*_stream_Funcs->biosLedAlternate)

#else // HAL_STREAM
extern const struct stream_funcs _stream_Funcs;
/**
 * STREAM Function Pointer - these are the STREAM Exports
 */
#define STREAM_out_init        (*_stream_Funcs.out_init)
#define STREAM_flush           (*_stream_Funcs.flush)
#define STREAM_put_byte        (*_stream_Funcs.put_byte)
#define STREAM_put_bytes       (*_stream_Funcs.put_bytes)
#define STREAM_put_word        (*_stream_Funcs.put_word)
#define STREAM_put_long        (*_stream_Funcs.put_long)
#define STREAM_in_init         (*_stream_Funcs.in_init)
#define STREAM_internal_stream  (*_stream_Funcs.internal_stream)
#define STREAM_external_stream  (*_stream_Funcs.external_stream)
#define STREAM_out_change_type  (*_stream_Funcs.change_type)
#define STREAM_get_byte        (*_stream_Funcs.get_byte)
#define STREAM_get_word        (*_stream_Funcs.get_word)
#define STREAM_get_long        (*_stream_Funcs.get_long)
#define STREAM_discard_bytes   (*_stream_Funcs.discard_bytes)
#define STREAM_memcpy          (*_stream_Funcs.memcpy)
#define STREAM_biosLedOn        (*_stream_Funcs.biosLedOn)
#define STREAM_biosLedOff       (*_stream_Funcs.biosLedOff)
#define STREAM_biosLedBlink     (*_stream_Funcs.biosLedBlink)
#define STREAM_biosLedFlash     (*_stream_Funcs.biosLedFlash)
#define STREAM_biosLedAlternate (*_stream_Funcs.biosLedAlternate)

#define STREAM_CORE_ZERO_VERSION    0x00
#define STREAM_CORE_ZERO_MACRO_SIZE 0x01
#define STREAM_CORE_ZERO_MACRO_ADDR 0x02
#define STREAM_CORE_ZERO_PUC_RESET  0x03
// function numbers in "zero function(s) in HAL modul" must start at 0xFF and count downward
#endif // HAL_STREAM

#define MESSAGE_SIZE_POS              0
#define MESSAGE_CMDTYP_POS            1
#define MESSAGE_MSG_ID_POS            2
#define MESSAGE_INSIZE_POS            3
#define MESSAGE_PAYLOAD_POS           4
#define MESSAGE_EXECUTE_CALL_ADDR_POS 4
#define MESSAGE_EXECUTE_ZERO_ADDR_POS 6
#define MESSAGE_EXECUTE_PAYLOAD_POS   6
#define MESSAGE_NO_RESPONSE           0x8000
#define EXCEPTION_NOT_IMPLEMENT_ERR   0x8001    
#define EXCEPTION_MSGID_ERR           0x8002    
#define EXCEPTION_CRC_ERR             0x8003    
#define EXCEPTION_RX_TIMEOUT_ERR      0x8004    
#define EXCEPTION_TX_TIMEOUT_ERR      0x8005 
#define EXCEPTION_RX_OVERFLOW_ERR     0x8006
#define EXCEPTION_TX_NO_BUFFER        0x8007
#define EXCEPTION_COM_RESET           0x8008
#define EXCEPTION_RX_NO_BUFFER        0x8009
#define EXCEPTION_RX_TO_SMALL_BUFFER  0x800A
#define EXCEPTION_RX_LENGTH           0x800B

typedef void *(*HalMainFunc)(void *stream_adr, unsigned long); // * hal_size, void * stream_adr)
typedef short (*FuncInOut)  (unsigned short id);

#ifndef HAL_REC
#define HAL_REC
struct _HalRec_
{
    unsigned short id;
    void  *function;
};
typedef struct _HalRec_ HalRec;
#endif

#define DEVICE_FLAG_XONOFF  0x00000001
#define DEVICE_FLAG_SBW4    0x00000002
#define DEVICE_FLAG_SBW2    0x00000004
#define DEVICE_FLAG_EASY    0x00000008

struct _HAL_INFOS_
{
    HalMainFunc init;
    short sw_0;
    short sw_1;
    unsigned short hal_size;
    HalRec *hal_list_ptr;
};
typedef struct _HAL_INFOS_ *HAL_INFOS_PTR;
typedef struct _HAL_INFOS_ HAL_INFOS;


extern short _stream_flush(void); //unsigned char res_type);
extern short _stream_to_tx_buffer(void *tx_char,unsigned short size); //,unsigned char res_type);
#ifdef __ICC430__
extern short _stream_out_init(unsigned char msg_id, unsigned char res_type);
#endif

extern unsigned char rx_queu_counter_public_;

#endif

