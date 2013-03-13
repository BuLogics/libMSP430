/**
* \ingroup MODULBIOS
*
* \file v3_0p.c
*
* \brief process and route the different message types
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

//! \li forwarding (execute) messages to HAL
//! \li upload of HAL macros
//! \li loop management

#ifdef uController_uif
    #include "uifv1/v3_0p_hw_uif.h"
    #include "uifv1/bios.h"
#elif uController_uifplus
    #include "uifplus/v3_0p_hw_uifplus.h"
    #include "uifplus/bios.h"
#else
	#include "lpt/bios.h"
#endif
#include "v3_0p.h"
#include "stream.h"
#include <stdlib.h>


typedef short (*HalFuncInOut)(unsigned short);

//------------------------------------------------------------------------------
// Global Variables - Data Memory used by this module
//
V3opLoopArray v3op_loop_array_[V3OP_LOOP_ARRAY_COUNT] = { 
    { 0xFFFF, (unsigned char*)0xFFFF, 0x00, 0x00, 0x00, 1 }, 
    { 0xFFFF, (unsigned char*)0xFFFF, 0x00, 0x00, 0x00, 1 },
    { 0xFFFF, (unsigned char*)0xFFFF, 0x00, 0x00, 0x00, 1 },
    { 0xFFFF, (unsigned char*)0xFFFF, 0x00, 0x00, 0x00, 1 }
};

HAL_INFOS no_hal_infos_ = {0, 0, 0, 0, NULL};

HAL_INFOS_PTR hal_infos_ = &no_hal_infos_;
// ID from and pointer to all HAL functions 
HAL_REC_ARRAY hal_ptr_ = NULL;
// informations about first and last measages 
unsigned short v30p_stream_flags_ = 0;

short v3opKillLoop(unsigned char msg_id);
short v3opPauseLoop(unsigned char msg_id);
short v3opResumeLoop(unsigned char msg_id);
short v3opSetLoop(unsigned char *payload_incl_addr, unsigned char flags);
short HAL_Zero(unsigned char *data);
short v3opRx(unsigned char *str);
short v3opSendException(unsigned char msg_id, unsigned short code, unsigned short *payload);

//! \brief some (HAL)zero functions are handled in bios
//! \details \li 0 -> versions
//! \li 1 -> count of installed HAL macros
//! \li 2 -> HAl macro indexes (addresses)
//! \li 3 -> hardware reset by generate a PUC
//! \param[in]  *data pointer on receive buffer
//! \return 0  -> message not handled by this function
//! \return 1  -> ok
//! \return <0 -> error
short HAL_Zero(unsigned char *data)
{
    short ret_value = 0;
    unsigned short i;
  
    if(data[MESSAGE_EXECUTE_ZERO_ADDR_POS] == STREAM_CORE_ZERO_VERSION)  // call for SW version
    {
        if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_DATA) >= 0)
        {
            STREAM_put_word((*hal_infos_).sw_0);
            STREAM_put_word((*hal_infos_).sw_1);
            STREAM_put_word(bios_info_hw_0_);
            STREAM_put_word(bios_info_hw_1_);
            STREAM_put_bytes((unsigned char*)&core_version_,sizeof(core_version_));
            STREAM_put_bytes((unsigned char*)&core_version_,sizeof(safe_core_version_));
            STREAM_put_word(BIOS_RX_QUEUS);
            STREAM_put_word(BIOS_RX_SIZE);
            STREAM_put_long(bios_device_flags_);
            ret_value = 1;
        }
        else
        {
            ret_value = -3;
        }
    }
    else if(data[MESSAGE_EXECUTE_ZERO_ADDR_POS] == STREAM_CORE_ZERO_MACRO_SIZE)
    {
        if((*hal_infos_).hal_size)
        {
            if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_DATA) >= 0)
            {
                STREAM_put_word((*hal_infos_).hal_size);
                ret_value =  1;
            }
            else
            {
                ret_value = -4;
            }
        }
        else
        {
            ret_value = -2;
        }
    }
    else if(data[MESSAGE_EXECUTE_ZERO_ADDR_POS] == STREAM_CORE_ZERO_MACRO_ADDR)
    {
        if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_DATA) >= 0)
        {
            for(i =0;i<(*hal_infos_).hal_size;i++)
            {
                STREAM_put_word(i);
                STREAM_put_word((*hal_ptr_)[i].id);
            }
            ret_value = 1;
        }
        else
        {
            ret_value = -5;
        }
    }
    else if(data[MESSAGE_EXECUTE_ZERO_ADDR_POS] == STREAM_CORE_ZERO_PUC_RESET)
    {
        if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_ACKNOWLEDGE) >= 0)
        {
            STREAM_flush();
            ret_value = -6;
        }
        v3opHwReset();
        // there can't send a ret_value
    }
    if(ret_value == 1)
    {
        ret_value = MESSAGE_NO_RESPONSE;
        STREAM_flush();
    }
    return(ret_value);
}

short v3opPauseLoop(unsigned char msg_id)
{
    if ( msg_id != 0x40 )
    {
        int i = 0;
        for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
        {
            if(v3op_loop_array_[i].msg_id == msg_id) 
            {
                v3op_loop_array_[i].active = 0;
                break;
            }
        }
    }
    return 1;
}

short v3opResumeLoop(unsigned char msg_id)
{
    if ( msg_id != 0x40 )
    {
        int i = 0;
        for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
        {
            if(v3op_loop_array_[i].msg_id == msg_id) 
            {
                v3op_loop_array_[i].active = 1;
                break;
            }
        }
    }
    return 1;
}

//! \brief kill all loop functions, including vcc monitor
void v3opKillAllLoops(void)
{
    int i = 0;
    for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
    {
        if(v3op_loop_array_[i].indata != NULL)
        {
            free(v3op_loop_array_[i].indata);
        }
        v3op_loop_array_[i].indata = NULL;
        v3op_loop_array_[i].addr = 0xFFFF;
        v3op_loop_array_[i].active = 1;
    }
}

//! \brief kill (stop) functions in loop
//! \param[in] addr index of function to kill
//! \param[in] id with there the function was install in the loop
//! \details if addr and id are zero, all function in loop are killed
//! \return  0 -> no function found in loop to kill (only posible if addr and id 0)
//! \return >0 -> count of functions in loop who killed
//! \return <0 -> no function to kill
short v3opKillLoop(unsigned char msg_id)
{
    unsigned short i;
    short ret_value = -1;
  
    if(msg_id == 0)
    {
        ret_value = 0;
        for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
        {
            if((v3op_loop_array_[i].addr != 0xFFFF) && ((*hal_ptr_)[v3op_loop_array_[i].addr].id < 0xFF00)) 
            {
                if(v3op_loop_array_[i].indata != NULL)
                {
                    free(v3op_loop_array_[i].indata);
                }
                v3op_loop_array_[i].indata = NULL;
                v3op_loop_array_[i].addr = 0xFFFF;
                v3op_loop_array_[i].active = 1;
                
                ret_value++;
            }
        }
    }
    else
    {
        for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
        {
            if(v3op_loop_array_[i].msg_id == msg_id) break;
        }
        if(i < V3OP_LOOP_ARRAY_COUNT)   // addr found
        {
            if(v3op_loop_array_[i].indata != NULL)
            {
                free(v3op_loop_array_[i].indata);
            }
            v3op_loop_array_[i].indata = NULL;
            v3op_loop_array_[i].addr = 0xFFFF;
            v3op_loop_array_[i].active = 1;
        }
        ret_value = 1;
    }
    return(ret_value);
}


//! \brief install a HAL function in loop
//! \param[in] *addr id (addr) to receive buffer
//! \param[in] flags 0 -> function in loop until kill
//! \param[in] flags 1 -> function in loop until function returns with !=0
//! \return <0 -> function not install
//! \return  1 -> function install in loop
short v3opSetLoop(unsigned char *payload_incl_addr, unsigned char flags)
{
    unsigned short i, j;
    unsigned char payload_size;
    short ret_value = -1;
    
    // search for same running function
    for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
    {
        if((v3op_loop_array_[i].addr == *(unsigned short*)&payload_incl_addr[MESSAGE_EXECUTE_CALL_ADDR_POS]) && 
           (v3op_loop_array_[i].msg_id == payload_incl_addr[MESSAGE_MSG_ID_POS]))
        {
            ret_value = -2;
            break;
        }
    }
    // search free slot
    for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
    {
        if(v3op_loop_array_[i].addr == 0xFFFF)
        {
            break;
        }
    }
    if(i < V3OP_LOOP_ARRAY_COUNT)   // free slot found
    {
        if(payload_incl_addr[0] >= MESSAGE_EXECUTE_PAYLOAD_POS)
        {
            payload_size = payload_incl_addr[0]+1;
            v3op_loop_array_[i].indata = (unsigned char*)malloc(payload_size);
            if(v3op_loop_array_[i].indata != NULL)
            {
                for(j = 0; j < payload_size; j++)
                {
                    v3op_loop_array_[i].indata[j] = payload_incl_addr[j];
                }
                v3op_loop_array_[i].flags = flags & V3OP_LOOP_WAIT_FLAG;
                v3op_loop_array_[i].msg_id = payload_incl_addr[2] | 0x40;
                v3op_loop_array_[i].msg_type = RESPTYP_DATA;
                v3op_loop_array_[i].addr = *(unsigned short*)&payload_incl_addr[MESSAGE_EXECUTE_CALL_ADDR_POS];
                v3op_loop_array_[i].active = 1;
                ret_value = 1;
            }
            else
            {
                ret_value = -4;
            }
        }
        else
        {
            payload_size = 0;
            v3op_loop_array_[i].indata = NULL;
            v3op_loop_array_[i].flags = flags & V3OP_LOOP_WAIT_FLAG;
            v3op_loop_array_[i].msg_id = payload_incl_addr[2] | 0x40;
            v3op_loop_array_[i].msg_type = RESPTYP_DATA;
            v3op_loop_array_[i].addr = *(unsigned short*)&payload_incl_addr[MESSAGE_EXECUTE_CALL_ADDR_POS];
            v3op_loop_array_[i].active = 1;
            ret_value = 1;
        }
    }
  else
  {
    ret_value = -3;
  }
  return(ret_value);
}

#ifndef NDEBUG
#define HAL_HISTORY
#endif
#ifdef HAL_HISTORY
// for debuging
volatile  unsigned short hal_history_[100];
volatile  unsigned short hal_history_count_ = 0;
volatile  unsigned short hal_history_stop_no_ = 0;
volatile  unsigned short hal_history_last_ = 0;
#endif

//! \brief process message type
//! \param[in] *str pointer on received buffer
//! \li str[1] message type
//! \li str[2] message id
//! \li str[3] function timeout (not used)
//! \li str[4...] message type (function) depended payload
//! \return 0
short v3opRx (unsigned char *str)
{
    unsigned char tmp_char;
    unsigned short call_addr;
    short ret_value = -1;
    //short ret_value = 0x8000;
	short ret_value_tmp = 0;
    HalFuncInOut pCallAddr;

    if((bios_rx_record_.last_cmd_typ != str[MESSAGE_CMDTYP_POS]) || !(bios_rx_record_.last_msg_id & 0x80))
    {
      bios_rx_record_.last_cmd_typ = str[MESSAGE_CMDTYP_POS];
      bios_rx_record_.last_msg_call = str[MESSAGE_EXECUTE_CALL_ADDR_POS];
      v30p_stream_flags_ |= MESSAGE_NEW_MSG;
    }
    else
    {
        v30p_stream_flags_ &= ~MESSAGE_NEW_MSG;
    }
    if(!(str[MESSAGE_MSG_ID_POS] & 0x80))
    {
        v30p_stream_flags_ |= MESSAGE_LAST_MSG;
    }
    else
    {
        v30p_stream_flags_ &= ~MESSAGE_LAST_MSG;
    }
    //printf("last %i, act %i %4x\n",bios_rx_record_.last_msg_id,str[MESSAGE_MSG_ID_POS],
    //	ret_value);
    bios_rx_record_.last_msg_id = str[MESSAGE_MSG_ID_POS];
    //printf("last %i, act %i\n",bios_rx_record_.last_msg_id,str[MESSAGE_MSG_ID_POS]);

    if(!bios_wb_control_)
    {
        biosLedOn(BIOS_LED_MODE);
        bios_wb_control_ = 1;
    }
    if(str[MESSAGE_CMDTYP_POS] == CMDTYP_EXECUTE) // bypass CMDTYP_EXECUTE at switch instruction
    {
        if(v30p_stream_flags_ & MESSAGE_NEW_MSG)
        {
            call_addr = *(unsigned short*)&str[MESSAGE_EXECUTE_CALL_ADDR_POS]; // first short is on a word boundary
            STREAM_discard_bytes(2);
        }
        else
        {
            call_addr = bios_rx_record_.last_msg_call;
        }
        if(call_addr == 0)
        {
            ret_value_tmp = HAL_Zero(str);
        }
        if(!ret_value_tmp)
        {
            // adjust to payload for HAL (payload without cmd-index)
            STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
            #ifdef HAL_HISTORY
            if(hal_history_count_ >= sizeof(hal_history_)/2)
            {
                hal_history_count_=0;
            }
            hal_history_[hal_history_count_++] = call_addr;
            hal_history_last_ = call_addr;
            #endif         
            if(call_addr < (*hal_infos_).hal_size)
            {
                pCallAddr = (HalFuncInOut)(*hal_ptr_)[call_addr].function;
                if(pCallAddr != NULL)
                {
                    ret_value = pCallAddr(v30p_stream_flags_);
                    if(!ret_value)
                    {
                        STREAM_flush();
                        ret_value = MESSAGE_NO_RESPONSE;
                    }
                }
            }          
        }
        else
        {
            ret_value = ret_value_tmp;
        }
    }
    else
    {
        switch(str[MESSAGE_CMDTYP_POS])
        {
            case CMDTYP_EXECUTELOOP:
                ret_value = v3opSetLoop(str, V3OP_LOOP_WAIT_FLAG);
                break;
            case CMDTYP_EXECUTEEVER:
                ret_value = v3opSetLoop(str, 0);
                break;
            case CMDTYP_KILL:
                ret_value = v3opKillLoop(str[MESSAGE_EXECUTE_CALL_ADDR_POS+2]);
                break;
            case CMDTYP_PAUSE_LOOP:
                {
                    unsigned char msgId = str[MESSAGE_EXECUTE_CALL_ADDR_POS+2];
                    ret_value = v3opPauseLoop( msgId | 0x40 );
                }
                break;
            case CMDTYP_RESUME_LOOP:
                {
                    unsigned char msgId = str[MESSAGE_EXECUTE_CALL_ADDR_POS+2];
                    ret_value = v3opResumeLoop( msgId | 0x40 );
                }
                break;
            case CMDTYP_UPINIT:   // init update
                ret_value = v3opCoreFlashFunctionInit(str);
                break;
            case CMDTYP_UPERASE:  // erase HAL part       
                ret_value = v3opCoreFlashFunctionErase(str);          
                if(ret_value == 1) // tell Bios that Hal is dead
                {
                    biosHalInterfaceClear();
                }
                break;
            case CMDTYP_UPWRITE:  // write into HAL part
                ret_value = v3opCoreFlashFunctionWrite(str);
                break;
            case CMDTYP_UPREAD:   // read HAL part
                ret_value = v3opCoreFlashFunctionRead(str);
                break;
            case CMDTYP_UPCORE:
                // Erase Segment with Core Signature, to start install new core on restart
                v3opUpCore();
                break;
        }
    }

    if((unsigned short)ret_value != MESSAGE_NO_RESPONSE)
    {
        if(ret_value >= 0)
        {
            if(STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_ACKNOWLEDGE) >= 0);
            {
                STREAM_put_word(ret_value);
                STREAM_flush();
            }
        }
        else
        {
            STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_EXCEPTION);
            STREAM_put_word(ret_value);
            if((unsigned short)ret_value == EXCEPTION_MSGID_ERR)
            {
                tmp_char = (bios_rx_record_.last_msg_id + 1) & 0x3F;
                if(!tmp_char)
                {
                    tmp_char++;
                }
                STREAM_put_word(tmp_char);
            }
            STREAM_flush();
        }
    }
    return(0);
}

//! \brief sends exceptions with details in payload
//! \param[in] msg_id message id
//! \param[in] code if payload = NULL, 'code' is sendet in as payload
//! \param[in] *payload if !=NULL payload is sendet with the length 'code'
//! \return 0 exception sended
//! \return -1 exception not sended, no output buffer aviable
short v3opSendException(unsigned char msg_id, unsigned short code, unsigned short *payload)
{
    unsigned short i;
  
    if(STREAM_out_init(msg_id, RESPTYP_EXCEPTION) >= 0)
    {
        if(payload == NULL)
        {
            STREAM_put_word(code);
        }
        else
        {
            for(i = 0;(i < code) && (i < 126); i++)
            {
                STREAM_put_word(payload[i]);
            }
        }
        STREAM_flush();
        return(0);
    }
    return(-1);
}
