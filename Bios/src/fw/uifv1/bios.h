/**
* \ingroup MODULBIOS
*
* \file bios.h
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

#ifndef _V3_BIOS_H_
#define _V3_BIOS_H_

#include "hw_compiler_specific.h"

extern void biosInitSystem (void);
extern void biosInitCom (unsigned long baudrate);
extern void biosSetWdt(unsigned char time);
extern void  biosmainLoop(void);
extern void biosUsbRxClear(void);
extern void biosUsbRxError(unsigned short code);
extern void biosUsbTxError(void);
extern void biosPrepareTx(unsigned int size);
extern void biosStartTx(void);
extern short biosLedOn(unsigned char no);
extern short biosLedOff(unsigned char no);
extern short biosLedBlink(unsigned char no,unsigned short time);
extern short biosLedFlash(unsigned char no,unsigned short time);
extern short biosLedAlternate(unsigned short time);
extern short biosHalInterfaceInit(void);
extern short biosHalInterfaceClear(void);



#define BIOS_LED_MODE  0
#define BIOS_LED_POWER 1

#define BIOS_RX_SIZE 258
#define BIOS_RX_QUEUS 2
#define BIOS_TX_SIZE 258
#define BIOS_TX_QUEUS 1
#define BIOS_TX_FLAGS 1

#define BIOS_RX_RDY         0x01
#define BIOS_RX_CRC_ERROR   0x02
#define BIOS_RX_SIZE_ERROR  0x04
#define BIOS_TX_TO_SEND     0x01
#define BIOS_TX_WAIT_ON_ACK 0x02
#define BIOS_TX_NO_SEND     0x04
#define BIOS_TX_BUSY        0x08


#define BIOS_TX_TIMEOUT 50


struct _BiosRxRecord_ 
{
    unsigned char  active;
    unsigned char  last_cmd_typ;
    unsigned char  last_msg_id;
    unsigned short last_msg_call;
    unsigned char  state[BIOS_RX_QUEUS];
    unsigned char  *data[BIOS_RX_QUEUS];
    unsigned short *datas[BIOS_RX_QUEUS];
    unsigned short count[BIOS_RX_QUEUS];
    unsigned short size[BIOS_RX_QUEUS];
    unsigned short crc[2];
};
typedef struct _BiosRxRecord_ BiosRxRecord;
extern volatile BiosRxRecord bios_rx_record_;


struct _BiosTxRecord_ 
{
    unsigned char  active;
    unsigned char  cannel_to_send;
    unsigned short send_counter[BIOS_TX_QUEUS];
    unsigned short ack_timeout[BIOS_TX_QUEUS];
    unsigned char  state[BIOS_TX_QUEUS];
    unsigned short count[BIOS_TX_QUEUS];
    unsigned short datas[BIOS_TX_QUEUS][BIOS_TX_SIZE/sizeof(unsigned short)];
    unsigned char  *data[BIOS_TX_QUEUS];
    unsigned short  *ext_data;
    unsigned short ext_size;
    unsigned short ext_counter;
};
typedef struct _BiosTxRecord_ BiosTxRecord;
extern volatile BiosTxRecord bios_tx_record_;

struct _BiosGlobalTimer_
{
    unsigned short count;
    unsigned char state;
};
typedef struct _BiosGlobalTimer_ BiosGlobalTimer;

#define BIOS_TIMER_BREAK 0x01
#define BIOS_TIMER_COUNT 2
#define BIOS_TIMER_RX 0
#define BIOS_TIMER_TX 1

extern volatile BiosGlobalTimer bios_global_timer_[BIOS_TIMER_COUNT];
extern char bios_wb_control_;
extern const unsigned short core_version_;
extern __ro_placement volatile const unsigned short safe_core_version_;
extern unsigned long bios_device_flags_;
extern unsigned short bios_info_hw_0_;
extern unsigned short bios_info_hw_1_;
#endif
