/*
 * DatabaseImplementation.cpp
 *
 * Definition of default Eem Timer Name-Value mapping.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include "DatabaseImplementation.h"

using namespace TI::DLL430::TemplateDeviceDb;

//default initializations - when c++0x is used someday, define inline, because intention is more clear when visible in h-file at declaration
const ClockPair EemTimerImpl::Timer::Empty = {"", 0};
const ClockPair EemTimerImpl::Timer::CRC16 = {"CRC16", 0};
const ClockPair EemTimerImpl::Timer::WDT_A = {"Watchdog Timer", 1};
const ClockPair EemTimerImpl::Timer::CCS = {"CCS", 2};
const ClockPair EemTimerImpl::Timer::USB = {"USB", 3};
const ClockPair EemTimerImpl::Timer::AES = {"AES128", 4};

const ClockPair EemTimerImpl::Timer::TA = {"TimerA", 0};
const ClockPair EemTimerImpl::Timer::TB = {"TimerB", 0};

const ClockPair EemTimerImpl::Timer::TA3 = {"TimerA3", 0};
const ClockPair EemTimerImpl::Timer::TB3 = {"TimerB3", 0};

const ClockPair EemTimerImpl::Timer::BT = {"BasicTimer", 0};
const ClockPair EemTimerImpl::Timer::BT_RTC = {"BasicTimer/RTC", 0};

const ClockPair EemTimerImpl::Timer::TA2_0 = {"Timer0_A2", 5};
const ClockPair EemTimerImpl::Timer::TA2_1 = {"Timer1_A2", 6};
const ClockPair EemTimerImpl::Timer::TA2_2 = {"Timer2_A2", 7};
const ClockPair EemTimerImpl::Timer::TA3_0 = {"Timer0_A3", 8};
const ClockPair EemTimerImpl::Timer::TA3_1 = {"Timer1_A3", 9};
const ClockPair EemTimerImpl::Timer::TA3_2 = {"Timer2_A3", 10};
const ClockPair EemTimerImpl::Timer::TA3_3 = {"Timer3_A3", 11};
const ClockPair EemTimerImpl::Timer::TA5_0 = {"Timer0_A5", 12};
const ClockPair EemTimerImpl::Timer::TA5_1 = {"Timer1_A5", 13};
const ClockPair EemTimerImpl::Timer::TA5_2 = {"Timer2_A5", 14};
const ClockPair EemTimerImpl::Timer::TA7_0 = {"Timer0_A7", 15};
const ClockPair EemTimerImpl::Timer::TA7_1 = {"Timer1_A7", 16};
const ClockPair EemTimerImpl::Timer::TA7_2 = {"Timer2_A7", 17};

const ClockPair EemTimerImpl::Timer::TD3_0 = {"Timer0_D3", 18};
const ClockPair EemTimerImpl::Timer::TD3_1 = {"Timer1_D3", 19};
const ClockPair EemTimerImpl::Timer::TD3_2 = {"Timer2_D3", 20};
const ClockPair EemTimerImpl::Timer::TD3_3 = {"Timer3_D3", 21};
const ClockPair EemTimerImpl::Timer::TB3_0 = {"Timer0_B3", 22};
const ClockPair EemTimerImpl::Timer::TB3_1 = {"Timer1_B3", 23};
const ClockPair EemTimerImpl::Timer::TB3_2 = {"Timer2_B3", 24};
const ClockPair EemTimerImpl::Timer::TB5_0 = {"Timer0_B5", 25};
const ClockPair EemTimerImpl::Timer::TB5_1 = {"Timer1_B5", 26};
const ClockPair EemTimerImpl::Timer::TB5_2 = {"Timer2_B5", 27};
const ClockPair EemTimerImpl::Timer::TB7_0 = {"Timer0_B7", 28};
const ClockPair EemTimerImpl::Timer::TB7_1 = {"Timer1_B7", 29};
const ClockPair EemTimerImpl::Timer::TB7_2 = {"Timer2_B7", 30};

const ClockPair EemTimerImpl::Timer::FLASH_CTRL = {"Flash Control", 0};
const ClockPair EemTimerImpl::Timer::FLASH_CTRLER = {"Flash Controller", 0};

const ClockPair EemTimerImpl::Timer::USART0 = {"USART0", 0};
const ClockPair EemTimerImpl::Timer::USART1 = {"USART1", 0};

const ClockPair EemTimerImpl::Timer::USCI0 = {"USCI0", 31};
const ClockPair EemTimerImpl::Timer::USCI1 = {"USCI1", 32};
const ClockPair EemTimerImpl::Timer::USCI2 = {"USCI2", 33};
const ClockPair EemTimerImpl::Timer::USCI3 = {"USCI3", 34};

const ClockPair EemTimerImpl::Timer::eUSCIA0 = {"eUSCIA0", 35};
const ClockPair EemTimerImpl::Timer::eUSCIA1 = {"eUSCIA1", 36};
const ClockPair EemTimerImpl::Timer::eUSCIA2 = {"eUSCIA2", 37};
const ClockPair EemTimerImpl::Timer::eUSCIA3 = {"eUSCIA3", 38};
const ClockPair EemTimerImpl::Timer::eUSCIB0 = {"eUSCIB0", 39};

const ClockPair EemTimerImpl::Timer::TB_MCLK = {"TimerB/MCLK (Pin)", 0};
const ClockPair EemTimerImpl::Timer::TA_SMCLK = {"TimerA/SMCLK (Pin)", 0};
const ClockPair EemTimerImpl::Timer::WDT_ACLK = {"Watchdog Timer/ACLK (Pin)", 0};

const ClockPair EemTimerImpl::Timer::MCLKpin = {"MCLK (Pin)", 0};
const ClockPair EemTimerImpl::Timer::SMCLKpin = {"SMCLK (Pin)", 0};
const ClockPair EemTimerImpl::Timer::ACLKpin = {"ACLK (Pin)", 0};

const ClockPair EemTimerImpl::Timer::RTC = {"RTC", 40};
const ClockPair EemTimerImpl::Timer::BTRTC = {"BTRTC", 41};

const ClockPair EemTimerImpl::Timer::COMP_B = {"Comparator B", 42};
const ClockPair EemTimerImpl::Timer::COMP_D = {"Comparator D", 43};
const ClockPair EemTimerImpl::Timer::LCD_B = {"LCDB", 44};

const ClockPair EemTimerImpl::Timer::LCD_FREQ = {"LCD Frequency", 0};

const ClockPair EemTimerImpl::Timer::APOOL = {"APOOL", 45};

const ClockPair EemTimerImpl::Timer::RF1A = {"RF1A", 46};
const ClockPair EemTimerImpl::Timer::RF1B = {"RF1B", 47};
const ClockPair EemTimerImpl::Timer::RF2A = {"RF2A", 48};
const ClockPair EemTimerImpl::Timer::RF2B = {"RF2B", 49};

const ClockPair EemTimerImpl::Timer::DAC12_0 = {"DAC12", 50};
const ClockPair EemTimerImpl::Timer::DAC12_1 = {"DAC12", 51};
const ClockPair EemTimerImpl::Timer::SD16 = {"SD16", 0};
const ClockPair EemTimerImpl::Timer::SD16A_4 = {"SD16A", 52};
const ClockPair EemTimerImpl::Timer::SD24B = {"SD24B", 53};
const ClockPair EemTimerImpl::Timer::ADC10_A = {"ADC10", 54};
const ClockPair EemTimerImpl::Timer::ADC10_B = {"ADC10B", 55};
const ClockPair EemTimerImpl::Timer::ADC12 = {"ADC12", 0};
const ClockPair EemTimerImpl::Timer::ADC12_A = {"ADC12A", 56};
const ClockPair EemTimerImpl::Timer::ADC12_B = {"ADC12B", 56};


const ClockName EemClocksImpl::Clocks::Empty = "";
const ClockName EemClocksImpl::Clocks::TACLK = "TACLK";
const ClockName EemClocksImpl::Clocks::SMCLK = "SMCLK";
const ClockName EemClocksImpl::Clocks::ACLK = "ACLK";
