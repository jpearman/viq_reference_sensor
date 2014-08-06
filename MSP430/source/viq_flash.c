/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     viq_flash.c                                                  */
/*    Author:     James Pearman                                                */
/*    Created:    16 May 2014                                                  */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00     5 August 2014 - Initial release                    */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    The author is supplying this software for use with the VEX IQ            */
/*    control system. This file can be freely distributed and teams are        */
/*    authorized to freely use this program , however, it is requested that    */
/*    improvements or additions be shared with the Vex community via the vex   */
/*    forum.  Please acknowledge the work of the authors when appropriate.     */
/*    Thanks.                                                                  */
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*      http://www.apache.org/licenses/LICENSE-2.0                             */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    or electronic mail using jbpearman_at_mac_dot_com                        */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*                                                                             */
/*    Some sections of the code were based on the work of Mitch Randall.       */
/*    ChildLikes, Inc.                                                         */
/*    mitch_at_childlikes_dot_com                                              */
/*    http://www.childlikes.com                                                */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/** @file    viq_flash.c
 *  @brief   Vex IQ sensor flash management
 * 
 * ### INFORMATION MEMORY
 * Information memory in the MSP430G2X03 is 256 bytes in size. It is arranged
 * in four segments of 64 bytes each. Each segment can be individually erased.
 * The four segments are:
 * - INFOA 0x10C0 Contains TI factory calibration data for the MSP430.
 * - INFOB 0x1080 This segment is not used.
 * - INFOC 0x1040 Mapped to registers FLASH_REG_MIN - FLASH_REG_MAX.
 * - INFOD 0x1000 Reserved - Used by VEX IQ bootloader                         */
/*-----------------------------------------------------------------------------*/

#include "viq.h"

#ifndef __GNUC__
// Create a variable to act as a pointer to the INFOC area of flash
#pragma DATA_SECTION( flash_regs, ".infoC" )
static  uint8_t flash_regs;
#endif

/*-----------------------------------------------------------------------------*/
/** @brief     Get the address of the flash memory                             */
/** @returns   The address of the flash registers                              */
/*-----------------------------------------------------------------------------*/

uint8_t *
viq_flash_addr()
{
#ifndef __GNUC__
    return( &flash_regs );
#else
    // I find no easy way to make this a variable in gcc as the default
    // linker script does not define the INFOC section
    return( (uint8_t *)0x00001040);
#endif
}

/*-----------------------------------------------------------------------------*/
/** @brief     Initialize the flash memory, mostly clocks                      */
/*-----------------------------------------------------------------------------*/

void
viq_flash_init()
{
    // Flash controller frequency select: Can clock as fast as 476 kHz
    // 421kHz = MCLK/38 = MCLK/(37+1) = MCLK/(0x25+1) for Flash Timing Generator
    FCTL2 = FWKEY + FSSEL_1 + 0x25;
}

/*-----------------------------------------------------------------------------*/
/** @brief     Erase the flash memory                                          */
/** @param[in] addr address of the beginning of the flash segment to erase     */
/*-----------------------------------------------------------------------------*/
/** @details
 *
 *     This function erases a segment of flash memory.
 *     In this case, the segment is 64 bytes starting at the physical address
 *     passed in the argument.
 *     This routine does not return until the segment has been erased.
 *     This function blocks because the processor and all interrupts are halted
 *     by the flash controller anyway.
 */

void
viq_flash_erase( uint8_t *addr )
{
    FCTL3 = FWKEY;              // Clear Lock bit
    FCTL1 = FWKEY + ERASE;      // Set Erase bit
    *addr = 0;                  // Dummy write to erase Flash segment
    FCTL1 = FWKEY;              // Clear WRT bit
    FCTL3 = FWKEY + LOCK;       // Set LOCK bit
}

/*-----------------------------------------------------------------------------*/
/** @brief     Write a byte to flash memory                                    */
/** @param[in] addr the address of flash memory to write to                    */
/** @param[in] data The data byte to write to flash memory                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *
 *     Write a byte to flash memory
 *     This routine does not return until the write completes.
 *     This function blocks because the processor and all interrupts are halted
 *     by the flash controller anyway.
 */

void
viq_flash_write( uint8_t *addr, uint8_t data )
{
    FCTL3 = FWKEY;              // Clear Lock bit
    FCTL1 = FWKEY + WRT;        // Set WRT bit for write operation
    *addr = data;
    FCTL1 = FWKEY;              // Clear WRT bit
    FCTL3 = FWKEY + LOCK;       // Set LOCK bit
}

