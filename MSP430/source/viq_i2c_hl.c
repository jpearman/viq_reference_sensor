/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     viq_i2c_hl.c                                                 */
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

#include <msp430.h>
#include "viq.h"
#include "viq_i2c_hl.h"
#include "viq_flash.h"

/*-----------------------------------------------------------------------------*/
/// @file    viq_i2c_hl.c
/// @brief   Vex IQ sensor I2C high level interface
/*-----------------------------------------------------------------------------*/

// Storage for all the I2C related data
static  i2cDataStruct   i2cData;

/*-----------------------------------------------------------------------------*/
/** @brief     Initialize the I2C bus to the VEX IQ brain                      */
/** @param[in] device_version Pointer to a version_t structure                 */ 
/** @param[in] default_address The initial i2c slave address to be used        */ 
/*-----------------------------------------------------------------------------*/
/** @details
 *     This function initializes the i2c communications interface to the VEX IQ
 *     brain.  The MSP430 cpu needs to have one of two serial interface
 *     blocks (USI or USCI) available, the code will choose the USCI block over
 *     the USI as it is a more capable interface.  Choice is determined by the
 *     target specified in the code composer project.
 */
 
void
viq_i2c_init( uint8_t *device_version, uint8_t default_address )
{
    uint16_t    i;

    // Init the i2c data structure
    i2cData.state         = kI2cStateAddress;
    i2cData.flags         = 0;
    i2cData.slave_address = default_address;
    i2cData.reg_address   = 0;
    i2cData.event_flags   = 0;

    i2cData.system        = device_version;
    i2cData.flash         = viq_flash_addr();

    i2cData.i2c_message   = viq_i2c_message;

    // clear control regs
    for(i=0;i<sizeof( i2cData.ctrl );i++)
        i2cData.ctrl[i] = 0;

    // clear user regs
    for(i=0;i<sizeof( i2cData.ctrl );i++)
        i2cData.user[i] = 0;

    // clear events
    for(i=0;i<sizeof( i2cData.events );i++)
        i2cData.events[i] = 0;

    // Initialize the low level driver
    // Give priority to the USCI block
    
#ifdef  __MSP430_HAS_USCI__
    viq_i2c_usci_init( &i2cData );

#else
#ifdef  __MSP430_HAS_USI__
    viq_i2c_usi_init( &i2cData );
#else
#error  "Need a serial interface module"
#endif
#endif
}

/*-----------------------------------------------------------------------------*/
/** @brief     Add an event to the event array                                 */
/** @param[in] reg The register to trigger an event on                         */
/*-----------------------------------------------------------------------------*/
/** @details
 *     An event is detection of a specific register that is written.  15 events
 *     may be defined, when an event is detected a bit is set in the event_flags
 *     variable corresponding to that event.
 */
void
viq_i2c_add_event( uint8_t reg )
{
    uint16_t        i;

    for(i=0;i<sizeof(i2cData.events)-1;i++)
        {
        // Find free event slot
        if( i2cData.events[i] == 0 )
            {
            i2cData.events[i] = reg;
            break;
            }
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief     Compare event_flags to required event mask                      */
/** @param[in] mask event mask                                                 */
/** @returns   bitwise AND of event_flags and the mask                         */
/*-----------------------------------------------------------------------------*/

int16_t
viq_i2c_check_event( uint16_t mask )
{
    // see if we wan't any of the events
    return( i2cData.event_flags & mask );
}

/*-----------------------------------------------------------------------------*/
/** @brief    Get the event mask without clearing it                           */
/** @returns  The current value of event_flags                                 */
/*-----------------------------------------------------------------------------*/

uint16_t
viq_i2c_get_events( )
{
    //get events mask
    return( i2cData.event_flags );
}

/*-----------------------------------------------------------------------------*/
/** @brief     Clear event_flags, return existing condition                    */
/** @returns   The current value of event_flags                                */
/*-----------------------------------------------------------------------------*/
/** @details
 *     Get the current value of event_flags and clear back to 0.
 *
 *     We don't want the i2c interrupt to change event_flags between reading and
 *     clearing so have to disable interrupts.
 */
uint16_t
viq_i2c_get_events_and_clear()
{
    uint16_t events;
    __disable_interrupt();
    events = i2cData.event_flags;
    i2cData.event_flags = 0;
    __enable_interrupt();

    return(events);
}

/*-----------------------------------------------------------------------------*/
/** @brief     Set the I2C slave address                                       */
/** @param[in] addr new slave address                                          */
/*-----------------------------------------------------------------------------*/

void
viq_i2c_set_slave_address( uint8_t addr )
{
#ifdef  __MSP430_HAS_USCI__
    viq_i2c_usci_setslaveaddress( addr );
#else
#ifdef  __MSP430_HAS_USI__
    // There is no hardware slave address
    i2cData.slave_address = addr;
#endif
#endif
}

/*-----------------------------------------------------------------------------*/
/** @brief     Get a pointer to the user registers                             */
/** @returns   Pointer to the user registers                                   */
/*-----------------------------------------------------------------------------*/

uint8_t *
viq_i2c_get_data_ptr()
{
    return( (uint8_t *)i2cData.user );
}

/*-----------------------------------------------------------------------------*/
/** @brief     Look for unique I2C events                                      */
/*-----------------------------------------------------------------------------*/
/** @details
 * 
 *     This function looks for unique I2C events that are common to all
 *     devices. No distinction is made between General Call accesses, vs
 *     slave address accesses. Only a non-query transaction is parsed.
 *     In some cases, the access may have been a "command", which is
 *     transmission of the register address followed by STOP.
 *  @note
 *     This function is being called from the interrupt generated by the i2c
 *     STOP bit using a callback, do not call directly.
 */

void
viq_i2c_message()
{
    uint8_t             address;
    uint16_t            bit;
    volatile register uint8_t    *p;

    // Was the last command
    if( i2cData.reg_address == CTRL_REG_CMD2 )
        {
        // Reset command
        // (0x4E) should be 0xCA according to the specification but we are ignoring
        if( i2cData.ctrl[CTRL_REG_CMD2 - CTRL_REG_MIN] == CTRL_CMD2_RESET )
            viq_soft_reset();
        else
        // Erase flash
        if( i2cData.ctrl[CTRL_REG_CMD2 - CTRL_REG_MIN] == CTRL_CMD2_ERASE )
            viq_flash_erase( i2cData.flash );
        else
        // Initialize
        if( i2cData.ctrl[CTRL_REG_CMD2 - CTRL_REG_MIN] == CTRL_CMD2_INIT )
            i2cData.event_flags |= EVENT_INIT;
        }
    else
    // Was this a command to change the slave address ?
    if( i2cData.reg_address == CTRL_REG_ADDR )
        {
        address = i2cData.ctrl[(uint16_t)(i2cData.reg_address - CTRL_REG_MIN)];
        if ((address >= SLAVE_ADDR_MIN) && (address <= SLAVE_ADDR_MAX))
            viq_i2c_set_slave_address( address );
        }
    else
    // Reset based on write access to 0x58
    if(i2cData.reg_address == CTRL_REG_RESET)
        viq_soft_reset();
    else
        {
        // Go through the event array until we find the zero
        for(p=i2cData.events,bit=1; *p != 0 && bit!=0x8000; bit<<=1)
            // look for match
            if(i2cData.reg_address == *p++)
                {
                // then set the corresponding flag bit
                i2cData.event_flags |= bit;
                break;
                }
        }
}



