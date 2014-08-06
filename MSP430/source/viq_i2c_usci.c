/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     viq_i2c_usci.c                                               */
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
/*                                                                             */
/*    Note: The VEX IQ brain communicates at 100KHz, the following is          */
/*          for informational purposes only.                                   */
/*                                                                             */
/*    This code has been tested at both 100KHz (standard mode) and 400KHz      */
/*    (fast mode) i2c clock speeds.  The MSP430 seems to have a minor bug when */
/*    running at 400KHz, if a START condition is detected within 10uS of a     */
/*    STOP condition, ie. two messages are sent back to back, then a PUC reset */
/*    can occur. This is not caused by un-handled interrupts or slow code.     */
/*                                                                             */
/*    Possible workarounds are as follows.                                     */
/*                                                                             */
/*    1. Disable the STOP bit interrupt, use this code in usci_init            */
/*        UCB0I2CIE &=  UCSTPIE;                                               */                        
/*        UCB0I2CIE |=  UCSTTIE;                                               */                        
/*                                                                             */
/*    2. Allow at least 20uS between i2c messages, this can typically done     */
/*       with a simple software loop (with optimization turned off)            */ 
/*                                                                             */
/*    3. Use RTOS features to send at most one message in a 1mS slot           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
               
#include <msp430.h>
#include "viq.h"
#include "viq_i2c_usci.h"

#if defined(__MSP430_HAS_USCI__) || defined(__DOXYGEN__)

/*-----------------------------------------------------------------------------*/
/// @file    viq_i2c_usci.c
/// @brief   Vex IQ sensor USCI interface code
/*-----------------------------------------------------------------------------*/

// local pointer to the I2C data structure
static  i2cDataStruct   *i2cData = (i2cDataStruct *)0;

/*-----------------------------------------------------------------------------*/
/** @brief     initialize the usci i2c interface                               */
/** @param[in] data pointer to the i2c data structure                          */
/*-----------------------------------------------------------------------------*/

void
viq_i2c_usci_init( i2cDataStruct *data )
{
    if( data == NULL )
        return;

    i2cData = data;

    viq_i2c_usci_init_slave();
}

/*-----------------------------------------------------------------------------*/
/** @brief     initialize the usci i2c interface as a slave                    */
/*-----------------------------------------------------------------------------*/

void
viq_i2c_usci_init_slave()
{
    // Note: all PORT registers except PXOUT and P2SEL (0xC0) are reset to zero on a PUC or POR
    // Touch only the bits that you need to touch
    P1SEL  =  I2C_LINES;    // Assign I2C pins to USCI_B0
    P1SEL2 =  I2C_LINES;    // Assign I2C pins to USCI_B0

    // look at 17.3.1 in slau144i.pdf for USCI initialization.
    // Enable SW reset select SMCLK
    UCB0CTL1 |= UCSSEL_2 | UCSWRST;
    // I2C Slave, synchronous mode
    UCB0CTL0  = UCMODE_3 + UCSYNC;

    // Set our slave address
    viq_i2c_usci_setslaveaddress( i2cData->slave_address );

    // Clear SW reset, resume operation
    UCB0CTL1  &= ~UCSWRST;
    // Enable STT and STP interrupt
    UCB0I2CIE |= UCSTPIE + UCSTTIE;
    // Clear any pending TX and RX interrupts
    IFG2       = ~(UCB0TXIE + UCB0RXIE);
    // Enable TX and RX interrupt
    IE2       |= UCB0TXIE + UCB0RXIE;
}

/*-----------------------------------------------------------------------------*/
/** @brief     Start and stop bit interrupt code                               */
/*-----------------------------------------------------------------------------*/
/** @details
 * 
 *  The USCI_B0 state ISR
 *  There are exactly 2 flags that can possibly cause this interrupt
 *  1) UCSTPIFG   This indicates a STOP condition - the end of a full,
 *  possible multiple-byte, memory read or write
 *  2) UCSTTIFG   This indicates a START or a RESTART condtition.
 *  Note that a register address ALWAYS follows a START or RESTART
 *
 *  The other two possible interrupt sources are not enabled:
 *   - UCNACKIFG  This is for a Not Acknowledge, and is not enabled
 *   - UCALIFG    This indicates a Loss of Arbitration, and is not enabled
 */

#ifdef __GNUC__
__attribute__((interrupt (USCIAB0RX_VECTOR))) void USCIAB0RX_ISR(void)
#else
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
#endif
{
    register uint8_t stat;

    // Get status register
    stat = UCB0STAT;

    // Register the fact that some kind of interrupt occurred.
    i2cData->flags |= I2C_Int;

    // Either are START or a STOP caused this interrupt
    // If START condition
    if(stat & UCSTTIFG)
        {
        // Next byte will be address
        i2cData->state  = kI2cStateAddress;
        // Set busy flag - we are processing message
        i2cData->flags |= I2C_Busy;

        // See if this is a General call message
        if (stat & UCGC)
            i2cData->flags |= I2C_GenCall;
        else
            i2cData->flags &= ~I2C_GenCall;

        // Clear flag bits
        i2cData->flags &= ~(I2C_Query | I2C_Modify | I2C_Command);

        // Clear interrupt START condition flag
        UCB0STAT &= ~UCSTTIFG;
        }
    else
    // If STOP condition
    if(stat & UCSTPIFG)
        {
        // if it was a query or a modify, then decrement register address
        // to get back to alignment
        if(i2cData->flags & (I2C_Query + I2C_Modify))
            i2cData->reg_address--;

        // Clear Busy flag
        i2cData->flags &= ~I2C_Busy;

        // Clear interrupt STOP condition flag
        UCB0STAT &= ~UCSTPIFG;

        // if the last access was not a query (it was a modify or a CMD)
        if(!(i2cData->flags & I2C_Query))
            i2cData->i2c_message();

        // if the last access was not a query (it was a modify or a CMD)
        if(i2cData->flags & (I2C_Modify | I2C_Command))
            // Set the flag to indicate that a register was modified.
            i2cData->flags |= I2C_Action;
        }
  }


/*-----------------------------------------------------------------------------*/
/** @brief     RX and TX interrupt code                                        */
/*-----------------------------------------------------------------------------*/
/** @details
 * 
 *  The USCI_B0 data ISR is used to move received/transmitted data.
 *  When this interrupt occurs, the response must be to either
 *  read UCB0RXBUF, or write UCB0TXBUF.
 *  Basically there are only three possible things that can happen:
 *  1) A register address is being sent from master to slave. UCB0RXBUF gets
 *     stored in i2cData->reg_address. This is what ALWAYS happens immediately after
 *     a START condition is detected.
 *  2) A byte is being sent from master to slave. UCB0RXBUF gets stored in
 *     i2cData->data[ address ]. This is what happens to any other bytes that
 *     are not immediately following a START.
 *  3) A byte is being sent from slave to master. UCB0TXBUF is written with
 *     i2cData->data[ address ]. An address byte was assumed to be sent before this
 *     happened.
 *
 *  @note
 *  This comment is from Mitch's originl code, it is in fact not quite accurate
 *  as there is no distinction made between General call and normal addressed
 *  accesses.
 *
 *  There is an exception for a General Call access. When the board is addressed
 *  with the General Call address (0x00), it does not read and write from
 *  the normally mapped register addresses. Instead one (or just a few) specific
 *  virtual register addresses are written (write-only!) for later parsing.
 *  It's probably best but not required, that the normally mapped register addresses
 *  do not overlap with the special General Call addresses.
 *  (0x4E and 0x4F are examples of special GC addresses).
 */

#ifdef __GNUC__
__attribute__((interrupt (USCIAB0TX_VECTOR))) void USCIAB0TX_ISR(void)
#else
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)    // RX/TX respond interrupt
#endif
{
    volatile int    dummy;

    // Register the fact that some kind of interrupt occurred.
    i2cData->flags |= I2C_Int;

    // Master is reading data from us
    if (UCB0CTL1 & UCTR)
        {
        // Read constant data from memory
        // Version information etc.
        if( i2cData->reg_address <= SYSTEM_REG_MAX )
            UCB0TXBUF = i2cData->system[i2cData->reg_address];
        else
        // Read status byte
        if( i2cData->reg_address == SYSTEM_REG_STATUS )
            UCB0TXBUF = i2cData->status;
        else
        // Read user data
        if( i2cData->reg_address >= USER_REG_MIN && i2cData->reg_address <= USER_REG_MAX )
            UCB0TXBUF = i2cData->user[(uint16_t)(i2cData->reg_address - USER_REG_MIN)];
        else
        // Read one of the special control registers
        if( i2cData->reg_address >= CTRL_REG_MIN && i2cData->reg_address <= CTRL_REG_MAX )
            UCB0TXBUF = i2cData->ctrl[(uint16_t)(i2cData->reg_address - CTRL_REG_MIN)];
        else
        // Read from flash mapped registers
        if( i2cData->reg_address >= FLASH_REG_MIN && i2cData->reg_address <= FLASH_REG_MAX )
            UCB0TXBUF = i2cData->flash[(uint16_t)(i2cData->reg_address - FLASH_REG_MIN)];
        else
            UCB0TXBUF = 0;

        // autoincrement register address regardless
        i2cData->reg_address++;

        // Set flags to indicate this transaction was a Query (read).
        i2cData->flags |= I2C_Query;
        }
    // Master must be sending us a byte
    else
        {
        // Is this the first byte ?
        if(i2cData->state == kI2cStateAddress)
            {
            // This is the first byte after a start or restart
            i2cData->reg_address = UCB0RXBUF;
            i2cData->state = kI2cStateData;
            }
        else
            {
            // Write to control registers
            if( i2cData->reg_address >= CTRL_REG_MIN && i2cData->reg_address <= CTRL_REG_MAX )
                {
                i2cData->ctrl[(uint16_t)(i2cData->reg_address - CTRL_REG_MIN)] = UCB0RXBUF;
                i2cData->flags |= I2C_Command;
                }
            else
            // Write to user data
            if( i2cData->reg_address >= USER_REG_MIN && i2cData->reg_address <= USER_REG_MAX )
                i2cData->user[(uint16_t)(i2cData->reg_address - USER_REG_MIN)] = UCB0RXBUF;
            else
            // Write to flash
            if( i2cData->reg_address >= FLASH_REG_MIN && i2cData->reg_address <= FLASH_REG_MAX )
                viq_flash_write(&i2cData->flash[i2cData->reg_address - FLASH_REG_MIN],UCB0RXBUF);
            else
            // Everywhere else, throw away data
                {
                dummy = UCB0RXBUF;
                dummy = dummy; // remove annoying warning
                }

            // autoincrement register address regardless
            i2cData->reg_address++;

            // Set flags to indicate this transaction was a modify (write).
            i2cData->flags |= I2C_Modify;
            }
        }
}

#endif  // __MSP430_HAS_USCI__



