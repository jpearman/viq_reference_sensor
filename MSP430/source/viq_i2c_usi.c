/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     viq_i2c_usi.c                                                */
/*    Author:     James Pearman                                                */
/*    Created:    19 May 2014                                                  */
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
/*    (fast mode) i2c clock speeds.  There are no known issues running in      */
/*    either mode.                                                             */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

#include <msp430.h>
#include "viq.h"
#include "viq_i2c_usi.h"

/*-----------------------------------------------------------------------------*/
/// @file    viq_i2c_usi.c
/// @brief   Vex IQ sensor USI interface code
/*-----------------------------------------------------------------------------*/

#if defined(__MSP430_HAS_USI__) || defined(__DOXYGEN__)

// local pointer to the I2C data structure
static  i2cDataStruct   *i2cData = (i2cDataStruct *)0;

// Local variables used by this code
static usi_byteState_t  usi_byteState;
static usi_i2cState_t   usi_i2cState;
static uint8_t          usi_receiveByte;

#ifndef __GNUC__
#pragma FUNC_ALWAYS_INLINE(viq_i2c_usi_rx_start_bit)
#pragma FUNC_ALWAYS_INLINE(viq_i2c_usi_rx_stop_bit)
#pragma FUNC_ALWAYS_INLINE(viq_i2c_usi_rx_byte)
#pragma FUNC_ALWAYS_INLINE(viq_i2c_usi_tx_byte)
#endif

/*-----------------------------------------------------------------------------*/
/** @brief     initialize the usi i2c interface                                */
/** @param[in] data pointer to the i2c data structure                          */
/*-----------------------------------------------------------------------------*/

void
viq_i2c_usi_init( i2cDataStruct *data )
{
    if( data == NULL )
        return;

    i2cData = data;

    // Init local variables
    usi_byteState = kReceiveFirstBit;
    usi_i2cState  = kUsiReceiveMode;
    usi_receiveByte = 0;
    
    viq_i2c_usi_init_slave();
}

/*-----------------------------------------------------------------------------*/
/** @brief     initialize the usi i2c interface as a slave                     */
/*-----------------------------------------------------------------------------*/

void
viq_i2c_usi_init_slave()
{
    // Note: all PORT registers except PXOUT are reset to zero on a PUC or POR
    // Assign pins to USI primary peripheral
    P1SEL  = I2C_LINES;
    P1SEL2 = 0;

    /*
    // Port 1 Resistor enable
    P1REN = I2C_LINES;
    // Port 1 pullup enbled
    P1OUT = I2C_LINES;
    */
    
    // Enable SDA, SCL.
    // MSB first, Slave mode, output latch depends on clock
    // output disabled
    // Place in reset
    USICTL0  = USIPE7 + USIPE6 + USISWRST;

    // I2C mode with both START and COUNT interrupts enabled
    USICTL1  = USII2C + USIIE + USISTTIE;

    // Clock control
    // SCLK, divide by 1
    // Inactive state high
    // Clock is coming from the master !!
    USICKCTL = USICKPL;

    // no bits to tx or rx  
    USICNT   = 0;
    
    // Enable
    USICTL0 &= ~USISWRST;
    
    // Clear all pending interrupt flags (including the count interrupt we just created)
    USICTL1 &= ~(USISTP | USIIFG | USISTTIFG);
}


/*-----------------------------------------------------------------------------*/
/** @brief     UCI handle start bit received                                   */
/*-----------------------------------------------------------------------------*/

inline void
viq_i2c_usi_rx_start_bit()
{
    // Next byte is slave address
    i2cData->state = kI2cStateSlaveAddress;

    // Ready for first bit
    usi_byteState = kReceiveFirstBit;

    // Clear flag bits
    i2cData->flags &= ~(I2C_Query | I2C_Modify | I2C_Command);

    // Start off in receive mode
    usi_i2cState = kUsiReceiveMode;
}

/*-----------------------------------------------------------------------------*/
/** @brief     UCI handle stop bit received                                    */
/*-----------------------------------------------------------------------------*/

inline void
viq_i2c_usi_rx_stop_bit()
{
    // if it was a query or a modify, then decrement register address
    // to get back to alignment
    if(i2cData->flags & (I2C_Query + I2C_Modify))
        i2cData->reg_address--;

    // Clear Busy flag
    i2cData->flags &= ~I2C_Busy;

    // if the last access was not a query (it was a modify or a CMD)
    if(!(i2cData->flags & I2C_Query))
        i2cData->i2c_message();

    // if the last access was not a query (it was a modify or a CMD)
    if(i2cData->flags & (I2C_Modify | I2C_Command))
        // Set the flag to indicate that a register was modified.
        i2cData->flags |= I2C_Action;
}

/*-----------------------------------------------------------------------------*/
/** @brief     UCI handle byte received from master                            */
/*-----------------------------------------------------------------------------*/

inline void
viq_i2c_usi_rx_byte()
{
    // Is this a slave address byte ?
    if(i2cData->state == kI2cStateSlaveAddress)
        {
        // We received a slave address, next byte will be register address
        i2cData->state = kI2cStateAddress;

        // We are busy
        i2cData->flags |= I2C_Busy;

        // Set general call flag bit if the address is 0x00
        if (!usi_receiveByte)
            i2cData->flags |= I2C_GenCall;
        else
            i2cData->flags &= ~I2C_GenCall;
        }
    else
    // Is this the register address ?
    if(i2cData->state == kI2cStateAddress)
        {
        // This is the first byte after a start or restart
        i2cData->reg_address = usi_receiveByte;
        i2cData->state = kI2cStateData;
        }
    // Must be a register data byte
    else
        {
        // Write to control registers
        if( i2cData->reg_address >= CTRL_REG_MIN && i2cData->reg_address <= CTRL_REG_MAX )
            {
            i2cData->ctrl[(uint16_t)(i2cData->reg_address - CTRL_REG_MIN)] = usi_receiveByte;
            i2cData->flags |= I2C_Command;
            }
        else
        // Wite to user data
        if( i2cData->reg_address >= USER_REG_MIN && i2cData->reg_address <= USER_REG_MAX )
            i2cData->user[(uint16_t)(i2cData->reg_address - USER_REG_MIN)] = usi_receiveByte;
        else
        // Write to flash
        if( i2cData->reg_address >= FLASH_REG_MIN && i2cData->reg_address <= FLASH_REG_MAX )
            viq_flash_write(&i2cData->flash[i2cData->reg_address - FLASH_REG_MIN],usi_receiveByte);

        // autoincrement register address regardless
        i2cData->reg_address++;

        // Set flags to indicate this transaction was a modify (write).
        i2cData->flags |= I2C_Modify;
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief     UCI handle byte requested by master                             */
/*-----------------------------------------------------------------------------*/

inline void
viq_i2c_usi_tx_byte()
{
    // Read constant data from memory
    // Version information etc.
    if( i2cData->reg_address <= SYSTEM_REG_MAX )
        USISRL = i2cData->system[i2cData->reg_address];
    else
    // Read status byte
    if( i2cData->reg_address == SYSTEM_REG_STATUS )
        USISRL = i2cData->status;
    else
    // Read user data
    if( i2cData->reg_address >= USER_REG_MIN && i2cData->reg_address <= USER_REG_MAX )
        USISRL = i2cData->user[(uint16_t)(i2cData->reg_address - USER_REG_MIN)];
    else
    // Read one of the special control registers
    if( i2cData->reg_address >= CTRL_REG_MIN && i2cData->reg_address <= CTRL_REG_MAX )
        USISRL = i2cData->ctrl[(uint16_t)(i2cData->reg_address - CTRL_REG_MIN)];
    else
    // Read from flash mapped registers
    if( i2cData->reg_address >= FLASH_REG_MIN && i2cData->reg_address <= FLASH_REG_MAX )
        USISRL = i2cData->flash[(uint16_t)(i2cData->reg_address - FLASH_REG_MIN)];
    else
        USISRL = 0;

    // autoincrement register address regardless
    i2cData->reg_address++;

    // Set flags to indicate this transaction was a Query (read).
    i2cData->flags |= I2C_Query;
}

/*-----------------------------------------------------------------------------*/
/** @brief     UCI interrupt code                                              */
/*-----------------------------------------------------------------------------*/
/** @details
 *
 *     The USI ISR
 *     There are exactly 2 flags that can possibly cause this interrupt
 *     1) USISTTIFG  This indicates a START condition occurred.
 *     2) USIIFG     This indicates that the counter reached zero.
 *                   This happens on reads and writes.
 *     Either way, SCL (the clock line) is held low by our hardware, and needs to be released.
 */
 
#ifdef __GNUC__
__attribute__((interrupt (USI_VECTOR))) void USI_ISR(void)
#else
#pragma vector = USI_VECTOR
__interrupt void USI_ISR(void)
#endif
{
    uint8_t ctl1_status;
    uint8_t clk_timeout;

    // Get status, we are interested in two bits from this register
    // USIIFG    - bit 0, indicates shift reg counter reached 0
    // USISTTIFG - bit 1, indicates start condition
    ctl1_status = USICTL1;

    // Register the fact that some kind of interrupt occurred.
    i2cData->flags |= I2C_Int;

    // Is this a counter=0 interrupt ?
    if(ctl1_status & USIIFG)
        {
        // A rising SCL edge brought the count to zero

        // Reset timeout counter
        clk_timeout = 0;

        // Wait for SCL to go low. Also detect START and STOP bits.
        while(SCL && !(USICTL1 & (USISTP | USISTTIFG)) && --clk_timeout)
            ;

        // Check if clock is still high
        if( SCL ) {
            // Save status
            ctl1_status = USICTL1;
            // Clear status and interrupt source
            USICTL1 &= ~(USISTP | USIIFG);
            
            // Was this a stop bit ?
            if(ctl1_status & USISTP) {
                // Handle the stop bit
                // This causes the message to be handled
                viq_i2c_usi_rx_stop_bit();
            }   
            // If not a stop bit it must be a start bit or timeout
            // timeout is error, start bit will cause another interrupt

            // We are done
            return;
        }
            
        // SCL is now being held low by hardware
        // Figure out what to do next
        switch( usi_byteState )
            {
            // First bit done. This state is singled out so that the above test
            // can happen on the first bit of every byte
            case kReceiveFirstBit:
              // next state is receive 7 more bits 
              usi_byteState = kReceiveByteBits;
              
              // Clear count interrupt, release SCL
              // 7 more bits to receive
              USICNT = 7;
              break;

            // Byte has been received
            case kReceiveByteBits:
                // Next state
                usi_byteState = kReceiveLastBit;
                
                // If we are in receive mode
                if(usi_i2cState == kUsiReceiveMode)
                    {
                    // Get the received byte
                    usi_receiveByte = USISRL;

                    // Which state are we in, slave address, register address or data
                    if( i2cData->state == kI2cStateSlaveAddress )
                        {
                        // Check to see if we are being addressed
                        if( (usi_receiveByte != 0) && ((usi_receiveByte & 0xfe) != i2cData->slave_address) )
                            {
                            // Bail, not for us.
                            usi_byteState = kReceiveFirstBit;

                            // Clear all interrupts
                            USICTL1 &= ~(USISTP | USIIFG | USISTTIFG);
                            return;
                            }
                        }
                        
                    // Put out an ACK (data low)
                    USISRL = 0;
                    // Output enable
                    USICTL0 |= USIOE;
                                    
                    // Handle the received byte
                    viq_i2c_usi_rx_byte();
                    }
                else
                    // We are in transmit mode
                    {
                    // Output disable - release SDA so the master can give ACK or NACK
                    USICTL0 &= ~USIOE;
                    }
                                
                // One bit to tx or rx
                USICNT = 1;
                break;

            // Ack or Nak bit
            case kReceiveLastBit:   
                // Back to the first state
                usi_byteState = kReceiveFirstBit;

                // Check current mode
                if(usi_i2cState == kUsiReceiveMode)
                    {
                    // Was the previous byte a slave address ?
                    // Was this a request for data
                    if( i2cData->state == kI2cStateAddress && (usi_receiveByte & 1) )
                        // Switch over to transmit
                        usi_i2cState = kUsiTransmitMode;
                    else
                        // Output disable - release SDA
                        USICTL0 &= ~USIOE;
                    }

                // See if we are in transmit mode
                if(usi_i2cState == kUsiTransmitMode)
                    {
                    // Check for an ACK
                    if(!(USISRL & 1))
                        {
                        // Get the data
                        viq_i2c_usi_tx_byte();
                        
                        // Output enable
                        USICTL0 |= USIOE;
                        }
                    }
              
                // Clear count interrupt, release SCL
                // Get ready for receive
                USICNT = 1;
                break;

            default:
                // Why are we here - error
                usi_byteState = kReceiveFirstBit;
                break;
            }
        }
    else
    // Was this a start bit interrupt
    if(ctl1_status & USISTTIFG)
        {
        // Handle the start bit
        viq_i2c_usi_rx_start_bit();

        // Clear USIIFG, USISTP. Set counter to be ready to interrupt on next bit.
        USICNT = 1;                     

        // Clear USISTTIFG (START) interrupt flag, in this case, this is what releases the SCL line.
        USICTL1 &= ~USISTTIFG;
        }
}

#endif  // __MSP430_HAS_USI__

