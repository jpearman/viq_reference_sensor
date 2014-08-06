/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     viq_i2c_hl.h                                                 */
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
/*    Description                                                              */
/*                                                                             */
/*  Sensors communicate with the VEX IQ brain using I2C.                       */
/*  An I2C message is in the following form                                    */
/*                                                                             */
/*  +---------------+---------------+---------------+ ... +---------------+    */
/*  | Slave Address | Reg Address   | Data 0        | ... | Data n        |    */
/*  +---------------+---------------+---------------+ ... +---------------+    */
/*                                                                             */
/*  Communication uses 256 "registers", an area of memory in the sensor that   */
/*  can be read or written to.  Parts of this memory are read only, part are   */
/*  also reserved and should not be used.  Some of the registers are common to */
/*  all VEX IQ sensors and have specific functionality.                        */
/*                                                                             */
/*  Data 0 will be read/written from/to the address specified by "Reg Address" */
/*  Data 1 (if present) will be read/written from/to "Reg Address + 1", etc.   */ 
/*                                                                             */
/*                                                                             */
/*  Address 00 - 23 Sensor information registers                               */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  |    | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | A | B | C | D | E | F |     */
/*  +====+===+===+===+===+===+===+===+===+===+===+===+===+===+===+===+===+     */
/*  | 00 | V | 1 | . | 0 | 0 | . | 0 | 0 | V | E | X |   | I | Q |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | 10 | G | E | N | E | R | I | C |   |vi |pi |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | 20 |ve |md |id |st |                                                     */
/*  +----+---+---+---+---+                                                     */
/*  vi, pi are extensions to the protocol specific to user sensors             */
/*  vi = vendor id  (0x20 to 0x7E, 0x20 is VEX)                                */
/*  pi = product id (0x20 to 0x7E)                                             */
/*  ve = firmware version as single byte                                       */
/*  md = operating mode                                                        */
/*  id = sensor id (set as 0xFF for user sensor, then use vi/pi for sub types) */
/*  st = sensor status                                                         */
/*                                                                             */
/*  Address 24 ~ 43 Sensor specific data area                                  */
/*  This area is unique to each VEX IQ sensor                                  */
/*  +----+               +---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | 20 |               |   |   |   |   |   |   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | 30 |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | 40 |   |   |   |   |                                                     */
/*  +----+---+---+---+---+                                                     */
/*                                                                             */
/*                                                                             */
/*  Address 44 ~ 4F reserved for system control functions                      */
/*  Address 50 ~ 5F reserved for system control functions                      */
/*  +----+               +---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | 40 |               |   |   |   |   |   |   |   |   |   |ADR|DTA|CMD|     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | 50 |   |   |   |       |   |   |   |RST|   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*                                                                             */
/*  Address 60 ~ 9F mapped to onboard flash                                    */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | 60 |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | 70 |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | 80 |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | 90 |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*                                                                             */
/*  Unused A0 - FF                                                             */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | A0 |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | B0 |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | C0 |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | D0 |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | E0 |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*  | F0 |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |     */
/*  +----+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+     */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

#include <msp430.h>

#include "viq_types.h"

#ifndef __VIQ_I2C_HL__
#define __VIQ_I2C_HL__

/*-----------------------------------------------------------------------------*/
/// @file    viq_i2c_hl.h
/// @brief   Vex IQ sensor I2C high level interface header
/*-----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------------------------*/
/// @brief   The two IO bits for the I2C lines - no need to change really
/*-----------------------------------------------------------------------------*/
#define I2C_LINES               (BIT6 | BIT7)

/** @brief  The initial (before sensor detection by the IQ brain) I2C address  */
#define INITIAL_ADR             0x62
/** @brief  The default (after sensor detection by the IQ brain) I2C address   */
#define DEFAULT_ADR             0x60

/** @brief  Minimum I2C address that the IQ brain will assign                  */
#define SLAVE_ADDR_MIN          0x20
/** @brief  Maximum I2C address that the IQ brain will assign                  */
#define SLAVE_ADDR_MAX          0x5E

/*-----------------------------------------------------------------------------*/
/** 
 * These registers contain non-volatile information such as firmware revision
 * @name System Registers                                                      */
/*-----------------------------------------------------------------------------*/
///@{
// Do not change these values!
#define SYSTEM_REG_MIN          0x00    ///< Low extent of system register region
#define SYSTEM_REG_MAX          0x22    ///< High extent of user register region
#define SYSTEM_REG_STATUS       0x23    ///< Status register, R/W from user code

///@}

/*-----------------------------------------------------------------------------*/
/**
 * These registers are used by code to communicate sensor specific information
 * to the VEX IQ brain.
 * @name User registers                                                        */
/*-----------------------------------------------------------------------------*/
///@{
#define USER_REG_MIN            0x24    ///< Low extent of user register region
#if defined (__MSP430G2553__) || defined(__DOXYGEN__)
/// High extent of user register region
#define USER_REG_MAX            0x43    // All 32 registers
#else
#define USER_REG_MAX            0x2B    // Limit to 8 registers if small memory
#endif

/// Address of the first user register
#define USER_REG_0              USER_REG_MIN
///@}

/*-----------------------------------------------------------------------------*/
/**
 * These registers are used for sensor control functions by the VEX iq brain, 
 * this register address range is the same for all sensors and should not be
 * changed.
 * @name Control registers                                                     */
/*-----------------------------------------------------------------------------*/
///@{
// Reserved - do not use
#define RESERVED_REG_MIN        0x44    ///< Low extent of reserved region
#define RESERVED_REG_MAX        0x4C    ///< High extent of reserved region

// IQ specific control registers
#define CTRL_REG_MIN            0x4D    ///< Low extent of control register region
#define CTRL_REG_MAX            0x4F    ///< High extent of control register region

// Only three custom control r/w registers are defined at present
#define CTRL_REG_ADDR           0x4D    ///< Set the device address
#define CTRL_REG_CMD1           0x4E    ///< cmd/control data byte (not used)
#define CTRL_REG_CMD2           0x4F    ///< cmd/control command byte

/// Custom control reset register
#define CTRL_REG_RESET          0x58    // Custom control reg for reset

// commands for register cmd2
/// Reset sensor
#define CTRL_CMD2_RESET         0x03
/// Erase user flash memory
#define CTRL_CMD2_ERASE         0xC8
/// Initialise sensor without reset    
#define CTRL_CMD2_INIT          0x34    
///@}

/*-----------------------------------------------------------------------------*/
/**
 * These registers map to info flash memory
 * @name Flash registers                                                       */
/*-----------------------------------------------------------------------------*/
///@{
//
/// Register that maps to the beginning of sensor NVRAM
#define FLASH_REG_MIN           0x60
/// Register that maps to the end of sensor NVRAM
#define FLASH_REG_MAX           0x9F
///@}


/*-----------------------------------------------------------------------------*/
/** @name    flag bits to indicate i2c state                                   */
/*-----------------------------------------------------------------------------*/
///@{
//
/// Bit reflects if an I2C access is in progress
#define I2C_Busy        (1<<0)
/// Bit reflects whether the last I2C access was on the General Call address
#define I2C_GenCall     (1<<1) 
/// Bit reflects whether the last I2C access was a Query 
#define I2C_Query       (1<<2)
/// Bit reflects whether the last I2C access was a Modify
#define I2C_Modify      (1<<3)
/// Bit reflects whether the last I2C access was a Command
#define I2C_Command     (1<<4)
/// Flag to indicate to the main loop that a Modify or Command occurred, and action may be required
#define I2C_Action      (1<<5)
/// Flag to indicate that some kind of I2C interrupt occurred
#define I2C_Int         (1<<6)  
///@}


/*-----------------------------------------------------------------------------*/
/** @name    Event flags                                                       */
/*-----------------------------------------------------------------------------*/
///@{
// Defines that map the bits in event_flags to the addresses in events
// Note 15 events are user defined, the 16th event is pre-defined for all devices.
#define EVENT_0               0x0001
#define EVENT_1               0x0002
#define EVENT_2               0x0004
#define EVENT_3               0x0008
#define EVENT_4               0x0010
#define EVENT_5               0x0020
#define EVENT_6               0x0040
#define EVENT_7               0x0080
#define EVENT_8               0x0100
#define EVENT_9               0x0200
#define EVENT_A               0x0400
#define EVENT_B               0x0800
#define EVENT_C               0x1000
#define EVENT_D               0x2000
#define EVENT_E               0x4000
// This last event is defined by a write of 0x34 to command register 0x4F
#define EVENT_INIT            0x8000
///@}


/*-----------------------------------------------------------------------------*/
/** @brief   State of the I2C interface                                        */
/*-----------------------------------------------------------------------------*/
/** @details
 *     The USCI interface will always be in one of two states either
 *     waiting for a register address or sending/receiving data.  The USI
 *     interface has an addition state as it is not able to match a slave address
 *     in hardware and needs to do that in software.
 */

typedef enum {
    kI2cStateAddress        = 0,
    kI2cStateData           = 1,

    // USI interface obly
    kI2cStateSlaveAddress   = 2
} i2c_state_t;

/*-----------------------------------------------------------------------------*/
/** @brief   structure holding all i2c related data                            */
/*-----------------------------------------------------------------------------*/
typedef struct _i2cDataStruct {
    i2c_state_t state;                      ///< state of the i2c interface
    volatile    uint8_t     flags;          ///< flags, busy etc.
    volatile    uint8_t     slave_address;  ///< our current slave address
    volatile    uint8_t     reg_address;    ///< next register to be read/written

    /// storage for the r/w control registers
    volatile    uint8_t ctrl[ CTRL_REG_MAX - CTRL_REG_MIN + 1 ];

    /// storage for sensor specific registers
    volatile    uint8_t user[ USER_REG_MAX - USER_REG_MIN + 1 ];

    /// storage for the r/w status register
    volatile    uint8_t status;
    
    /// pointer to the firmware version etc. in ROM
    uint8_t    *system;
    
    /// pointer to sensor flash memory accessible from the IQ brain
    uint8_t    *flash;

    /// bitmask indicating which events were detected
    volatile    uint16_t    event_flags;

    /// storage for registers that when written create an event
    volatile    uint8_t     events[16];

    /// callback from low level to high level i2c driver
    void        (*i2c_message)( void );
} i2cDataStruct;


// Include low level header if we have that interface on this device
#ifdef  __MSP430_HAS_USCI__
#include "viq_i2c_usci.h"
#else
#ifdef  __MSP430_HAS_USI__
#include "viq_i2c_usi.h"
#endif
#endif


void     viq_i2c_init( uint8_t *device_version, uint8_t default_address );
void     viq_i2c_add_event( uint8_t reg );
int16_t  viq_i2c_check_event( uint16_t mask );
uint16_t viq_i2c_get_events(void);
uint16_t viq_i2c_get_events_and_clear(void);
void     viq_i2c_set_slave_address( uint8_t addr );
uint8_t *viq_i2c_get_data_ptr(void);
void     viq_i2c_message(void);

#ifdef __cplusplus
}
#endif

#endif  // __VIQ_I2C_HL__
