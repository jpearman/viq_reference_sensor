/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     viq_device.h                                                 */
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

#include "viq_types.h"

#ifndef __VIQ_DEVICE__
#define __VIQ_DEVICE__

/*-----------------------------------------------------------------------------*/
/// @file    viq_device.h
/// @brief   Vex IQ sensor device management header
/*-----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/// The port that the I2C enable line is connected to
#define I2C_EN_PORT         1
/// The port input that the I2C enable line is connected to
#define I2C_EN_BIT          BIT5

/// Set pullup resistor on I2C_EN line
#define I2C_PORT_SETUP()    do { \
                               P1REN |=  I2C_EN_BIT; \
                               P1OUT |=  I2C_EN_BIT; \
                               } while(0)

/// Enable the interrupt for the I2C_EN bit
/// not used !
#define I2C_PORT_IRQ_SETUP() do { \
                               P1IES |=   I2C_EN_BIT; \
                               P1IFG |=  ~I2C_EN_BIT; \
                               P1IE   |=   I2C_EN_BIT; \
                               } while(0)

/// Read the I2C enable line state
#define I2C_EN               (P1IN & I2C_EN_BIT)

/*-----------------------------------------------------------------------------*/
/** @brief   Data structure holding information for a cooperative task         */
/*-----------------------------------------------------------------------------*/
/** @details
 *     Most MSP430 devices do not have enough resources to run an RTOS,
 *     however, it's often useful to partition code into a number of individual
 *     "tasks" that can handle different functions.
 *     Using cooperative tasks allows different block of code to run from the main
 *     program loop at pre-determined time interval.
 */

// Uncomment this to allow each task to be labeled for debug purposes
// We can use this if the processor has more memory available
//#define   VIQ_TASK_LABEL  1

typedef struct _ViqTask {
    uint32_t time;              ///< Absolute time that the task will next run
    uint16_t interval;          ///< Time interval for running this task in mS
    uint16_t event_mask;        ///< I2C events that will cause this task to run

    void    (* callback)(void); ///< Main task function callback

#ifdef  VIQ_TASK_LABEL
    // Save memory for release version
    uint8_t  name[4];           ///< 4 char label for this task
#endif
    } ViqTask;


/*-----------------------------------------------------------------------------*/
/** @brief   Maximum number of cooperative tasks we can handle                 */
/*-----------------------------------------------------------------------------*/
/** @details
 *      Each cooperative uses 10 bytes of memory (14 with label).  Depending on
 *      the available processor memory it is necessary to adjust this value.
 *
 */
 
#if defined (__MSP430G2553__) || defined(__DOXYGEN__)
#define  VIQ_MAX_TASKS   8
#else
// Small memory version
#define  VIQ_MAX_TASKS   4
#endif

void     viq_task_add( void callback(void), uint16_t interval, const int8_t *label, uint16_t events );
bool_t   viq_tasks_check(void);

uint32_t viq_system_tick( bool_t block );
void     viq_clock_init(void);
void     viq_port_init(void);
void     viq_startup( bool_t block );
void     viq_cleanup(void);
void     viq_soft_reset(void);

#ifdef __cplusplus
}
#endif

#endif  // __VIQ_DEVICE__
