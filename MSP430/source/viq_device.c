/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     viq_device.c                                                 */
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
/// @file    viq_device.c
/// @brief   Vex IQ sensor device management
/*-----------------------------------------------------------------------------*/

#include "viq.h"

// Only include in this file as it contains const devce specific data
#include "viq_version.h"


/*-----------------------------------------------------------------------------*/
/** @cond                                                                      */
//     Bits withing the system_flags variable
#define     SYSTICK_FLAG_TOGGLE     (1<<0)  // Increase system tick when 1
#define     SYSTICK_FLAG_UPDATED    (1<<1)  // system_tick was updated
#define     SYSTICK_FLAG_OVERFLOW   (1<<2)  // system_tick overflowed

// 6 bytes of memory needed
static volatile    uint16_t    system_flags;
static volatile    uint32_t    system_tick;

// next task slot, we assume tasks are added but never removed
static  uint8_t next_task = 0;

// static storage for the task definitions
static ViqTask ViqTasks[VIQ_MAX_TASKS];

// Duration to debounce the I2C enable line
// in approximately 2uS steps
#define I2C_ENABLE_DEBOUNCE  20

void    viq_user_port_init(void) __attribute__((weak));

/** @endcond  */
/*-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*/
/** @brief     Initialize system clocks                                        */
/*-----------------------------------------------------------------------------*/
/** @details
 *      MSP430 system clocks are initialized here
 *      Default is 16MHz DCO
 *      Watchdog is setup as an interval timer with a divisor of 8192
 *      This gives a nominal 0.5mS system tick period.  The watchdog
 *      interrupt updates a "system_tick" variable every other interrupt
 *      thereby creating a 1mS system tick counter.
 */
 
void
viq_clock_init()
{
    // WDTPW   = watchdog password
    // WDTHOLD = "hold" bit
    // Stop Watchdog timer
    WDTCTL = WDTPW + WDTHOLD;

    // Use calibration data for basic clock setup
    // Set DCO to 16MHz
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL  = CALDCO_16MHZ;

    // Tweak frequency if needed
    // My DCO ran slightly slow so the MOD bits were changed
    // This would normally then be stored back to INFO memory but
    // I left it here as an example/
    DCOCTL  = (CALDCO_16MHZ & ~(MOD4|MOD3|MOD2|MOD1|MOD0))
                            |  (MOD4|MOD3|MOD2|     MOD0);

    // Use MCLK for watchdog set as interval counter
    // I use this as a 1mS tick
    WDTCTL = WDT_MDLY_8;
    // enable watchdog interval interrupts
    IE1   |= WDTIE;
    
    // Initialize flash manager clocks
    viq_flash_init();
}

/*-----------------------------------------------------------------------------*/
/** @brief    Initialize the device IO ports here                              */
/*-----------------------------------------------------------------------------*/
/** @details
 *      This is where IO ports are initialized.  
 *      A typical sensor will use three bits on port 1 to interface with the
 *      VEX IQ brain.  Two bits are used for the I2C interface (usually P1.6
 *      and P1.7), a third is used as an enable signal (usually P1.5) during
 *      the I2C enumeration period when the sensor is being detected by the brain.
 */
 
void
viq_port_init()
{
    // Init the device to default port state

    // Port 1 Resistor enable
    P1REN = 0;
    // Port 1 outputs set to 0
    P1OUT = 0;

#ifdef  __MSP430_HAS_PORT2_R__
    // Port 2 disable
    P2REN = 0;
    P2OUT = 0;
    P2DIR = 0;
    P2SEL = 0;
#endif
#ifdef  __MSP430_HAS_PORT3_R__
    // Port 3 disable
    P3REN = 0;
    P3OUT = 0;
    P3DIR = 0;
    P3SEL = 0;
#endif

    // I2C enable line in viq_device.h
    I2C_PORT_SETUP();

    // If the user has defined a function for additional
    // port setup then call.
    if( viq_user_port_init )
        viq_user_port_init();
}

/*-----------------------------------------------------------------------------*/
/** @brief     Device startup function                                         */
/** @param[in] block If TRUE then wait for I2C enable line to fall             */
/*-----------------------------------------------------------------------------*/
/** @details
 * 
 *     Once all initialization has been finished then call this function.
 *     It blocks the sensor until the I2C enable line has been pulled low. It then
 *     enables the I2C interface (by setting the default I2C address).
 *     The one parameter allows the I2C enable line check to be skipped for debug
 *     purposes.
 */

void
viq_startup( bool_t block )
{
    uint16_t debounce;

    // Initialize the i2c bus and give it an address
    // not used by the IQ brain
    viq_i2c_init( (uint8_t *)&version_data, INITIAL_ADR );
    __enable_interrupt();

    // Wait forever until the I2C_Enable line stays low continuously for at least 40us
    // I2C_ENABLE_DEBOUNCE*2uS (on 2553 @ 16MHz, adjust accordingly for other processors).
    // set "block" as false to bypass this for debug with one sensor on the IQ brain
    if( block )
        {
        do
            {
            // Spin until I2C_EN goes low
            // Should probably use power save and an interrupt here
            // but this is "good enough".
            while(I2C_EN)
                ;                     
            
            // Loop N times unless I2C_EN goes back high
            for(debounce=0; debounce<I2C_ENABLE_DEBOUNCE && !I2C_EN; debounce++) 
                __delay_cycles(20);
            }
        while(debounce < I2C_ENABLE_DEBOUNCE);
        }

    // Change the slave address to the default with interrupts
    // disabled (why disabled?, just because we can I guess)
    __disable_interrupt();
    viq_i2c_set_slave_address( DEFAULT_ADR );
    __enable_interrupt();
}

/*-----------------------------------------------------------------------------*/
/** @brief     PORT 1 interrupt vector placeholder                             */
/*-----------------------------------------------------------------------------*/

// PORT1 Interrupt vector
#ifdef __GNUC__
__attribute__((interrupt (PORT1_VECTOR))) void port1_int(void)
#else
#pragma vector= PORT1_VECTOR
__interrupt void port1_int(void)
#endif
{
    // Not used in this code
}

/*-----------------------------------------------------------------------------*/
/** @brief     Cleanup the sensor before a soft restart                        */
/*-----------------------------------------------------------------------------*/
/** @details
 *        This function is used to disable ports or do any sensor specific
 *        cleanup before performing a soft reset.
 */
 
void
viq_cleanup()
{
#ifdef  __MSP430_HAS_TA3__
    // Clean up the timer if we were using it
    TACTL   = 0;
    TACCTL0 = 0;
    TACCTL1 = 0;
    TACCTL2 = 0;
#endif

    // Clear the IO PORTS
#ifdef  __MSP430_HAS_PORT1_R__
    P1OUT = 0;      // This is how the ports come up from POR
#endif

#ifdef  __MSP430_HAS_PORT2_R__
    P2OUT = 0;      // This is how the ports come up from POR
#endif

#ifdef  __MSP430_HAS_PORT3_R__
    P3OUT = 0;      // This is how the ports come up from POR
#endif
}

/*-----------------------------------------------------------------------------*/
/** @brief     Reset the sensor                                                */
/*-----------------------------------------------------------------------------*/
/*
 * @details
 *       Create a reset by causing a flash access violation
 *       (a PUC is generated, but not a POR)
 */

void viq_soft_reset(void)
{
    // Cleanup bnefore we reset
    viq_cleanup();

    // Create a flash key violation to generate a Device Reset (PUC)
    FCTL2 = 0xFF00;
}

/*-----------------------------------------------------------------------------*/
/** @brief     Add a cooperative task                                          */
/** @param[in] callback pointer to the function called when time expires       */
/** @param[in] interval period to wake this task in mS                         */
/** @param[in] label 4 character label for this task                           */
/** @param[in] events 16 bit event mask for this task                          */
/*-----------------------------------------------------------------------------*/
/** @details
 *     Add a cooperative task to the task list
 *     A task callback can be triggered by timer expiration or by an event flag.
 *
 *     examples.
 *
 *     Add a task that is executed every 10mS
 *     viq_task_add( myFunc, 10, "TEST", 0 );
 *
 *     Add a task that is executed every time the EVENT_0 event occurs or
 *     60 seconds has elapsed.
 *     viq_task_add( myFunc, 60000, "TEST", EVENT_0 );
 */
 
void
viq_task_add( void callback(void), uint16_t interval, const int8_t *label, uint16_t events )
{
#ifdef VIQ_TASK_LABEL
    uint16_t        i;
    const uint8_t  *p = label;
#endif

    ViqTask     *TheTask;

    if(next_task >= VIQ_MAX_TASKS)
        return;

    TheTask = &ViqTasks[next_task++];

    TheTask->time       = viq_system_tick(0) + interval;
    TheTask->interval   = interval;
    TheTask->callback   = callback;
    TheTask->event_mask = events;

    // Labels use memory, not needed for release code or small
    // memory devices
#ifdef VIQ_TASK_LABEL
    // clear name buffer
    for(i=0;i<sizeof(TheTask->name);i++)
        TheTask->name[i] = 0;
    // copy label to name - avoid clib calls - small label in this version
    for(i=0;i<sizeof(TheTask->name)-1;i++)
        TheTask->name[i] = *p++;
#endif
}

/*-----------------------------------------------------------------------------*/
/** @brief     check to see if a task function needs to be called              */
/** @returns   true if main program should reinitialize                        */
/*-----------------------------------------------------------------------------*/
/** @details
 *     Call this function often from the main program loop to allow tasks to be
 *     executed.  Tasks run in a cooperative mode, ie. if a task blocks then
 *     other tasks will not be called.
 */
 
bool_t
viq_tasks_check()
{
    int16_t      i;
    ViqTask     *TheTask;
    uint16_t    events = viq_i2c_get_events_and_clear();

    // Run through the list of tasks
    for(i=0;i<next_task;i++)
        {
        TheTask = &ViqTasks[i];
        // See if task time has expired
        if(viq_system_tick( TRUE ) >= TheTask->time )
            {
            TheTask->time = TheTask->time + TheTask->interval;
            TheTask->callback();
            }
        else
        // See if an event can trigger the callback
        if( TheTask->event_mask != 0 )
            {
            if( events & TheTask->event_mask )
                TheTask->callback();
            }
        }

    // Finally check for init event
    if( events & EVENT_INIT )
        return(TRUE);

    return(FALSE);
}

/*-----------------------------------------------------------------------------*/
/** @brief     Get the value of the system_tick variable                       */
/** @param[in] block if true, then wait until the system_tick variable changes */
/** @returns   value of system_tick                                            */
/*-----------------------------------------------------------------------------*/
/** @details
 *  
 *   The variable, system_tick, is updated every 1mS by the watchdog timer in
 *   interval mode.  System tick is a 32 bit value, this function disables
 *   the watchdog interrupt while copying and returning that value.  The block
 *   parameter can be used to put the system into low power mode 0 until
 *   the system_tick variable has been updated.
 */

uint32_t
viq_system_tick( bool_t block )
{
    register uint32_t ret;

    // Should we block until change in tick count ?
    if( block )
        {
        while( !(system_flags & SYSTICK_FLAG_UPDATED) ) {
             // enter low power mode
            LPM0;
            }
        }

    // disable watchdog interval interrupts
    IE1   &= WDTIE;

    // Clear update flag
    if( system_flags & SYSTICK_FLAG_UPDATED )
        system_flags &= ~SYSTICK_FLAG_UPDATED;

    // check for overflow
    if( system_flags & SYSTICK_FLAG_OVERFLOW )
        {
        // reset overflow flag
        system_flags &= ~SYSTICK_FLAG_OVERFLOW;
        // No action here yet
        }

    // get new value of system tick as 32 bit
    ret = system_tick;

    // enable watchdog interval interrupts
    IE1   |= WDTIE;

    return( ret );
}

/*-----------------------------------------------------------------------------*/
/** @brief     System tick interrupt                                           */
/*-----------------------------------------------------------------------------*/
/** @details
 *
 *   This interrupt runs every 0.5mS.
 *   We want the system tick to increment at a 1mS interval so we use a toggle
 *   flag to increment it every other call.
 *   This function takes about 2uS to run when using a 16MHz cpu clock.
 */

#ifdef __GNUC__
__attribute__((interrupt (WDT_VECTOR))) void watchdog_interval_int(void)
#else
#pragma vector=WDT_VECTOR       // 0xFFF4
__interrupt void watchdog_interval_int(void)
#endif
{
    LPM0_EXIT; // Clear LPM bits upon ISR Exit

    // increment every other tick
    if( system_flags & SYSTICK_FLAG_TOGGLE )
        {
        system_flags &= ~SYSTICK_FLAG_TOGGLE;
        // Increase the system tick
        // This is a 32 bit unsigned variable
        system_tick++;
        system_flags |= SYSTICK_FLAG_UPDATED;

        // Indicate overflow if counter is 0
        if( system_tick == 0 )
            system_flags |= SYSTICK_FLAG_OVERFLOW;
        }
    else
        system_flags |= SYSTICK_FLAG_TOGGLE;
        
    // WDTIFG is automatically reset
}





