/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     main.c                                                       */
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
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/** @mainpage VEX IQ sensor reference code
 *
 *
 *  Example code to demonstrate a VEX IQ user sensor.
 *
 *  This code is designed to run on the MSP430 launchpad evaluation card.  It 
 *  creates a sensor compatible with the VEX IQ brain with the following
 *  functionality.
 *
 *  PWM control of the led connected to P1.0 (IO Port 1, Pin 0)
 *  Monitoring of the switch connected to P1.3
 *  Monitoring of the internal temperature sensor (ADC10, Channel 10)
 *
 *  The code can be compiled for either of the two MSP430 devices supplied with
 *  the launchpad, the MSP430G2553 or the MSP430G2452.
 *
 *  Connection to the VEX IQ brain uses the i2c interface, this is typically
 *  connected to pins P1.6 and P1.7 on an MSP430 device.
 *
 *             MSP430G2553/G2452
 *            +-----------------+
 *         /|\|              XIN|-
 *          | |                 |
 *          --|RST          XOUT|-
 *            |                 |
 *            |P2.0         P1.0|-->LED
 *            |P2.1         P1.1|                  6P6C offset latch connector
 *            |P2.2         P1.2|                 +---------------------------+
 *            |P2.3         P1.3|<--Switch        | 1 DIO (not used)          |
 *            |P2.4         P1.4|                 | 5 7.2V (do not connect)   |
 *            |P2.5         P1.5|<----------------| 2 I2C_ENABLE              |
 *            |             P1.6|<----------------| 6 I2C_CLOCK               |
 *            |             P1.7|<--------------->| 3 I2C_DATA                |
 *            |                 |              -- | 4 GND                     |
 *            +-----------------+              |  +---------------------------+
 *                                            \|/
 *
 *
 *  The launchpad eval card does not provide pullup resistors in the two I2C
 *  lines, however, the VEX IQ brain has weak pullup resistors that allow the
 *  bus to work without them.  If an official IQ sensor is plugged into the
 *  same port triplet (IQ ports are grouped into 4 groups of three ports each) then
 *  this can also improve signal quality.
 *
*//*---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*/
/** @file    main.c
 *  @brief   Vex IQ sensor example code
 *
 *     This is example code that demonstrates how to setup a VEX IQ custom
 *     sensor and communicate with the VEX IQ brain.  This example is intended to
 *     be used with the MSP430 launchpad evaluation card, it implements three
 *     simple functions.
 *     
 *     The led connected to P1.0 can have its brightness set by sending a single
 *     byte from the IQ brain.
 *       0 = off
 *     128 = half brightness
 *     255 = fully on
 *
 *     The switch connected to P1.3 can be read by the IQ brain.
 *
 *     The MSP430 internal temperature can be read by the IQ brain.
 *
 */

#include <msp430.h> 
#include "viq.h"

// user registers specific to this sensor/demo
#define DATA_TEMP       0   ///< User register 0x24 for temperature
#define DATA_SWITCH     1   ///< User register 0x25 for switch status
#define DATA_LED_PWM    4   ///< User register 0x28 for led brightness

/// @cond

// led pwm - Default is 50%
static uint16_t led_pwm = 0x8000;

// From datasheet - there should be a better way to set this up
// but as only a demo really doesn;t matter
#define CALADC_15T85  *((uint16_t *)0x10E4)
#define CALADC_15T30  *((uint16_t *)0x10E2)

// inverse gain * 256
static int16_t  adc10_gain;

/// @endcond


/*-----------------------------------------------------------------------------*/
/** @brief     Temperature reading task                                        */
/*-----------------------------------------------------------------------------*/
/** @details
 *     Read internal ADC channel 10, this is temperature.  See the datasheet
 *     for details on calibration and conversion to deg C.  
 */
 
void
temp_task(void)
{
    register uint8_t        *data;
    register int16_t         temp;

    // Get pointer for user data area
    data = viq_i2c_get_data_ptr();

    // get temperature and convert to degC * 4
    temp = ((( ADC10MEM - CALADC_15T30 ) * adc10_gain) + (30 << 8)) >> 6;

    // Save as degC * 4 so we can fit in an 8 bit value
    // range is 0 to 63 deC
    data[DATA_TEMP] = temp;

    // Enable and start next Temperature conversion
    ADC10CTL0 |= ENC + ADC10SC;
}

/*-----------------------------------------------------------------------------*/
/** @brief     LED pwm set task                                                */
/*-----------------------------------------------------------------------------*/
/** @details
 *      I'm using timer A, channel 1 to do pwm control of the single led on
 *      the launchpad.  Led is turned on at timer overflow and off by the compare
 *      interrupts.
 */

void
led_task(void)
{
    register uint8_t        *data;

    // Get pointer for user data area
    data = viq_i2c_get_data_ptr();

    // Get the LED brightness
    led_pwm = (data[DATA_LED_PWM] << 8);
}

/*-----------------------------------------------------------------------------*/
/** @private                                                                   */
/** @brief     Timer0 interrupt                                                */
/*-----------------------------------------------------------------------------*/

#ifdef __GNUC__
__attribute__((interrupt (TIMER0_A1_VECTOR))) void Timer0_A1_int(void)
#else
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1_int(void)
#endif
{
    uint8_t intstatus = TA0IV;

    switch( intstatus )
        {
        // Timer cap/compare 1
        case    TA0IV_TACCR1:
            // Turn LED off at counter compare if less than max
            if( led_pwm < 0xFD00 )
                P1OUT &= ~BIT0;
            break;
            
        // Timer cap/compare 2
        case    TA0IV_TACCR2:
            break;
            
        // Timer overflow
        case    TA0IV_TAIFG:
            // CC value
            TACCR1 = led_pwm;

            // Enable counter compare interupt unless fully off or on
            // Just doing this so we don't have interrupts really close
            // together.
            if( led_pwm > 0x0300 && led_pwm < 0xFD00 )
                {
                TACCTL1 &= ~CCIFG;
                TACCTL1 |= CCIE;
                }
            else
                TACCTL1 &= ~CCIE;

            // Turn LED on at overflow if it is more than 1
            if( led_pwm > 0x0300 )
                P1OUT |= BIT0;
            else
                P1OUT &= ~BIT0;
            break;
        
        // Everything else is reserved and should not happen
        default:
            break;
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief     Switch read task                                                */
/*-----------------------------------------------------------------------------*/
/** @details
 *     The launchpad only has one user switch connected to P1.3.  The switch is
 *     read here and saved in the user register area.  If we had a multi byte
 *     value to be saved, for example a 32 bit number, synchronization with the i2c
 *     messages would be required to avoid partial numbers being received by the
 *     IQ brain.  One way to check for this would be to monitor the state of the
 *     I2C_Busy flag and write the data on the busy to not_busy transition.
 *     The MSP430 is resource limited, more powerful processors could handle this
 *     by using ping-pong buffers for the user registers or even perhaps mutex or
 *     semaphore techniques if running an RTOS.
 */
 
void
switch_task(void)
{
    register uint8_t        *data;

    // Get pointer for user data area
    data = viq_i2c_get_data_ptr();

    // Set switch status in i2c data
    data[DATA_SWITCH] = (P1IN & BIT3) >> 3;
}

/*-----------------------------------------------------------------------------*/
/** @brief     Setup user IO ports and other peripherals                       */
/*-----------------------------------------------------------------------------*/
/** @details
 *     Every sensor will have hardware specific setup requirements.  This
 *     function is called after the generic port setup code has run to allow
 *     other hardware to be initialized.
 */
 
void
viq_user_port_init()
{
    // Set P1.0 and P1.1 to output direction
    P1DIR |= (BIT0 | BIT1);
    // R34 is not populated on my launchpad so pullup P1.3
    // using internal weak resistor.
    P1REN |= BIT3;
    P1OUT |= BIT3;

    // Timer A setup
    // Use for PWM on the LED attached to P1.0
    // MCLK, Continuous mode, divide by 1
    // PWM rate is 256Hz (16MHz / 0x10000)
    TACTL = TASSEL_2 + MC_2 + ID_0;
    // Initial brightness
    TACCR1 = led_pwm;

    // Enable timer interrupts
    TACCTL1 |= CCIE;
    TACTL   |= TAIE;

    // ADC10 setup
    // Info memory has cal data for 85 and 30 deg C
    adc10_gain = ((85-30) << 8) / (CALADC_15T85 - CALADC_15T30);

    // Sample tmperature sensor
    // Input channel 10, clock divide by 4
    ADC10CTL1 = INCH_10 + ADC10DIV_3;

    // ADC clock sel, select ref 1, ref on, 64 clock S&H time, ADC on
    ADC10CTL0 = ADC10SSEL_0 + SREF_1 + REFON + + ADC10SHT_3 + ADC10ON;

    // Enable and start conversion
    ADC10CTL0 |= ENC + ADC10SC;
}

/*-----------------------------------------------------------------------------*/
/** @private                                                                   */
/** @brief Code main entry                                                     */
/*-----------------------------------------------------------------------------*/

int main(void) {
    bool_t      init = FALSE;

    // Initialize clocks
    viq_clock_init();

    // Initialize digital IO
    viq_port_init();

    // Now detect the enable signal and startup
    viq_startup(TRUE);

    // Add an event on user register 4, this will be EVENT_0
    viq_i2c_add_event( USER_REG_0 + 4 );

    // Add the temperature reading task
    // run twice per second
    viq_task_add( temp_task,  500,"TEMP",       0 );

    // Add led change task triggered by EVENT_0 or 20 seconds
    viq_task_add( led_task , 20000,"LED0", EVENT_0 );

    // run switch reading task 50 times per second
    viq_task_add( switch_task,  20,"SW__",       0 );

    // Loop forever
    while(TRUE)
        {
        //
        init = FALSE;

        // If necessary re-initialize the sensor here
        // not used in this example

        // Let the tasks do all the work
        while(! init)
            {
            init = viq_tasks_check();
            }
        }
}


