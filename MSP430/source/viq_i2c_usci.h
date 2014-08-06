/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     viq_i2c_usci.h                                               */
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

#ifndef __VIQ_I2C_USCI__
#define __VIQ_I2C_USCI__

/*-----------------------------------------------------------------------------*/
/// @file    viq_i2c_usci.h
/// @brief   Vex IQ sensor USCI interface code header
/*-----------------------------------------------------------------------------*/

/// macro to change slave address
#define viq_i2c_usci_setslaveaddress(addr)  UCB0I2COA = UCGCEN | ((addr) >> 1)

#ifdef __cplusplus
extern "C" {
#endif

void    viq_i2c_usci_init( i2cDataStruct *data );
void    viq_i2c_usci_init_slave( void );

#ifdef __cplusplus
}
#endif

#endif  // __VIQ_I2C_USCI__
