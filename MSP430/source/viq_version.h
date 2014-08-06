/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     viq_version.h                                                */
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
/// @file    viq_version.h
/// @brief   Vex IQ sensor version information
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/** @brief This sensor firmware version                                        */
/*-----------------------------------------------------------------------------*/
#define VIQ_VERSION           1

// All strings must be 8 characters
#define VIQ_VERSION_STR       "V1.00.00"    ///< Sensor version string
#define VIQ_VENDOR_STR        "VEX IQ  "    ///< Sensor Vendor string
#define VIQ_ID_STR            "GENERIC "    ///< Sensor part number

// Do not use this - it is reserved
#define VIQ_RSV_STR           "      "      ///< Reserved header space

// A user sensor uses two bytes to define its function
// This follows the same idea as USB but only uses one byte for
// Each part of the vid/pid information
// For compatibility with vex sensors, make these values display ascii
// (0x20 through 0x7E)
// This demo code used vendor 'P' and product '1' (Pearman 1)
// but is completely arbitrary at this point
#define VIQ_USER_VENDOR       0x50          ///< Manufacturer of this sensor
#define VIQ_USER_PRODUCT      0x31          ///< Product id for this user sensor
        
/*-----------------------------------------------------------------------------*/
/** @name
 *
 *  @details
 *     The vex sensor operational modes are included for information only.  
 *  @warning
 *     All user created sensors should use the RAM_MODE mode.
 *  @{
 *
 *  @defgroup devMode VEX IQ Operating mode
 *  @{
 */
#define BOOT_MODE             0     ///< Bootload code
#define RAM_MODE              1     ///< Normal user code
#define TEST_MODE             3     ///< Functional Test Code
/** @} */
/** @} */

/*-----------------------------------------------------------------------------*/
/** @name
 *
 *  @details
 *     The vex sensor ids are included for information only.
 *  @warning
 *     All user created sensors should have the USER_SENSOR id.
 *  @{
 *
 *  @defgroup devId VEX IQ Device ID
 *  @{
 */

#define MOTOR_SENSOR          2     ///< VEX IQ Motor
#define LED_SENSOR            3     ///< VEX IQ Touch LED
#define RGB_SENSOR            4     ///< VEX IQ color sensor
#define BUMPER_SENSOR         5     ///< VEX IQ Bumper switch
#define GYRO_SENSOR           6     ///< VEX IQ Gyro
#define SONAR_SENSOR          7     ///< VEX IQ ultrasonic sensor

/*-----------------------------------------------------------------------------*/
/*      All user sensors should only use USER_SENSOR id                        */
/*-----------------------------------------------------------------------------*/
/// @brief User sensor id
#define USER_SENSOR           255

/** @} */
/** @} */

/*-----------------------------------------------------------------------------*/
/** @brief  Holds static version information for this sensor                   */
/*-----------------------------------------------------------------------------*/
/** @details
 *      To save memory, version information is hard coded into ROM
 *      the vendor_id and product_id fields are specific to user sensors and
 *      will both be set to 0x20 by the VEX sensors.
 */
 
typedef struct _version_t {
    uint8_t     version_str[8];     ///< Version string
    uint8_t     vendor_str[8];      ///< Vendor string
    uint8_t     device_str[8];      ///< Device string (often part number)
    uint8_t     vendor_id;          ///< Vendor id as single byte (user sensors only)
    uint8_t     product_id;         ///< Product id as single byte (user sensors only)
    uint8_t     reserved_str[6];    ///< do not use
    uint8_t     version;            ///< version as single byte
    uint8_t     mode;               ///< sensor mode, only allowable value is RAM_MODE
    uint8_t     id;                 ///< sensor id, only allowable value is USER_SENSOR
    uint8_t     pad;                ///< pad for word alignment
} version_t;

/*-----------------------------------------------------------------------------*/
/** @brief Initialization for the version information                          */

const version_t version_data = 
  { 
  VIQ_VERSION_STR,
  VIQ_VENDOR_STR,
  VIQ_ID_STR,
  VIQ_USER_VENDOR,
  VIQ_USER_PRODUCT,
  VIQ_RSV_STR,
  VIQ_VERSION,
  RAM_MODE,
  USER_SENSOR, 0 };

/*-----------------------------------------------------------------------------*/
