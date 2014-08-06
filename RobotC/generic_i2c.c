/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     generic_i2c.c                                                */
/*    Author:     James Pearman                                                */
/*    Created:    20 June 2014                                                 */
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

#ifndef __GENERIC_I2C__
#define __GENERIC_I2C__

#pragma systemFile

/*-----------------------------------------------------------------------------*/
/** @file    generic_i2c.c
 *  @brief   Vex IQ generic sensor communication
 */

#define I2C_STATUS_TIMEOUT  10
#define vexIQ_SensorUSER    255

/*-----------------------------------------------------------------------------*/
/** @brief Write registers in the I2C sensor                                   */
/** @param[in] port the I2C port                                               */
/** @param[in] addr the sensor register to start writing to                    */
/** @param[in] buf pointer to buffer with the register data                    */
/** @param[in] len the number of bytes to write to the sensor                  */
/*-----------------------------------------------------------------------------*/

void
genericI2cWrite( portName port, int addr, char *buf, int len )
{
    TVexIqI2CResults     status;
    unsigned long        timeout = nSysTime + I2C_STATUS_TIMEOUT;

    // bounds check address and length
    if( addr < 0 || addr+len > 255 )
        return;

    // make sure bus is free
    status = vexIqGetI2CStatus( port );
    while(( status == i2cRsltBusy ) && (nSysTime < timeout) )
        {
        abortTimeslice();
        status = vexIqGetI2CStatus( port );
        }

    // Under these conditions send the message
    if( (status == i2cRsltIdleAndOK) || (status == i2cRsltIdleAndFailed) ||
        (status == i2cRsltInvalidBufferStatus) || (status == i2cRsltTimedOut) )
        {
        StartI2CDeviceBytesWrite( port, addr, buf, len );
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief Read registers from the I2C sensor                                  */
/** @param[in] port the I2C port                                               */
/** @param[in] addr the sensor register to start reading from                  */
/** @param[in] buf pointer to storage to save the register data                */
/** @param[in] len the number of bytes to read from the sensor                 */
/*-----------------------------------------------------------------------------*/

void
genericI2cRead( portName port, int addr, char *buf, int len )
{
    TVexIqI2CResults     status;
    unsigned long        timeout = nSysTime + I2C_STATUS_TIMEOUT;

    // bounds check address and length
    if( addr < 0 || addr+len > 255 )
        return;

    // make sure bus is free
    status = vexIqGetI2CStatus( port );
    while(( status == i2cRsltBusy ) && (nSysTime < timeout) )
        {
        abortTimeslice();
        status = vexIqGetI2CStatus( port );
        }

    // Under these conditions send the message
    if( (status == i2cRsltIdleAndOK) || (status == i2cRsltIdleAndFailed) ||
        (status == i2cRsltInvalidBufferStatus) || (status == i2cRsltTimedOut) )
        {
        // Send read register message
        StartI2CDeviceBytesRead( port, addr, len );

        // wait for bus to be free, message is about 10 bytes/mS
        timeout = nSysTime + (len/10) + I2C_STATUS_TIMEOUT;
        status = vexIqGetI2CStatus( port );
        while(( status == i2cRsltBusy ) && (nSysTime < timeout) )
            {
            abortTimeslice();
            status = vexIqGetI2CStatus( port );
            }

        // check status
        status = vexIqGetI2CStatus( port );
        if(status == i2cRsltIdleAndOK)
            // Get the data returned from the sensor
            StoreI2CDeviceBytesReadFromPortBuffer( port, buf, len );
        }
}

/*-----------------------------------------------------------------------------*/
/** @brief  Find the first generic sensor installed                            */
/** @returns the port number if found else (-1)                                */
/*-----------------------------------------------------------------------------*/
/**
 * @details
 *  scan all ports looking for an installed generic sensor
 */

portName
genericI2cFindFirst()
{
    TVexIQDeviceTypes   type;
    TDeviceStatus       status;
    short               ver;
    portName            index;

    // Get all device info
    for(index=PORT1;index<=PORT12;index++)
        {
        getVexIqDeviceInfo( index, type, status, ver );
        if( (char)type == vexIQ_SensorUSER )
            return((portName)index);
        }
    return((portName)-1);
}

/*-----------------------------------------------------------------------------*/
/** @brief  Find the next generic sensor installed                             */
/** @returns the port number if found else (-1)                                */
/*-----------------------------------------------------------------------------*/
/**
 * @details
 *  scan all ports looking for an installed generic sensor
 */

portName
genericI2cFindNext( portName port )
{
    TVexIQDeviceTypes   type;
    TDeviceStatus       status;
    short               ver;
    portName            index;

    // bounds check
    if( port > PORT12 )
        return((portName)-1);

    // Get all device info
    for(index=port;index<=PORT12;index++)
        {
        getVexIqDeviceInfo( index, type, status, ver );
        if( (char)type == vexIQ_SensorUSER )
            return((portName)index);
        }
    return((portName)-1);
}

#endif
