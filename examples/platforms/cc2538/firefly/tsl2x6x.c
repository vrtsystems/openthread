/*
 *  Copyright (c) 2017, Zolertia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements a basic Grove digital light sensor driver
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "../cc2538-reg.h"
#include "../i2c.h"
#include "../gpio.h"
#include "tsl2x6x.h"

#define LUX_SCALE           14     // scale by 2^14
#define RATIO_SCALE         9      // scale ratio
#define CH_SCALE            10     // scale channel values by 2^10

static int tsl2x6xCalculateLux(uint8_t *buf)
{
    uint32_t ch0, ch1, chscale = 0;
    uint32_t ratio = 0;
    uint32_t lratio, tmp = 0;
    uint16_t buffer[2];

    buffer[0] = (buf[1] << 8 | (buf[0]));
    buffer[1] = (buf[3] << 8 | (buf[2]));

    // 420ms no gain

    chscale = (1 << CH_SCALE);
    chscale = chscale << 4;

    ch0 = (buffer[0] * chscale) >> CH_SCALE;
    ch1 = (buffer[1] * chscale) >> CH_SCALE;

    if (ch0 > 0)
    {
        ratio = (ch1 << CH_SCALE);
        ratio = ratio / ch0;
    }

    lratio = (ratio + 1) >> 1;

    if (lratio <= K1T)
    {
        tmp = (ch0 * B1T) - (ch1 * M1T);
    }
    else if (lratio <= K2T)
    {
        tmp = (ch0 * B2T) - (ch1 * M2T);
    }
    else if (lratio <= K3T)
    {
        tmp = (ch0 * B3T) - (ch1 * M3T);
    }
    else if (lratio <= K4T)
    {
        tmp = (ch0 * B4T) - (ch1 * M4T);
    }
    else if (lratio <= K5T)
    {
        tmp = (ch0 * B5T) - (ch1 * M5T);
    }
    else if (lratio <= K6T)
    {
        tmp = (ch0 * B6T) - (ch1 * M6T);
    }
    else if (lratio <= K7T)
    {
        tmp = (ch0 * B7T) - (ch1 * M7T);
    }
    else if (lratio > K8T)
    {
        tmp = (ch0 * B8T) - (ch1 * M8T);
    }

    tmp += (1 << (LUX_SCALE - 1));
    return (int)(tmp >> LUX_SCALE);
}

static int tsl2x6xWriteReg(uint8_t *buf, uint8_t num)
{
    return cc2538I2cBurstSend(TSL2X6X_ADDR, buf, num);
}

static int tsl2x6xReadReg(uint8_t reg, uint8_t *buf, uint8_t regNum)
{
    cc2538I2cMasterEnable();
    if (cc2538I2cSingleSend(TSL2X6X_ADDR, reg) == I2C_MASTER_ERR_NONE)
    {
      while(cc2538I2cBusy());
      return cc2538I2cBurstReceive(TSL2X6X_ADDR, buf, regNum);
    }
    return I2CM_STAT_ERROR;
}

int tsl2x6xReadLight(int *lux)
{
    uint8_t buf[4];
    buf[0] = (TSL2X6X_COMMAND + TSL2X6X_CONTROL);
    buf[1] = TSL2X6X_CONTROL_POWER_ON;

    cc2538I2cPinInit(GPIO_C_NUM, 2, GPIO_C_NUM, 3);
    cc2538I2cMasterEnable();

    if (tsl2x6xWriteReg(buf, 2) == I2C_MASTER_ERR_NONE)
    {
        if (cc2538I2cSingleReceive(TSL2X6X_ADDR, &buf[0]) == I2C_MASTER_ERR_NONE)
         {
            if ((buf[0] & 0x0F) == TSL2X6X_CONTROL_POWER_ON)
            {
                if (tsl2x6xReadReg((TSL2X6X_COMMAND + TSL2X6X_D0LOW), &buf[0], 2) == I2C_MASTER_ERR_NONE)
                {
                    if (tsl2x6xReadReg((TSL2X6X_COMMAND + TSL2X6X_D1LOW), &buf[2], 2) == I2C_MASTER_ERR_NONE)
                    {
                        *lux = tsl2x6xCalculateLux(buf);
                        return I2C_MASTER_ERR_NONE;
                    }
                }
            }
        }
    }
    return I2CM_STAT_ERROR;
}

