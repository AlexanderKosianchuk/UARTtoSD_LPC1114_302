/**************************************************************************/
/*! 
    @file     main.c
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2011, microBuilder SARL
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include "projectconfig.h"
#include "sysinit.h"
#include "mpu9150.h"

#include "core/gpio/gpio.h"
#include "core/systick/systick.h"

#include "core/uart/uart.h"

#include "core/timer32/timer32.h"
#include "core/ssp/ssp.h"
#include "core/i2c/i2c.h"
#include "drivers/fatfs/diskio.h"
#include "drivers/fatfs/ff.h"


/**************************************************************************/
/*!
    Declarations
*/
/**************************************************************************/

extern volatile uint32_t I2CMasterState;
extern volatile uint32_t I2CSlaveState;

extern volatile uint8_t I2CMasterBuffer[I2C_BUFSIZE];    // Master Mode
extern volatile uint8_t I2CSlaveBuffer[I2C_BUFSIZE];     // Master Mode

extern volatile uint32_t I2CReadLength;
extern volatile uint32_t I2CWriteLength;

extern volatile uint32_t RdIndex;
extern volatile uint32_t WrIndex;

extern volatile uint32_t _I2cMode;                       // I2CMASTER or I2CSLAVE

static bool _MPU9150Initialised = false;

int SLAVE_ADDRESS = 0xA6UL;
int DEVID = 0x00UL;

typedef enum
{
  MPU9150_ERROR_OK = 0,                // Everything executed normally
  MPU9150_ERROR_I2CINIT = 1,               // Unable to initialise I2C
  MPU9150_ERROR_I2CBUSY = 2,               // I2C already in use
  MPU9150_ERROR_NOCONNECTION = 3,          // Unable to read device ID during init
  MPU9150_ERROR_LAST = 4
}
MPU9150Error_t;

/**************************************************************************/
/*!
    Main program entry point.
*/
/**************************************************************************/

void clearBuff() {
	for ( i = 0; i < I2C_BUFSIZE; i++ )
	{
		I2CMasterBuffer[i] = 0x00;
	}

	for ( i = 0; i < I2C_BUFSIZE; i++ )
	{
		I2CSlaveBuffer[i] = 0x00;
	}
}

/*==========================*/

int main(void)
{
	systemInit();

	if (i2cInit( (uint32_t)I2CMASTER ) == FALSE )	/* initialize I2c */
	{
		while ( 1 );				/* Fatal error */
	}

	clearBuff();

	//read device id to check ADXL375
	int tryReadGyroCount = 0;
	bool gyroOK = false;
	do {
		I2CWriteLength = 2;
		I2CReadLength = 0;
		I2CMasterBuffer[0] = SLAVE_ADDRESS;
		I2CMasterBuffer[1] = DEVID;
		i2cEngine();

		I2CWriteLength = 1;
		I2CReadLength = 1;
		I2CMasterBuffer[0] = SLAVE_ADDRESS | MPU9150_READBIT;
		i2cEngine();

		systickDelay(10);

	} while((I2CMasterState == 259) && (tryReadGyroCount <= 100));

	if(tryReadGyroCount < 100) {
		gyroOK = true;
	}



	// Shift values to create properly formed integer
	uint8_t value = I2CSlaveBuffer[0];

	while ( 1 );
	return 0;
}
