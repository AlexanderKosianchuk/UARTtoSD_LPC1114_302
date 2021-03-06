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

#include "core/gpio/gpio.h"
#include "core/systick/systick.h"

#include "core/uart/uart.h"

#include "core/timer32/timer32.h"
#include "core/ssp/ssp.h"
#include "core/i2c/i2c.h"
#include "drivers/fatfs/diskio.h"
#include "drivers/fatfs/ff.h"

#include "ADXL375.h"

/**************************************************************************/
/*!
    Declarations
*/
/**************************************************************************/

uint8_t word[] = "hello there!";
UINT readBytes = 0;
bool uartHasData = false, frameFound = false;
uint8_t uartReadArr [CFG_UART_BUFSIZE] = {0};

// variables for fatfs operation
FRESULT res;
FIL fil;				/* File object */
DIR dir;				/* Pointer to the open directory object */
FILINFO fileInfo;		/* Pointer to file information to return */
static FATFS Fatfs[1];
UINT writtenBytes = 0;
uint32_t fileNameCounter = 0;
char fileName[] = {'0','0','0','0','l','u','c','h','.','d','a','t','\0'};

//vars for blinking
uint32_t currentSecond, lastSecond, uartDataReseivedSecond;
uint32_t cardWrittingErrorCount = 0;

/**************************************************************************/
/*!
    GYRO Declarations
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

int ADXL375_FIFO_streamMode = 0x0FUL; // stream mode 16 samples

/**************************************************************************/
/*!
    Functions.
*/
/**************************************************************************/

//stop looping in case file operations error
//and turn on led
void checkRes(FRESULT *res)
{
  if (*res != FR_OK){
	  gpioSetValue(CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON);

	  // try to mount card again to continue writing
	do {
		*res = f_mount(0, &Fatfs[0]);
		systickDelay(1000);
	} while (*res != FR_OK);

  }
}

// Toggle led once per second in case normal UART reading and writing to SD
void blinking(uint32_t *current, uint32_t *last, uint32_t *uartDataReseived)
{
	*current = systickGetSecondsActive();
	if(*uartDataReseived > (*current - 10)) {
		if (*current != *last)
		{
		  *last = *current;
		  gpioSetValue(CFG_LED_PORT, CFG_LED_PIN, lastSecond % 2);
		}
	} else {
		//if last uart transmission was gather than 10 seconds ago
		//turn off led
		 gpioSetValue(CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF);
	}

}

// int to char
void itoa(char *arr, uint32_t *num)
{
	int i = 3;
	do {
		arr[i--] = (*num % 10) + '0';
		*num /= 10;
	} while (*num);
}

//if frame found returns start index
//else returns -1
int findStartFrame(uint8_t *uartReadArr[], UINT *readBytes)
{
	uint8_t jj = 0;
	for(jj = 0; jj < (*readBytes - 1); jj++) {
		if((*uartReadArr[jj] == 0xff) && (uartReadArr[jj] == 0xff)) {
			return jj;
		}
	}
	return -1;
}

void writeArrayToCard(FIL fil, char fileName [], uint8_t arr [], UINT count)
{
	res = f_open (&fil, fileName, FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
	checkRes(&res);
	res = f_lseek(&fil, (&fil)->fsize);
	checkRes(&res);
	res = f_write (&fil, arr, count, &writtenBytes);
	checkRes(&res);
	f_close(&fil);
}

void clearBuff() {
	int i = 0;
	for ( i = 0; i < I2C_BUFSIZE; i++ )
	{
		I2CMasterBuffer[i] = 0x00;
	}

	for ( i = 0; i < I2C_BUFSIZE; i++ )
	{
		I2CSlaveBuffer[i] = 0x00;
	}
}

/**************************************************************************/
/*!
    Main program entry point.
*/
/**************************************************************************/
int main(void)
{
  // Configure cpu and mandatory peripherals
  systemInit();
  res = f_mount(0, &Fatfs[0]);
  checkRes(&res);

  res = f_opendir (&dir,(XCHAR *)'/');
  checkRes(&res);

  fileNameCounter = 0;
  do {
	  res = f_readdir (&dir, &fileInfo);
	  checkRes(&res);
	  fileNameCounter++;
  } while((res == FR_OK) && (fileInfo.fname[0] != 0));

  itoa(&fileName, &fileNameCounter);

  currentSecond = lastSecond = uartDataReseivedSecond = 0;

  while (1)
  {
	//uartSend((uint8_t *)word, sizeof(word));
	uartHasData = uartRxBufferReadArray(uartReadArr, &readBytes);

		//if we still not found frame start
		//when found dont search again,
		//just write all from uart

//		if (uartHasData == true) {
//			writeArrayToCard(fil, fileName, &uartReadArr, &readBytes);
//			uartDataReseivedSecond = systickGetSecondsActive();
//	  	}

		clearBuff();

		//read device id to check ADXL375
		int tryReadGyroCount = 0;
		bool gyroOK = false;
		do {
			I2CWriteLength = 3;
			I2CReadLength = 0;
			I2CMasterBuffer[0] = SLAVE_ADDR;
			I2CMasterBuffer[1] = ADXL375_FIFO_CTL;
			I2CMasterBuffer[2] = 0x4FUL;//ADXL375_FIFO_streamMode;
			i2cEngine();
			clearBuff();

			do {
				I2CWriteLength = 2;
				I2CReadLength = 0;
				I2CMasterBuffer[0] = SLAVE_ADDR;
				I2CMasterBuffer[1] = ADXL375_FIFO_STATUS;
				i2cEngine();
				clearBuff();

				I2CWriteLength = 1;
				I2CReadLength = 1;
				I2CMasterBuffer[0] = SLAVE_ADDR | READ_WRITE;
				i2cEngine();

				systickDelay(100);
			} while(I2CSlaveBuffer[0] != 128);

			int jo = I2CSlaveBuffer[0];

			I2CWriteLength = 2;
			I2CReadLength = 0;
			I2CMasterBuffer[0] = SLAVE_ADDR;
			I2CMasterBuffer[1] = ADXL375_FIFO_CTL;
			i2cEngine();
			clearBuff();

			I2CWriteLength = 1;
			I2CReadLength = 16;
			I2CMasterBuffer[0] = SLAVE_ADDR | READ_WRITE;
			i2cEngine();

			jo = I2CSlaveBuffer[0];

		} while((I2CMasterState == 259) && (tryReadGyroCount <= 100));

		if(tryReadGyroCount < 100) {
			gyroOK = true;
		}


//		if ((uartHasData == true) && (frameFound == false)){
//
//			if(frameFound) {
//				writeArrayToCard(&fileName, &uartReadArr, &readBytes);
//				uartDataReseivedSecond = systickGetSecondsActive();
//			} else {
//				int frameFoundIndex = -1;
//				frameFoundIndex = findStartFrame(&uartReadArr, &readBytes);
//
//				if(frameFoundIndex >= 0) {
//					frameFound = true;
//				}
//			}
//		} else if ((uartHasData == true) && frameFound == true){
//			writeArrayToCard(&fileName, &uartReadArr, &readBytes);
//			uartDataReseivedSecond = systickGetSecondsActive();
//		}
//  	}

  	blinking(&currentSecond, &lastSecond, &uartDataReseivedSecond);
  }

  return 0;
}
