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
#include "drivers/fatfs/diskio.h"
#include "drivers/fatfs/ff.h"

/**************************************************************************/
/*!
    Declarations
*/
/**************************************************************************/

uint8_t word[] = "hello there!";
UINT readBytes = 0;
bool uartHasData = false;
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
/**************************************************************************/
/*!
    Functions.
*/
/**************************************************************************/

//stop looping in case file operations error
//and turn on led
void checkRes(bool *res) {
  if (*res != FR_OK){
	  gpioSetValue(CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON);
  }
}

// Toggle led once per second in case normal UART reading and writing to SD
void blinking(uint32_t *current, uint32_t *last, uint32_t *uartDataReseived) {
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
void itoa(char *arr, uint32_t *num) {
	int i = 3;
	do {
		arr[i--] = (*num % 10) + '0';
		*num /= 10;
	} while (*num);
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
	uartSend((uint8_t *)word, sizeof(word));
	uartHasData = uartRxBufferReadArray(uartReadArr, &readBytes);

  	if (uartHasData){
  		res = f_open (&fil, fileName, FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
  		checkRes(&res);
  		res = f_lseek(&fil, (&fil)->fsize);
  		checkRes(&res);
  		res = f_write (&fil, uartReadArr, readBytes, &writtenBytes);
  		checkRes(&res);
  		f_close(&fil);

  		uartDataReseivedSecond = systickGetSecondsActive();
  	}

  	blinking(&currentSecond, &lastSecond, &uartDataReseivedSecond);
  }

  return 0;
}
