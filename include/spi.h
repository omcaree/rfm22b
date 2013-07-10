/* Copyright (c) 2013 Owen McAree
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/* Serial Peripheral Interface (SPI) bus Class
 *	This class allows you to take control of devices on the SPI bus from Linux
 *	It has been developed and tested on the BeagleBone Black but should work on
 *	any Linux system with spidev support (e.g. Raspberry Pi)
 *
 *	Usage is simple...
 *	To create an instance, point the constructor at your SPI bus
 *		SPI *myBus = new SPI("/dev/spidev1.0");
 *
 *	Set the speed of the bus (or leave it at the 100kHz default)
 *		myBus->setMaxSpeedHz(1000000);
 *
 *	Transfer some data
 *		uint8_t tx[] = {0x55, 0x00};
 *		uint8_t rx[] = {0x00, 0x00};
 *		myBus->transfer(tx, rx, 2);
 *	Note that tx and rx arrays must be the same size
 *	(the size is passed as the 3rd parameter to 'transfer')
 *
 *	Close the bus
 *		myBus->close();
 */
 
#ifndef spi_h
#define spi_h

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

class SPI {
public:
	// Constructor, device path required
	SPI(const char *device);
	
	// Set or get the SPI mode
	void setMode(uint8_t mode);
	uint8_t getMode();
	
	// Set or get the bits per word
	void setBitsPerWord(uint8_t bits);
	uint8_t getBitsPerWord();
	
	// Set or get the SPI clock speed
	void setMaxSpeedHz(uint32_t speed);
	uint32_t getMaxSpeedHz();
	
	// Set or get the SPI delay
	void setDelayUsecs(uint16_t delay);
	uint16_t getDelayUsecs();
	
	// Transfer some data
	//	tx:	Array of bytes to be transmitted
	//	rx: Array of bytes to be received
	//	length:	Length of arrays (must be equal)
	// If you just want to send data you still need to pass in
	// an rx array, but you can safely ignore its output
	// Returns true if transfer was successful (false otherwise)
	bool transfer(uint8_t *tx, uint8_t *rx, int length);
	
	// Close the bus
	void close();
private:
	const char *device;
	int fd;
	uint8_t mode;
	uint8_t bits;
	uint32_t speed;
	uint16_t delay;
};

#endif