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
 
#include "spi.h"
#include <string.h>

// Constructor
//	Opens the SPI device and sets up some default values
//	Default bits per word is 8
//	Default clock speed is 10kHz
SPI::SPI(const char *device) {
	this->device = device;
	this->fd = open(this->device, O_RDWR);
	if (fd < 0) {
		perror("Unable to open SPI device");
	}
	this->setMode(0);
	this->setBitsPerWord(8);
	this->setMaxSpeedHz(100000);
	this->setDelayUsecs(0);
}

// Set the mode of the bus (see linux/spi/spidev.h)
void SPI::setMode(uint8_t mode) {
	int ret = ioctl(this->fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		perror("Unable to set SPI mode");

	ret = ioctl(this->fd, SPI_IOC_RD_MODE, &this->mode);
	if (ret == -1)
		perror("Unable to get SPI mode");
}

// Get the mode of the bus
uint8_t SPI::getMode() {
	return this->mode;
}

// Set the number of bits per word
void SPI::setBitsPerWord(uint8_t bits) {
	int ret = ioctl(this->fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		perror("Unable to set bits per word");

	ret = ioctl(this->fd, SPI_IOC_RD_BITS_PER_WORD, &this->bits);
	if (ret == -1)
		perror("Unable to get bits per word");
}

// Get the number of bits per word
uint8_t SPI::getBitsPerWord() {
	return this->bits;
}

// Set the bus clock speed
void SPI::setMaxSpeedHz(uint32_t speed) {
	int ret = ioctl(this->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		perror("Unable to set max speed Hz");

	ret = ioctl(this->fd, SPI_IOC_RD_MAX_SPEED_HZ, &this->speed);
	if (ret == -1)
		perror("Unable to get max speed Hz");
}

// Get the bus clock speed
uint32_t SPI::getMaxSpeedHz() {
	return this->speed;
}

// Set the bus delay
void SPI::setDelayUsecs(uint16_t delay) {
	this->delay = delay;
}

// Get the bus delay
uint16_t SPI::getDelayUsecs() {
	return this->delay;
}

// Transfer some data
bool SPI::transfer(uint8_t *tx, uint8_t *rx, int length) {
   struct spi_ioc_transfer tr;

   memset(&tr,0,sizeof(tr));
   tr.tx_buf = (unsigned long)tx;	//tx and rx MUST be the same length!
   tr.rx_buf = (unsigned long)rx;
   tr.len = length;
   tr.delay_usecs = this->delay;
   tr.speed_hz = this->speed;       
   tr.bits_per_word = this->bits;
   tr.cs_change = 0;

   int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
   if (ret == 1) {
      perror("Unable to send spi message");
	  return false;
	}
	return true;
}

// Close the bus
void SPI::close() {
   ::close(this->fd);
}
