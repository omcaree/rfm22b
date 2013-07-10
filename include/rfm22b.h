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

/* Class to control the HopeRF RFM22B module
 *	Inherits SPI functionality from SPI class
 *	This class is a work in progress, at present all it can do is get/set the carrier
 *	frequency. This has been tested on the 868MHz modules but should work for all
 *	frequencies (High or Low bands 24-47, See Table 12 of the datasheet).
 */
 
#ifndef rfm22b_h
#define rfm22b_h

#include "spi.h"

class RFM22B : public SPI {
public:
	// Constructor requires SPI device path, passes this is SPI class
	RFM22B(const char *device) : SPI(device) {}
	
	// Set or get the carrier frequency (in Hz);
	void setCarrierFrequency(unsigned int frequency);
	unsigned int getCarrierFrequency();
private:
	// Helper functions for getting ad getting indivisual bytes to the bus
	uint8_t readByte(uint8_t reg);
	void writeByte(uint8_t reg, uint8_t value);
};

#endif