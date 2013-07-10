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
 
#include "rfm22b.h"

// Set the frequency of the carrier wave
//	This function calculates the values of the registers 0x75-0x77 to achieve the 
//	desired carrier wave frequency (without any hopping set)
//	Frequency should be passed in integer Hertz
void RFM22B::setCarrierFrequency(unsigned int frequency) {
	// Don't set a frequency outside the range specified in the datasheet
	if (frequency < 240E6 || frequency > 960E6) {
		printf("Cannot set carrier frequency to %fMHz, out of range!",frequency/1E6f);
		return;
	}
	
	// The following determines the register values, see Section 3.5.1 of the datasheet
	
	// Are we in the 'High Band'? (i.e. is hbsel == 1)
	uint8_t hbsel = (frequency >= 480E6);
	
	// What is the integer part of the frequency
	uint8_t fb = frequency/10E6/(hbsel+1) - 24;
	
	// Calculate register 0x75 from hbsel and fb. sbsel (bit 6) is always set
	uint8_t fbs = (1<<6) | (hbsel<<5) | fb;
	
	// Calculate the fractional part of the frequency
	uint16_t fc = (frequency/(10E6f*(hbsel+1)) - fb - 24) * 64000;
	
	// Split the fractional part in to most and least significant bits
	// (Registers 0x76 and 0x77 respectively)
	uint8_t ncf1 = (fc >> 8);
	uint8_t ncf0 = fc & 0xff;

	// Write the registers to the device
	this->writeByte(0x75, fbs);
	this->writeByte(0x76, ncf1);
	this->writeByte(0x77, ncf0);
}

// Get the frequency of the carrier wave in integer Hertz
//	Without any frequency hopping
unsigned int RFM22B::getCarrierFrequency() {
	// Read the register values
	uint8_t fbs = this->readByte(0x75);
	uint8_t ncf1 = this->readByte(0x76);
	uint8_t ncf0 = this->readByte(0x77);

	// The following calculations ceom from Section 3.5.1 of the datasheet
	
	// Determine the integer part
	uint8_t fb = fbs & 0x1F;
	
	// Are we in the 'High Band'?
	uint8_t hbsel = (fbs >> 5) & 1;

	// Determine the fractional part
	uint16_t fc = (ncf1 << 8) | ncf0;

	// Return the frequency
	return 10E6*(hbsel+1)*(fb+24+fc/64000.0);
}

// Helper function to read a single byte from the device
uint8_t RFM22B::readByte(uint8_t reg) {
	// rx and tx arrays must be the same length
	// Must be 2 elements as the device only responds whilst it is being sent
	// data. tx[0] should be set to the requested register value and tx[1] left
	// clear. Once complete, rx[0] will be left clear (no data was returned whilst
	// the requested register was being sent), and rx[1] will contain the value
	uint8_t tx[] = {0x00, 0x00};
	uint8_t rx[] = {0x00, 0x00};
	
	tx[0] = reg;
	
	this->transfer(tx,rx,2);
	
	return rx[1];
}

// Helper function to write a single byte to a register
void RFM22B::writeByte(uint8_t reg, uint8_t value) {
	// tx and rx arrays required even though we aren't receiving anything
	uint8_t tx[] = {0x00, 0x00};
	uint8_t rx[] = {0x00, 0x00};
	
	// tx[0] is the requested register with the final bit set high to indicate
	// a write operation (see Section 3.1 of the datasheet)
	tx[0] = reg | (1<<7);
	
	// tx[1] is the value to be set
	tx[1] = value;
	
	this->transfer(tx,rx,2);
}