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

/* This is a simple test program for the RFM22B class
 *	Currently all it can do it get and set the carrier frequency of the module
 *	as this is all the functionality available from the class. More coming soon...
 */
 
#include "rfm22b.h"

int main() {
	// Initialise the radio
	RFM22B *myRadio = new RFM22B("/dev/spidev1.0");
	
	// Set the bus speed
	myRadio->setMaxSpeedHz(200000);
	
	// Radio configuration
	myRadio->reset();
	myRadio->setCarrierFrequency(869.5E6);
	myRadio->setModulationType(RFM22B::GFSK);
	myRadio->setModulationDataSource(RFM22B::FIFO);
	myRadio->setDataClockConfiguration(RFM22B::NONE);
	myRadio->setTransmissionPower(5);
	myRadio->setGPIOFunction(RFM22B::GPIO0, RFM22B::TX_STATE);
	myRadio->setGPIOFunction(RFM22B::GPIO1, RFM22B::RX_STATE);
	
	// What header do we want?
	myRadio->setCheckHeader(123456789);
	
	// Listen for a packet
	printf("Listening to a message...\n");
	
	char input[RFM22B::MAX_PACKET_LENGTH];
	myRadio->receive((uint8_t*)input,RFM22B::MAX_PACKET_LENGTH);
	printf("%s\n", input);
		
	myRadio->close();
}