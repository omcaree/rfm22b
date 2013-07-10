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
	RFM22B *myRadio = new RFM22B("/dev/spidev1.0");
	myRadio->setMaxSpeedHz(100000);
	
	printf("Frequency is %.2fMHz\n",myRadio->getCarrierFrequency()/1E6f);
	
	myRadio->setCarrierFrequency(869E6);

	printf("Frequency is %.2fMHz\n",myRadio->getCarrierFrequency()/1E6f);
	
	myRadio->close();
}