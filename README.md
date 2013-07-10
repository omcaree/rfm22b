RFM22B C++ Class for Linux
==========================

This project aims to be a complete C++ interface to the HopeRF RFM22B wireless transceiver. At the moment it doesn't do much, just a basic test of the SPI bus code.

rfm22b_test.cpp is provided to demonstrate the baisc functionality of getting and setting the Nominal Carrier Wave Frequency (implemented from Section 3.5.1 of the [datasheet](http://www.hoperf.com/upload/rf/RFM22B_23B.pdf)). It looks like this:

	#include "rfm22b.h"

	int main() {
		RFM22B *myRadio = new RFM22B("/dev/spidev1.0");
		myRadio->setMaxSpeedHz(100000);
		
		printf("Frequency is %.2fMHz\n",myRadio->getCarrierFrequency()/1E6f);
		
		myRadio->setCarrierFrequency(869E6);

		printf("Frequency is %.2fMHz\n",myRadio->getCarrierFrequency()/1E6f);
		
		myRadio->close();
	}
	
You can change the SPI device path and desired frequency as you require. To build it, simply type

	make

The code must run as root in order to get permission to access the SPI device (or change the permissions of the device, it's up to you). I have been testing on the 868MHz module (which comes preset to 915MHz), so this is the output:

	ubuntu@arm:~/rfm22b$ sudo ./rfm22b_test
	Frequency is 915.00MHz
	Frequency is 869.00MHz


