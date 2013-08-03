RFM22B C++ Class for Linux
==========================

This project aims to be a complete Linux C++ class for the (HopeRF RFM22B wireless transceiver)[http://www.hoperf.com/rf_fsk/fsk/rfm22b.htm]. Setting the majority of control registers is handled through convenient member functions with the options defined as enums.

It has been developed on a pair of BeagleBone Blacks running Ubuntu, but it should work perfectly well on any Linux device with an SPI bus (e.g. Raspberry Pi). A simple SPI class is included to abstract this functionality, therefore with a little work this should be portable to other operating systems.

Examples
========
A few simple test programs are included to demonstrate basic useage. You may need to modify the path to the SPI device (/dev/spidev1.0 by default) in the examples. Once this is done simply build them with

	make

rfm22b_setup_test
-----------------
This example demonstrates the getting and setting of various configuration registers (full list available below). The output will look something like this...

	Initial Settings...
			Frequency is 915.000MHz
			FH Step is 0.000kHz
			Channel is 0
			Frequency deviation is 20.000kHz
			Data rate is 39.993kbps
			Modulation Type 0
			Modulation Data Source 0
			Data Clock Configuration 0
			Transmission Power is 1dBm
			GPIO0 Function is 0x00
			GPIO1 Function is 0x00
			GPIO2 Function is 0x00

	New Settings...
			Frequency is 869.000MHz
			FH Step is 20.000kHz
			Channel is 3
			Frequency deviation is 62.500kHz
			Data rate is 14.999kbps
			Modulation Type 3
			Modulation Data Source 2
			Data Clock Configuration 0
			Transmission Power is 11dBm
			GPIO0 Function is 0x12
			GPIO1 Function is 0x15
			GPIO2 Function is 0x1C

In the first instance, all the register values are read and printed to screen. They are then modified, read and printed again.

rfm22b_receive_test
-------------------
This example receives a short string (less than the maximum packet size of 64 bytes). Running the example will result in a blocking call to the receive function (which times out after 30s by default). If a message is received before the timeout it is printed to the screen. See rfm22b_send_test below for the sender.

rfm22b_send_test
----------------
This example is the companion to the receive example above. It sends the string 'Hello World!'.

Class Details
=============
This section gives some details on the member functions within the class. It is still a work in progress so I recommend looking at the code yourself to make sure you know what each function does.

I have only included the details of the setters, most parameters also have corresponding getters.

Constructor
-----------
To get a new instance of RFM22B, call the constructor with the path to the SPI device, e.g.

	RFM22B *myRadio = new RFM22B("/dev/spidev1.0");
	
SPI Bus Speed (setMaxSpeedHz)
-----------------------------
Inherited from the SPI class, sets the SPI clock frequency to be used, e.g.

	myRadio->setMaxSpeedHz(1000000);
	
Carrier Frequency (setCarrierFrequency)
---------------------------------------
To configure the carrier frequency registers. Pass the desired frequency in integer Hertz, e.g.

	myRadio->setCarrierFrequency(868E6);
	
Frequency Hopping (setFrequencyHoppingStepSize and setChannel)
--------------------------------------------------------------
To configure frequency hopping you must specify a step size and a channel. The step size should be set in Hertz, although it is floored to the nearest 10kHz as this is the resolution of the device. A simple DSSS algorithm could be (with no synchronisation, so it won't actually work!)

	myRadio->setFrequencyHoppingStepSize(20E3);
	uint8_t channel = 0;
	while (true) {
		myRadio->setChannel(channel++);
	}
	
Frequency Deviation (setFrequencyDeviation)
-------------------------------------------
Sets the frequency deviation to be used in all FM schemes. The deviation is in Hertz but floored to the nearest 625Hz. Valid values are from 625-320000

	myRadio->setFrequencyDeviation(6.25E3);
	
Data Rate (setDataRate)
-----------------------
Sets the datarate in bps. Valid values are from 123-256000.

	myRadio->setDataRate(100000);
	
Modulation Type (setModulationType)
-----------------------------------
The modulation type is set by passing an enum value to the function. Possible values are UNMODULATED_CARRIER, OOK, FSK, GFSK (see Section 4.1 of the datasheet for details). E.g.

	myRadio->setModulationType(RFM22B::GFSK);
	
Modulation Data Source (setModulationDataSource)
------------------------------------------------
The data source is set by passing an enum value. Valid values are DIRECT_GPIO, DIRECT_SDI, FIFO, PN9 (see Section 4.2 of the datasheet for details). E.g.

	myRadio->setModulationDataSource(RFM22B::FIFO);
	
Transmission Power (setTransmissionPower)
-----------------------------------------
Output power in dBm (1-20)

	myRadio->setTransmissionPower(5);
	
GPIO Function (setGPIOFunction)
-------------------------------
Specify the function of the GPIOS. enums are used to specify which GPIO to configure and its configuration. Refer to rfm22b_enums.h or the (register description)[http://www.hoperf.com/upload/rf/AN440.pdf] for all the options. The most common use for the GPIOs is to switch the antenna between receive and transmit modes.

	myRadio->setGPIOFunction(RFM22B::GPIO0, RFM22B::TX_STATE);
	myRadio->setGPIOFunction(RFM22B::GPIO1, RFM22B::RX_STATE);
	
Interrupts (setInterruptEnable and getInterruptStatus)
------------------------------------------------------
Enable an interrupt and read its status. Refer to rfm22b_enums.h or the (register description)[http://www.hoperf.com/upload/rf/AN440.pdf] for all the options. The most common interrupt is VALID_PACKET_RECEIVED. This is used internally in the 'receive' function (see below)

	myRadio->setInterruptEnable(VALID_PACKET_RECEIVED, true);
	while (!myRadio->getInterruptStatus(VALID_PACKET_RECEIVED)) {
		// Loop forever until packet is received
	}
	// Handle packet receipt

Receiver and Transmit modes (enableRXMode and enablTXMode)
-----------------------------------------------------------
Switch the device into receiver or transmit mode.

	// On transmitter
	myRadio->enableTXMode();
	
	// On receiver
	myRadio->enableRXMode();

Reset (reset)
-------------
Performs a soft reset which defaults all the registers.

	myRadio->reset();

Packet Headers (setTransmitHeader and setCheckHeader)
-----------------------------------------------------
Set the header (32bit int) for transmitted packets, or the header to check incoming packets against

	// On transmitter
	myRadio->setTransmitHeader(123456789);
	
	// On receiver
	myRadio->setCheckHeader(123456789);
	
RSSI (getRSSI)
--------------
Get the current Receive Strength Signal Indicator (RSSI) value.

	printf("RSSI: %d\n", myRadio->getRSSI());
	
Received Packet Length (getReceivedPacketLength)
------------------------------------------------
Gets the number of bytes received

	printf("%d bytes received\n", myRadio->getReceivedPacketLength());
	
Transmit Packet Length (setTransmitPacketLength)
------------------------------------------------
Set the length of the packet to be transmitter. Must be set before entering transmit mode otherwise nothing will be sent.

	myRadio->setTransmitPacketLength(16);
	
Send Packet (send)
------------------
Sends a byte array. If you attempt to send more than 64 bytes (FIFO maximum), the data is truncated and only the first 64 bytes get sent.

	uint8_t data[] = { 4, 3, 2, 1 };
	myRadio->send(data, 4);
	
Receive Packet (receive)
------------------------
Receive a byte array. Takes an optional timeout parameter which is the number of seconds to block for, if not specified it defaults to 30s.

	uint8_t data[16];
	myRadio->receive(data,16,10);