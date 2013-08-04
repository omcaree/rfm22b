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
	this->setRegister(FREQUENCY_BAND_SELECT, fbs);
	this->setRegister(NOMINAL_CARRIER_FREQUENCY_1, ncf1);
	this->setRegister(NOMINAL_CARRIER_FREQUENCY_0, ncf0);
}

// Get the frequency of the carrier wave in integer Hertz
//	Without any frequency hopping
unsigned int RFM22B::getCarrierFrequency() {
	// Read the register values
	uint8_t fbs = this->getRegister(FREQUENCY_BAND_SELECT);
	uint8_t ncf1 = this->getRegister(NOMINAL_CARRIER_FREQUENCY_1);
	uint8_t ncf0 = this->getRegister(NOMINAL_CARRIER_FREQUENCY_0);

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

// Get and set the frequency hopping step size
//	Values are in Hertz (to stay SI) but floored to the nearest 10kHz
void RFM22B::setFrequencyHoppingStepSize(unsigned int step) {
	if (step > 2550000) {
		step = 2550000;
	}
	this->setRegister(FREQUENCY_HOPPING_STEP_SIZE, step/10000);
}
unsigned int RFM22B::getFrequencyHoppingStepSize() {
	return this->getRegister(FREQUENCY_HOPPING_STEP_SIZE)*10000;
}

// Get and set the frequency hopping channel
void RFM22B::setChannel(uint8_t channel) {
	this->setRegister(FREQUENCY_HOPPING_CHANNEL_SELECT, channel);
}
uint8_t RFM22B::getChannel() {
	return this->getRegister(FREQUENCY_HOPPING_CHANNEL_SELECT);
}

// Set or get the frequency deviation (in Hz, but floored to the nearest 625Hz)
void RFM22B::setFrequencyDeviation(unsigned int deviation) {
	if (deviation > 320000) {
		deviation = 320000;
	}
	this->setRegister(FREQUENCY_DEVIATION, deviation/625);
}
unsigned int RFM22B::getFrequencyDeviation() {
	return this->getRegister(FREQUENCY_DEVIATION)*625;
}

// Set or get the data rate (bps)
void RFM22B::setDataRate(unsigned int rate) {
	// Get the Modulation Mode Control 1 register (for scaling bit)
	uint8_t mmc1 = this->getRegister(MODULATION_MODE_CONTROL_1);
	
	uint16_t txdr;
	// Set the scale bit (5th bit of 0x70) high if data rate is below 30kbps
	// and calculate the value for txdr registers (0x6E and 0x6F)
	if (rate < 30000) {
		mmc1 |= (1<<5);
		txdr = rate * ((1 << (16 + 5)) / 1E6);
	} else {
		mmc1 &= ~(1<<5);
		txdr = rate * ((1 << (16)) / 1E6);
	}
	
	// Set the data rate bytes
	this->set16BitRegister(TX_DATA_RATE_1, txdr);
	
	// Set the scaling byte
	this->setRegister(MODULATION_MODE_CONTROL_1, mmc1);
}
unsigned int RFM22B::getDataRate() {
	// Get the data rate scaling value (5th bit of 0x70)
	uint8_t txdtrtscale = (this->getRegister(MODULATION_MODE_CONTROL_1) >> 5) & 1;

	// Get the data rate registers
	uint8_t txdr = this->get16BitRegister(TX_DATA_RATE_1);
	
	// Return the data rate (in bps, hence extra 1E3)
	return (txdr * 1E6) / (1 << (16 + 5 * txdtrtscale));
	
}

// Set or get the modulation type
void RFM22B::setModulationType(RFM22B_Modulation_Type modulation) {
	// Get the Modulation Mode Control 2 register
	uint8_t mmc2 = this->getRegister(MODULATION_MODE_CONTROL_2);
	
	// Clear the modtyp bits
	mmc2 &= ~0x03;
	
	// Set the desired modulation
	mmc2 |= modulation;

	// Set the register
	this->setRegister(MODULATION_MODE_CONTROL_2, mmc2);
}
RFM22B::RFM22B_Modulation_Type RFM22B::getModulationType() {
	// Get the Modulation Mode Control 2 register
	uint8_t mmc2 = this->getRegister(MODULATION_MODE_CONTROL_2);
	
	// Determine modtyp bits
	uint8_t modtyp = mmc2 & 0x03;
	
	// Ugly way to return correct enum
	switch (modtyp) {
		case 1:
			return OOK;
		case 2:
			return FSK;
		case 3:
			return GFSK;
		case 0:
		default:
			return UNMODULATED_CARRIER;
	}
}

void RFM22B::setModulationDataSource(RFM22B_Modulation_Data_Source source) {
	// Get the Modulation Mode Control 2 register
	uint8_t mmc2 = this->getRegister(MODULATION_MODE_CONTROL_2);
	
	// Clear the dtmod bits
	mmc2 &= ~(0x03<<4);
	
	// Set the desired data source
	mmc2 |= source << 4;

	// Set the register
	this->setRegister(MODULATION_MODE_CONTROL_2, mmc2);
}
RFM22B::RFM22B_Modulation_Data_Source RFM22B::getModulationDataSource() {
	// Get the Modulation Mode Control 2 register
	uint8_t mmc2 = this->getRegister(MODULATION_MODE_CONTROL_2);
	
	// Determine modtyp bits
	uint8_t dtmod = (mmc2 >> 4) & 0x03;
	
	// Ugly way to return correct enum
	switch (dtmod) {
		case 1:
			return DIRECT_SDI;
		case 2:
			return FIFO;
		case 3:
			return PN9;
		case 0:
		default:
			return DIRECT_GPIO;
	}
}

void RFM22B::setDataClockConfiguration(RFM22B_Data_Clock_Configuration clock) {
	// Get the Modulation Mode Control 2 register
	uint8_t mmc2 = this->getRegister(MODULATION_MODE_CONTROL_2);
	
	// Clear the trclk bits
	mmc2 &= ~(0x03<<6);
	
	// Set the desired data source
	mmc2 |= clock << 6;

	// Set the register
	this->setRegister(MODULATION_MODE_CONTROL_2, mmc2);
}
RFM22B::RFM22B_Data_Clock_Configuration RFM22B::getDataClockConfiguration() {
	// Get the Modulation Mode Control 2 register
	uint8_t mmc2 = this->getRegister(MODULATION_MODE_CONTROL_2);
	
	// Determine modtyp bits
	uint8_t dtmod = (mmc2 >> 6) & 0x03;
	
	// Ugly way to return correct enum
	switch (dtmod) {
		case 1:
			return GPIO;
		case 2:
			return SDO;
		case 3:
			return NIRQ;
		case 0:
		default:
			return NONE;
	}
}

// Set or get the transmission power
void RFM22B::setTransmissionPower(uint8_t power) {
	// Saturate to maximum power
	if (power > 20) {
		power = 20;
	}

	// Get the TX power register
	uint8_t txp = this->getRegister(TX_POWER);
	
	// Clear txpow bits
	txp &= ~(0x07);
	
	// Calculate txpow bits (See Section 5.7.1 of datasheet)
	uint8_t txpow = (power + 1) / 3;
	
	// Set txpow bits
	txp |= txpow;
	
	// Set the register
	this->setRegister(TX_POWER, txp);
}
uint8_t RFM22B::getTransmissionPower() {
	// Get the TX power register
	uint8_t txp = this->getRegister(TX_POWER);
	
	// Get the txpow bits
	uint8_t txpow = txp & 0x07;
	
	// Calculate power (see Section 5.7.1 of datasheet)
	if (txpow == 0) {
		return 1;
	} else {
		return txpow * 3 - 1;
	}
}

// Set or get the GPIO configuration
void RFM22B::setGPIOFunction(RFM22B_GPIO gpio, RFM22B_GPIO_Function func) {
	// Get the GPIO register
	uint8_t gpioX = this->getRegister(gpio);
	
	// Clear gpioX bits
	gpioX &= ~((1<<5)-1);
	
	// Set the gpioX bits
	gpioX |= func;
	
	// Set the register
	this->setRegister(gpio, gpioX);
}

uint8_t RFM22B::getGPIOFunction(RFM22B_GPIO gpio) {
	// Get the GPIO register
	uint8_t gpioX = this->getRegister(gpio);

	// Return the gpioX bits
	// This should probably return an enum, but this needs a lot of cases
	return gpioX & ((1<<5)-1);
}

// Enable or disable interrupts
void RFM22B::setInterruptEnable(RFM22B_Interrupt interrupt, bool enable) {
	// Get the (16 bit) register value
	uint16_t intEnable = this->get16BitRegister(INTERRUPT_ENABLE_1);
	
	// Either enable or disable the interrupt
	if (enable) {
		intEnable |= interrupt;
	} else {
		intEnable &= ~interrupt;
	}
	
	// Set the (16 bit) register value
	this->set16BitRegister(INTERRUPT_ENABLE_1, intEnable);
}

// Get the status of an interrupt
bool RFM22B::getInterruptStatus(RFM22B_Interrupt interrupt) {
	// Get the (16 bit) register value
	uint16_t intStatus = this->get16BitRegister(INTERRUPT_STATUS_1);
	
	// Determine if interrupt bit is set and return
	if ((intStatus & interrupt) > 0) {
		return true;
	} else {
		return false;
	}
}

// Set the operating mode
//	This function does not toggle individual pins as with other functions
//	It expects a bitwise-ORed combination of the modes you want set
void RFM22B::setOperatingMode(uint16_t mode) {
	this->set16BitRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_1, mode);
}

// Get operating mode (bitwise-ORed)
uint16_t RFM22B::getOperatingMode() {
	return this->get16BitRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_1);
}

// Manuall enter RX or TX mode
void RFM22B::enableRXMode() {
	this->setOperatingMode(READY_MODE | RX_MODE);
}
void RFM22B::enableTXMode() {
	this->setOperatingMode(READY_MODE | TX_MODE);
}

// Reset the device
void RFM22B::reset() {
	this->setOperatingMode(READY_MODE | RESET);
}

// Set or get the trasmit header
void RFM22B::setTransmitHeader(uint32_t header) {
	this->set32BitRegister(TRANSMIT_HEADER_3, header);
}
uint32_t RFM22B::getTransmitHeader() {
	return this->get32BitRegister(TRANSMIT_HEADER_3);
}

// Set or get the check header
void RFM22B::setCheckHeader(uint32_t header) {
	this->set32BitRegister(CHECK_HEADER_3, header);
}
uint32_t RFM22B::getCheckHeader() {
	return this->get32BitRegister(CHECK_HEADER_3);
}

// Get and set all the FIFO threshold
void RFM22B::setTXFIFOAlmostFullThreshold(uint8_t thresh) {
	this->setFIFOThreshold(TX_FIFO_CONTROL_1, thresh);
}
void RFM22B::setTXFIFOAlmostEmptyThreshold(uint8_t thresh) {
	this->setFIFOThreshold(TX_FIFO_CONTROL_2, thresh);
}
void RFM22B::setRXFIFOAlmostFullThreshold(uint8_t thresh) {
	this->setFIFOThreshold(RX_FIFO_CONTROL, thresh);
}
uint8_t RFM22B::getTXFIFOAlmostFullThreshold() {
	return this->getRegister(TX_FIFO_CONTROL_1);
}
uint8_t RFM22B::getTXFIFOAlmostEmptyThreshold() {
	return this->getRegister(TX_FIFO_CONTROL_2);
}
uint8_t RFM22B::getRXFIFOAlmostFullThreshold() {
	return this->getRegister(RX_FIFO_CONTROL);
}
void RFM22B::setFIFOThreshold(RFM22B_Register reg, uint8_t thresh) {
	thresh &= ((1 << 6) - 1);
	this->setRegister(reg, thresh);
}

// Get RSSI value
uint8_t RFM22B::getRSSI() {
	return this->getRegister(RECEIVED_SIGNAL_STRENGTH_INDICATOR);
}
// Get input power (in dBm)
//	Coefficients approximated from the graph in Section 8.10 of the datasheet
int8_t RFM22B::getInputPower() {
	return 0.56*this->getRSSI()-128.8;
}

// Get length of last received packet
uint8_t RFM22B::getReceivedPacketLength() {
	return this->getRegister(RECEIVED_PACKET_LENGTH);
}

// Set length of packet to be transmitted
void RFM22B::setTransmitPacketLength(uint8_t length) {
	return this->setRegister(TRANSMIT_PACKET_LENGTH, length);
}

void RFM22B::clearRXFIFO() {
	//Toggle ffclrrx bit high and low to clear RX FIFO
	this->setRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_2, 2);
	this->setRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_2, 0);
}

void RFM22B::clearTXFIFO() {
	//Toggle ffclrtx bit high and low to clear TX FIFO
	this->setRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_2, 1);
	this->setRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_2, 0);
}

// Send data
void RFM22B::send(uint8_t *data, int length) {
	// Clear TX FIFO
	this->clearTXFIFO();
	
	// Initialise rx and tx arrays
	uint8_t tx[MAX_PACKET_LENGTH+1] = { 0 };
	uint8_t rx[MAX_PACKET_LENGTH+1] = { 0 };
	
	// Set FIFO register address (with write flag)
	tx[0] = FIFO_ACCESS | (1<<7);
	
	// Truncate data if its too long
	if (length > MAX_PACKET_LENGTH) {
		length = MAX_PACKET_LENGTH;
	}
	
	// Copy data from input array to tx array
	for (int i = 1; i <= length; i++) {
		tx[i] = data[i-1];
	}
	
	// Set the packet length
	this->setTransmitPacketLength(length);
	
	// Make the transfer
	this->transfer(tx,rx,length+1);
	
	// Enter TX mode
	this->enableTXMode();

	// Loop until packet has been sent (device has left TX mode)
	while (((this->getRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_1)>>3) & 1)) {}
	
	return;
};

// Receive data (blocking with timeout). Returns number of bytes received
int RFM22B::receive(uint8_t *data, int length, int timeout) {
	// Make sure RX FIFO is empty, ready for new data
	this->clearRXFIFO();
	
	// Enter RX mode
	this->enableRXMode();
		
	// Initialise rx and tx arrays
	uint8_t tx[MAX_PACKET_LENGTH+1] = { 0 };
	uint8_t rx[MAX_PACKET_LENGTH+1] = { 0 };
	
	// Set FIFO register address
	tx[0] = FIFO_ACCESS;
	
	// Timing for the interrupt loop timeout
	struct timeval start, end;
    gettimeofday(&start, NULL);
	long elapsed = 0;
	
	// Loop endlessly on interrupt or timeout
	//	Don't use interrupt registers here as these don't seem to behave consistently
	//	Watch the operating mode register for the device leaving RX mode. This is indicitive
	//	of a valid packet being received
	while (((this->getRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_1)>>2) & 1) && elapsed < timeout) {
		// Determine elapsed time
		gettimeofday(&end, NULL);
		elapsed = (end.tv_usec - start.tv_usec)/1000 + (end.tv_sec - start.tv_sec)*1000;
	}	
	
	// If timeout occured, return -1
	if (elapsed >= timeout) {
		return -1;
	}
	
	// Get length of packet received
	uint8_t rxLength = this->getReceivedPacketLength();
	
	// Make the transfer
	this->transfer(tx,rx,rxLength+1);
	
	// Copy the data to the output array
	for (int i = 1; i <= rxLength; i++) {
		data[i-1] = rx[i];
	}
	
	return rxLength;
};

// Helper function to read a single byte from the device
uint8_t RFM22B::getRegister(uint8_t reg) {
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

// Similar to function above, but for readying 2 consequtive registers as one
uint16_t RFM22B::get16BitRegister(uint8_t reg) {
	uint8_t tx[] = {0x00, 0x00, 0x00};
	uint8_t rx[] = {0x00, 0x00, 0x00};
	
	tx[0] = reg;
	
	this->transfer(tx,rx,3);

	return (rx[1] << 8) | rx[2];
}

// Similar to function above, but for readying 4 consequtive registers as one
uint32_t RFM22B::get32BitRegister(uint8_t reg) {
	uint8_t tx[] = {0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx[] = {0x00, 0x00, 0x00, 0x00, 0x00};
	
	tx[0] = reg;
	
	this->transfer(tx,rx,5);

	return (rx[1] << 24) | (rx[2] << 16) | (rx[3] << 8) | rx[4];
}

// Helper function to write a single byte to a register
void RFM22B::setRegister(uint8_t reg, uint8_t value) {
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

// As above, but for 2 consequitive registers
void RFM22B::set16BitRegister(uint8_t reg, uint16_t value) {
	// tx and rx arrays required even though we aren't receiving anything
	uint8_t tx[] = {0x00, 0x00, 0x00};
	uint8_t rx[] = {0x00, 0x00, 0x00};
	
	// tx[0] is the requested register with the final bit set high to indicate
	// a write operation (see Section 3.1 of the datasheet)
	tx[0] = reg | (1<<7);
	
	// tx[1-2] is the value to be set
	tx[1] = (value >> 8);
	tx[2] = (value) & 0xFF;
	
	this->transfer(tx,rx,3);
}

// As above, but for 4 consequitive registers
void RFM22B::set32BitRegister(uint8_t reg, uint32_t value) {
	// tx and rx arrays required even though we aren't receiving anything
	uint8_t tx[] = {0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx[] = {0x00, 0x00, 0x00, 0x00, 0x00};
	
	// tx[0] is the requested register with the final bit set high to indicate
	// a write operation (see Section 3.1 of the datasheet)
	tx[0] = reg | (1<<7);
	
	// tx[1-4] is the value to be set
	tx[1] = (value >> 24);
	tx[2] = (value >> 16) & 0xFF;
	tx[3] = (value >> 8) & 0xFF;
	tx[4] = (value) & 0xFF;
	
	this->transfer(tx,rx,5);
}