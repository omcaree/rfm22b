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
	
	// Split up MSB and LSB
	uint8_t txdr1 = (txdr >> 8);
	uint8_t txdr0 = txdr & 0xff;
	
	// Set the data rate bytes
	// TODO: could do this in one if we had writeArray
	this->setRegister(TX_DATA_RATE_1, txdr1);
	this->setRegister(TX_DATA_RATE_0, txdr0);
	
	// Set the scaling byte
	this->setRegister(MODULATION_MODE_CONTROL_1, mmc1);
}
unsigned int RFM22B::getDataRate() {
	// Get the data rate scaling value (5th bit of 0x70)
	uint8_t txdtrtscale = (this->getRegister(MODULATION_MODE_CONTROL_1) >> 5) & 1;

	// Get the data rate registers
	// TODO: could do this in one if we had readArray
	uint8_t txdr1 = this->getRegister(TX_DATA_RATE_1);
	uint8_t txdr0 = this->getRegister(TX_DATA_RATE_0);
	
	// Combine bytes
	uint16_t txdr = (txdr1 << 8) | txdr0;
	
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

bool RFM22B::getInterruptStatus(RFM22B_Interrupt interrupt) {
	// Get the (16 bit) register value
	uint8_t intStatus = this->getRegister(INTERRUPT_STATUS_1);
	
	// Determine if interrupt bit is set and return
	if ((intStatus & interrupt) > 0) {
		return true;
	} else {
		return false;
	}
}

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
	
	// tx[1] is the value to be set
	tx[1] = (value >> 8);
	tx[2] = (value) & 0xFF;
	
	this->transfer(tx,rx,3);
}