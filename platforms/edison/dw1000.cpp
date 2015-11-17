/*
* Authors: Kiveisha Yevgeniy and Tschirschnitz Maximilian
* Based on Matthias Grob & Manuel Stalder - ETH Zürich - 2015
*
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject to
* the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
* LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
* OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
* WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "dw1000.h"

DW1000::DW1000()
{
	spi = mraa_spi_init(5);
	mraa_spi_frequency(spi, DW1000_SPI_CLOCK_SPEED);

	cs = mraa_gpio_init(DW_CS);
	mraa_gpio_dir(cs, MRAA_GPIO_OUT);
	mraa_gpio_write(cs, 0x1);

	resetAll();                         // we do a soft reset of the DW1000 everytime the driver starts

	//Those values are for the 110kbps mode (5, 16MHz, 1024 Symbols) and are quite complete
	writeRegister16(DW1000_AGC_CTRL, 0x04, 0x8870);             //AGC_TUNE1 for 16MHz PRF
	writeRegister32(DW1000_AGC_CTRL, 0x0C, 0x2502A907);         //AGC_TUNE2 (Universal)
	writeRegister16(DW1000_AGC_CTRL, 0x12, 0x0055);             //AGC_TUNE3 (Universal)

	writeRegister16(DW1000_DRX_CONF, 0x02, 0x000A);             //DRX_TUNE0b for 110kbps
	writeRegister16(DW1000_DRX_CONF, 0x04, 0x0087);             //DRX_TUNE1a for 16MHz PRF
	writeRegister16(DW1000_DRX_CONF, 0x06, 0x0064);             //DRX_TUNE1b for 110kbps & > 1024 symbols
	writeRegister32(DW1000_DRX_CONF, 0x08, 0x351A009A);         //PAC size for 1024 symbols preamble & 16MHz PRF
	//writeRegister32(DW1000_DRX_CONF, 0x08, 0x371A011D);               //PAC size for 2048 symbols preamble

	writeRegister8 (DW1000_LDE_CTRL, 0x0806, 0xD);              //LDE_CFG1
	writeRegister16(DW1000_LDE_CTRL, 0x1806, 0x1607);           //LDE_CFG2 for 16MHz PRF

	writeRegister32(DW1000_TX_POWER, 0, 0x28282828);            //Power for channel 5

	writeRegister8(DW1000_RF_CONF, 0x0B, 0xD8);                 //RF_RXCTRLH for channel 5
	writeRegister32(DW1000_RF_CONF, 0x0C, 0x001E3FE0);          //RF_TXCTRL for channel 5

	writeRegister8 (DW1000_TX_CAL, 0x0B, 0xC0);                 //TC_PGDELAY for channel 5

	writeRegister32 (DW1000_FS_CTRL, 0x07, 0x0800041D);         //FS_PLLCFG for channel 5
	writeRegister8 (DW1000_FS_CTRL, 0x0B, 0xA6);                //FS_PLLTUNE for channel 5

	loadLDE();                          // important everytime DW1000 initialises/awakes otherwise the LDE algorithm must be turned off or there's receiving malfunction see User Manual LDELOAD on p22 & p158

	// 110kbps CAUTION: a lot of other registers have to be set for an optimized operation on 110kbps
	writeRegister16(DW1000_TX_FCTRL, 1, 0x0800 | 0x0100 | 0x0080); // use 1024 symbols preamble (0x0800) (previously 2048 - 0x2800), 16MHz pulse repetition frequency (0x0100), 110kbps bit rate (0x0080) see p.69 of DW1000 User Manual
	writeRegister8(DW1000_SYS_CFG, 2, 0x44);    // enable special receiving option for 110kbps (disable smartTxPower)!! (0x44) see p.64 of DW1000 User Manual [DO NOT enable 1024 byte frames (0x03) becuase it generates disturbance of ranging don't know why...]

	writeRegister16(DW1000_TX_ANTD, 0, 16384); // set TX and RX Antenna delay to neutral because we calibrate afterwards
	writeRegister16(DW1000_LDE_CTRL, 0x1804, 16384); // = 2^14 a quarter of the range of the 16-Bit register which corresponds to zero calibration in a round trip (TX1+RX2+TX2+RX1)

	writeRegister8(DW1000_SYS_CFG, 3, 0x20);    // enable auto reenabling receiver after error
}

uint32_t DW1000::getDeviceID()
{
	uint32_t result;
	/* GetDeviceID! */
	readRegister(DW1000_DEV_ID, 0, (uint8_t*)&result, 4);
	return result;
}

uint64_t DW1000::getEUI()
{
	uint64_t result;
	readRegister(DW1000_EUI, 0, (uint8_t*)&result, 8);
	return result;
}

void DW1000::setEUI(uint64_t EUI)
{
	writeRegister(DW1000_EUI, 0, (uint8_t*)&EUI, 8);
}

float DW1000::getVoltage()
{
	uint8_t buffer[7] = {0x80, 0x0A, 0x0F, 0x01, 0x00};             // algorithm form User Manual p57
	writeRegister(DW1000_RF_CONF, 0x11, buffer, 2);
	writeRegister(DW1000_RF_CONF, 0x12, &buffer[2], 1);
	writeRegister(DW1000_TX_CAL, 0x00, &buffer[3], 1);
	writeRegister(DW1000_TX_CAL, 0x00, &buffer[4], 1);
	readRegister(DW1000_TX_CAL, 0x03, &buffer[5], 2);               // get the 8-Bit readings for Voltage and Temperature
	float Voltage = buffer[5] * 0.0057 + 2.3;
	
	return Voltage;
}

uint64_t DW1000::getStatus()
{
	return readRegister40(DW1000_SYS_STATUS, 0);
}

uint64_t DW1000::getRXTimestamp()
{
	uint64_t temp = readRegister40(DW1000_RX_TIME, 0);
	return temp;
}

uint64_t DW1000::getTXTimestamp()
{
	return readRegister40(DW1000_TX_TIME, 0);
}

void DW1000::sendString(char* message)
{
	sendFrame((uint8_t*)message, strlen(message)+1);
}

void DW1000::receiveString(char* message)
{
	readRegister(DW1000_RX_BUFFER, 0, (uint8_t*)message, getFramelength());  // get data from buffer
}

void DW1000::sendFrame(uint8_t* message, uint16_t length) {
	if (length >= 125) length = 125;                                // check for maximim length a frame can have with 127 Byte frames
	writeRegister(DW1000_TX_BUFFER, 0, message, length);            // fill buffer

	uint8_t backup = readRegister8(DW1000_TX_FCTRL, 1);             // put length of frame
	length += 2;                                                    // including 2 CRC Bytes
	length = ((backup & 0xFC) << 8) | (length & 0x03FF);
	writeRegister16(DW1000_TX_FCTRL, 0, length);

	stopTRX();                                                      // stop receiving
	writeRegister8(DW1000_SYS_CTRL, 0, 0x02);                       // trigger sending process by setting the TXSTRT bit
	startRX();                                                      // enable receiver again
}

void DW1000::sendDelayedFrame(uint8_t* message, uint16_t length, uint64_t TxTimestamp) {
	if (length >= 125) length = 125;                                // check for maximim length a frame can have with 127 Byte frames
	writeRegister(DW1000_TX_BUFFER, 0, message, length);            // fill buffer

	uint8_t backup = readRegister8(DW1000_TX_FCTRL, 1);             // put length of frame
	length += 2;                                                    // including 2 CRC Bytes
	length = ((backup & 0xFC) << 8) | (length & 0x03FF);
	writeRegister16(DW1000_TX_FCTRL, 0, length);

	writeRegister40(DW1000_DX_TIME, 0, TxTimestamp);                //write the timestamp on which to send the message

	stopTRX();                                                      // stop receiving
	writeRegister8(DW1000_SYS_CTRL, 0, 0x02 | 0x04);                // trigger sending process by setting the TXSTRT and TXDLYS bit
	startRX();                                                      // enable receiver again
}

void DW1000::startRX() {
	writeRegister8(DW1000_SYS_CTRL, 0x01, 0x01);                    // start listening for preamble by setting the RXENAB bit
}

void DW1000::stopTRX() {
	writeRegister8(DW1000_SYS_CTRL, 0, 0x40);                       // disable tranceiver go back to idle mode
}

void DW1000::loadLDE() {                                            // initialise LDE algorithm LDELOAD User Manual p22
	writeRegister16(DW1000_PMSC, 0, 0x0301);                        // set clock to XTAL so OTP is reliable
	writeRegister16(DW1000_OTP_IF, 0x06, 0x8000);                   // set LDELOAD bit in OTP
	usleep(150);
	writeRegister16(DW1000_PMSC, 0, 0x0200);                        // recover to PLL clock
}

void DW1000::resetRX()
{
	writeRegister8(DW1000_PMSC, 3, 0xE0);   // set RX reset
	writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear RX reset
}

void DW1000::resetAll()
{
	writeRegister8(DW1000_PMSC, 0, 0x01);   // set clock to XTAL
	writeRegister8(DW1000_PMSC, 3, 0x00);   // set All reset
	usleep(10);                            // wait for PLL to lock
	writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear All reset
}

void DW1000::setInterrupt(bool RX, bool TX)
{
	writeRegister16(DW1000_SYS_MASK, 0, RX*0x4000 | TX*0x0080);  // RX good frame 0x4000, TX done 0x0080
}

uint16_t DW1000::getFramelength()
{
	uint16_t framelength = readRegister16(DW1000_RX_FINFO, 0);      // get framelength
	framelength = (framelength & 0x03FF) - 2;                       // take only the right bits and subtract the 2 CRC Bytes
	return framelength;
}

uint8_t DW1000::readRegister8(uint8_t reg, uint16_t subaddress)
{
	uint8_t result;
	readRegister(reg, subaddress, &result, 1);
	return result;
}

uint16_t DW1000::readRegister16(uint8_t reg, uint16_t subaddress)
{
	uint16_t result;
	readRegister(reg, subaddress, (uint8_t*)&result, 2);
	return result;
}

uint64_t DW1000::readRegister40(uint8_t reg, uint16_t subaddress)
{
	uint64_t result;
	readRegister(reg, subaddress, (uint8_t*)&result, 5);
	result &= 0xFFFFFFFFFF;                                 // only 40-Bit
	return result;
}

void DW1000::writeRegister8(uint8_t reg, uint16_t subaddress, uint8_t buffer)
{
	writeRegister(reg, subaddress, &buffer, 1);
}

void DW1000::writeRegister16(uint8_t reg, uint16_t subaddress, uint16_t buffer)
{
	writeRegister(reg, subaddress, (uint8_t*)&buffer, 2);
}

void DW1000::writeRegister32(uint8_t reg, uint16_t subaddress, uint32_t buffer)
{
	writeRegister(reg, subaddress, (uint8_t*)&buffer, 4);
}

void DW1000::writeRegister40(uint8_t reg, uint16_t subaddress, uint64_t buffer)
{
	writeRegister(reg, subaddress, (uint8_t*)&buffer, 5);
}

void DW1000::readRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length)
{
	uint8_t *request;
	uint8_t *response;
	uint8_t header[3];
	int headerLen = 1;

	if (subaddress > 0) {
		header[0] = reg | DW1000_SUBADDRESS_FLAG;

		if (subaddress > 0x7F) {
			header[1] = (uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG;
			header[2] = (uint8_t)(subaddress >> 7);
			headerLen+=2;
		} else {
			header[1] = (uint8_t)subaddress;
			headerLen++;
		}
	} else {
		header[0] = reg;
	}

	request =  (uint8_t *) malloc(sizeof(uint8_t) * (length + headerLen));
	response = (uint8_t *) malloc(sizeof(uint8_t) * (length + headerLen));
	memset(request,  0x00, length + headerLen);
	memset(response, 0x00, length + headerLen);
	memcpy(request, header, headerLen);

	mraa_gpio_write(cs, 0x1);
	mraa_gpio_write(cs, 0x0);
	usleep(100);
	mraa_spi_transfer_buf(spi, request, response, length + headerLen);
	usleep(100);
	mraa_gpio_write(cs, 0x1);

	memcpy (buffer, &response[headerLen], length);

	free(request);
	free(response);
}

void DW1000::writeRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length)
{
	uint8_t *packet;
	uint8_t *response;
	uint8_t header[3];
	int headerLen = 1;

	reg |= DW1000_WRITE_FLAG;

	if (subaddress > 0) {
		header[0] = reg | DW1000_SUBADDRESS_FLAG;

		if (subaddress > 0x7F) {
			header[1] = (uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG;
			header[2] = (uint8_t)(subaddress >> 7);
			headerLen+=2;
		} else {
			header[1] = (uint8_t)subaddress;
			headerLen++;
		}
	} else {
		header[0] = reg;
	}

	packet = (uint8_t *) malloc(sizeof(uint8_t) * (length + headerLen));
	response = (uint8_t *) malloc(sizeof(uint8_t) * (length + headerLen));
	memset (packet, 0x00, length + headerLen);
	memcpy (packet, header, headerLen);
	memcpy (&packet[headerLen], buffer, length);

	mraa_gpio_write(cs, 0x1);
	mraa_gpio_write(cs, 0x0);
	usleep (100);
	mraa_spi_transfer_buf(spi, packet, response, length + headerLen);
	usleep (100);
	mraa_gpio_write(cs, 0x1);

	free(packet);
	free(response);
}

void DW1000::setupTransaction(uint8_t reg, uint16_t subaddress, bool write)
{
	reg |= (write * DW1000_WRITE_FLAG); /* Set read/write flag */

	if(subaddress > 0) /* There's a subadress, we need to set flag and send second header byte */
	{
		mraa_spi_write(spi, reg | DW1000_SUBADDRESS_FLAG);

		if(subaddress > 0x7F) /* Sub address too long, we need to set flag and send third header byte */
		{
			mraa_spi_write(spi, (uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG);
			mraa_spi_write(spi, (uint8_t)(subaddress >> 7));
		}
		else
		{
			mraa_spi_write(spi, (uint8_t)subaddress);
		}

	}
	else
	{
		mraa_spi_write(spi, reg);
	}
}