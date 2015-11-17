#include <stdio.h>
#include <string.h>
#include "dw1000.h"

DW1000  *dw;   // Device driver instanceSPI pins: (MOSI, MISO, SCLK, CS, IRQ)

int main(int argc, char **args) {
	dw = new DW1000 ();
	
	printf("Device information: \n");
	printf(" - DEVICE_ID: \t 0x%X\r\n", dw->getDeviceID());
	dw->setEUI(0x0102030405060708);
	printf(" - EUI: \t %016llX\r\n", dw->getEUI());
	printf(" - Voltage: \t %fV\r\n", dw->getVoltage());

	if (dw->getDeviceID() != 0xDECA0130) {
		printf("Quitting due to invalid device id.\n");
		return 0;
	}
	
	return 0;
}