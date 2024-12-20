#include"I2C.h"
void initI2C(void)
{
	//This function is for eewiki and is to be updated to handle any port
	//enable GPIO peripheral that contains I2C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	//enable I2C module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);

	//reset I2C module
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);

	// Select the I2C function for these pins.
	GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

	// Configure the pin muxing for I2C0 functions on port B2 and B3.
	GPIOPinConfigure(GPIO_PD0_I2C3SCL);
	GPIOPinConfigure(GPIO_PD1_I2C3SDA);


	// Enable and initialize the I2C0 master module.  Use the system clock for
	// the I2C0 module.  The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will
	// be set to 400kbps.
	I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), true);

	//clear I2C FIFOs
	HWREG(I2C3_BASE + I2C_O_FIFOCTL) = 80008000;
}

uint8_t readI2C(uint16_t device_address, uint16_t device_register)
{
	//specify that we want to communicate to device address with an intended write to bus
	I2CMasterSlaveAddrSet(I2C3_BASE, device_address, false);

	//the register to be read
	I2CMasterDataPut(I2C3_BASE, device_register);

	//send control byte and register address byte to slave device
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);

	//wait for MCU to complete send transaction
	while(I2CMasterBusy(I2C3_BASE));

	//read from the specified slave device
	I2CMasterSlaveAddrSet(I2C3_BASE, device_address, true);

	//send control byte and read from the register from the MCU
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

	//wait while checking for MCU to complete the transaction
	while(I2CMasterBusy(I2C3_BASE));

	//Get the data from the MCU register and return to caller
	return( I2CMasterDataGet(I2C3_BASE));
}

void writeI2C(uint16_t device_address, uint16_t device_register, uint8_t device_data)
{
	//specify that we want to communicate to device address with an intended write to bus
	I2CMasterSlaveAddrSet(I2C3_BASE, device_address, false);

	//register to be read
	I2CMasterDataPut(I2C3_BASE, device_register);

	//send control byte and register address byte to slave device
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	//wait for MCU to finish transaction
	while(I2CMasterBusy(I2C3_BASE));

	I2CMasterSlaveAddrSet(I2C3_BASE, device_address, false);

	//specify data to be written to the above mentioned device_register
	I2CMasterDataPut(I2C3_BASE, device_data);

	//wait while checking for MCU to complete the transaction
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

	//wait for MCU & device to complete transaction
	while(I2CMasterBusy(I2C3_BASE));
}
