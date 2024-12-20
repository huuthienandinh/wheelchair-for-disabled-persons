#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
void initI2C(void);
uint8_t readI2C(uint16_t device_address, uint16_t device_register);
void writeI2C(uint16_t device_address, uint16_t device_register, uint8_t device_data);
