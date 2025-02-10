#ifndef VL53L1X_H_
#define VL53L1X_H_

#include "i2c.h"
#include "gpio.h"

typedef struct vl53l1x
{
	// I2C information
	I2C_HandleTypeDef* hi2c;
	uint8_t address;

	// XSHUT pin
	GPIO_TypeDef* xshut_port;
	uint16_t xshut_pin;

	// Used for driver back-end, not directly related to VL53L1X
	uint8_t id;
} VL53L1X;


int TOF_InitStruct(VL53L1X* sensor, I2C_HandleTypeDef* hi2c, uint8_t address, GPIO_TypeDef* xshut_port, uint16_t xshut_pin);

int TOF_BootSensor(VL53L1X* sensor);
int TOF_BootMultipleSensors(VL53L1X** sensors, uint8_t count);

void TOF_TurnOn(const VL53L1X* sensor);
void TOF_TurnOff(const VL53L1X* sensor);

uint16_t TOF_GetDistance(const VL53L1X* sensor);
void TOF_SetLogFunction(int (*prnt)(const char*, ...));

#endif // VL53L1X_H_
