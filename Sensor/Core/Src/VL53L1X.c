#include "VL53L1X.h"
#include "VL53L1X_api.h"

// Helper functions & variables
#define _VL53L1X_LOG(fmt, ...) { if (vl53l1x_log_func != NULL) vl53l1x_log_func("TOF Debug: " fmt, ##__VA_ARGS__); }
#define _VL53L1X_MAX_SENSORS 10

VL53L1X* registered_sensors[_VL53L1X_MAX_SENSORS] = { 0 };
const size_t registered_sensors_cnt = _VL53L1X_MAX_SENSORS;
static int (*vl53l1x_log_func)(const char*, ...) = NULL;

// Function definitions
int TOF_InitStruct(VL53L1X* sensor, I2C_HandleTypeDef* hi2c, uint8_t address, GPIO_TypeDef* xshut_port, uint16_t xshut_pin)
{
	if (address == 0x29)
		_VL53L1X_LOG("Trying to set 0x29 as the address for the ToF, this can cause issues\r\n");

	sensor->hi2c = hi2c;
	sensor->address = address;

	sensor->xshut_port = xshut_port;
	sensor->xshut_pin = xshut_pin;

	// Find free slot
	uint8_t id;
	for (id = 0; (id < _VL53L1X_MAX_SENSORS) && registered_sensors[id] != NULL; id++);

	if (id >= _VL53L1X_MAX_SENSORS)
	{
		_VL53L1X_LOG("Added too many sensors, increase _VL53L1X_MAX_SENSORS value, currently %d\r\n", registered_sensors_cnt);
		return 1;
	}

	sensor->id = id;
	registered_sensors[id] = sensor;
	return 0;
}

void TOF_TurnOn(const VL53L1X* sensor)
{
	HAL_GPIO_WritePin(sensor->xshut_port, sensor->xshut_pin, GPIO_PIN_SET);
}

void TOF_TurnOff(const VL53L1X* sensor)
{
	HAL_GPIO_WritePin(sensor->xshut_port, sensor->xshut_pin, GPIO_PIN_RESET);
}

int TOF_BootMultipleSensors(VL53L1X** sensors, uint8_t count)
{
	int status = 0;

	for (uint8_t n = 0; n < count; n++) {
		TOF_TurnOff(sensors[n]);
	}

	for (uint8_t n = 0; n < count; n++) {
		status |= TOF_BootSensor(sensors[n]);
	}

	return status;
}

int TOF_BootSensor(VL53L1X* sensor)
{
	int status = 0;
	TOF_TurnOn(sensor);
    HAL_Delay(2); // Sometimes this is too fast and line can't settle

    // Check if sensor is already configured with the correct address
    uint16_t id = 0;
    if (VL53L1X_GetSensorId(sensor->id, &id) != 0)
    {
    	uint8_t real_address = sensor->address;
    	sensor->address = 0x29;

    	status = VL53L1X_SetI2CAddress(sensor->id, real_address);

    	sensor->address = real_address;

    	if (status != 0)
    	{
    		if (sensor->address == 0x20) {
    			status = 1;
    			return status;
    		}
    		else if (sensor->address == 0x26) {
    			status = 2;
    			return status;
    		}
    		else {
    			status = 9;
    			return status;
    		}
    		_VL53L1X_LOG("Unable to configure address for sensor 0x%02X\r\n", sensor->address);
    		return status;
    	}
    }

	// Initialise sensor
    status = VL53L1X_SensorInit(sensor->id);
	if (status)
	{
		status = 3;
		_VL53L1X_LOG("Unable to initialise sensor 0x%02X\r\n", sensor->address);
		return status;
	}
	status = VL53L1X_StartRanging(sensor->id);
	if (status)
	{
		status = 4;
		_VL53L1X_LOG("Unable to start measurements on sensor 0x%02X\r\n", sensor->address);
		return status;
	}

	_VL53L1X_LOG("Sensor 0x%02X configured successfully\r\n", sensor->address);
	return 0;
}

uint16_t TOF_GetDistance(const VL53L1X* sensor)
{
	uint16_t reading = 0;
	if(VL53L1X_GetDistance(sensor->id, &reading) != 0)
	{
		_VL53L1X_LOG("Failed to read distance from sensor 0x%02X\r\n", sensor->address);
		return 0xFFFF; // VL53L1X_GetDistance() returns the reading for previous sensor if reading failed
	}
	return reading;
}

void TOF_SetLogFunction(int (*prnt)(const char*, ...))
{
	vl53l1x_log_func = prnt;
}
