/*
 * sensor.cpp
 *
 *  Created on: Oct 20, 2016
 *      Author: arthurnguyen
 */

#include "sensor.hpp"

#define RED 0x80
#define YELLOW 0xC0
#define GREEN 0x40

extern bool Sensorcomplete = false;

int frontStart = 0;
int frontStop = 0;
int backStart = 0;
int backStop = 0;
int leftStart = 0;
int leftStop = 0;
int rightStart = 0;
int rightStop = 0;

int frontDistance = 0;
int backDistance = 0;
int leftDistance = 0;
int rightDistance = 0;

void initializeCAN()
{
	CAN_init(can1, 100, 4, 4, NULL, NULL);
	CAN_reset_bus(can1);
	CAN_bypass_filter_accept_all_msgs();
}

bool checkCANbus()
{
	//check CAN bus
	if(CAN_is_bus_off(can1))
	{
		CAN_reset_bus(can1);
		LE.on(4);
		return true;
	}
	return false;
}

void updateCANsonar(SENSOR_SONARS_t *CAN_sensor)
{
	SENSOR_SONARS_t sensor_data;
	sensor_data.SENSOR_SONARS_LEFT_UNSIGNED = leftDistance;
	sensor_data.SENSOR_SONARS_RIGHT_UNSIGNED = rightDistance;
	sensor_data.SENSOR_SONARS_FRONT_UNSIGNED = frontDistance;
	sensor_data.SENSOR_SONARS_BACK_UNSIGNED = backDistance;
	*CAN_sensor = sensor_data;
}


//bool initializeRX_1()
//{
//    static GPIO *Sensor_Input = new GPIO(P0_30);
//    Sensor_Input->setAsOutput();
//    Sensor_Input->setHigh();
//    vTaskDelay(250);
//    Sensor_Input->setLow();
//    delete Sensor_Input;
//    return true;
//}
//
//bool initializeRX_2()
//{
//	static GPIO *Sensor_Input = new GPIO(P0_29);
//	Sensor_Input->setAsOutput();
//	Sensor_Input->setHigh();
//	vTaskDelay(250);
//	Sensor_Input->setLow();
//	delete Sensor_Input;
//	return true;
//}

void frontstartTimer(void)
{
    frontStart = (int)sys_get_uptime_us();
}

void frontstopTimer(void)
{
   frontStop = (int)sys_get_uptime_us() - frontStart;
   frontDistance = frontStop/147;
}

void backstartTimer(void)
{
    backStart = (int)sys_get_uptime_us();
}
void backstopTimer(void)
{
    backStop = (int)sys_get_uptime_us() - backStart;
    backDistance = backStop/147;
}

void leftstartTimer(void)
{
    leftStart = (int)sys_get_uptime_us();
}
void leftstopTimer(void)
{
    leftStop = (int)sys_get_uptime_us() - leftStart;
    leftDistance = leftStop/147;
}

void rightstartTimer(void)
{
    rightStart = (int)sys_get_uptime_us();
}
void rightstopTimer(void)
{
    rightStop = (int)sys_get_uptime_us() - rightStart;
    rightDistance = rightStop/147;
}


void enableHeadlights()
{
	static GPIO *headlights = new GPIO(P1_22);
		headlights->setAsOutput();
		if(LS.getPercentValue() <= 10)
		{
			headlights->setHigh();
		}
		else
		{
			headlights->setLow();
		}
}

