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

bool initializeRX_1()
{
    static GPIO *Sensor_Input = new GPIO(P0_30);
    Sensor_Input->setAsOutput();
    Sensor_Input->setHigh();
    vTaskDelay(250);
    Sensor_Input->setLow();
    delete Sensor_Input;
    return true;
}

bool initializeRX_2()
{
	static GPIO *Sensor_Input = new GPIO(P0_29);
	Sensor_Input->setAsOutput();
	Sensor_Input->setHigh();
	vTaskDelay(250);
	Sensor_Input->setLow();
	delete Sensor_Input;
	return true;
}

void frontstartTimer(void)
{
    frontStart = (int)sys_get_uptime_us();
}

void frontstopTimer(void)
{
   frontStop = (int)sys_get_uptime_us() - frontStart;
   frontDistance = frontStop/147;
   long yield = 0;
   //xSemaphoreGiveFromISR(sensor1, &yield);
   portYIELD_FROM_ISR(&yield);
}

void backstartTimer(void)
{
    backStart = (int)sys_get_uptime_us();
}
void backstopTimer(void)
{
    backStop = (int)sys_get_uptime_us() - backStart;
    backDistance = backStop/147;
    long yield = 0;
    //xSemaphoreGiveFromISR(sensor2, &yield);
    portYIELD_FROM_ISR(&yield);
}

void leftstartTimer(void)
{
    leftStart = (int)sys_get_uptime_us();
}
void leftstopTimer(void)
{
    leftStop = (int)sys_get_uptime_us() - leftStart;
    leftDistance = leftStop/147;
    long yield = 0;
    //xSemaphoreGiveFromISR(sensor3, &yield);
    portYIELD_FROM_ISR(&yield);
}

void rightstartTimer(void)
{
    rightStart = (int)sys_get_uptime_us();
}
void rightstopTimer(void)
{
    rightStop = (int)sys_get_uptime_us() - rightStart;
    rightDistance = rightStop/147;
    long yield = 0;
    //xSemaphoreGiveFromISR(sensor4, &yield);
    portYIELD_FROM_ISR(&yield);
}

void sendLEDmessage(int distance1, int distance2, int distance3)
{
	//get LED information from sensors
	uint8_t leftLED, frontLED, rightLED;

	if(leftDistance < 20)
	{
		leftLED = RED;
	}
	else if(leftDistance >=20 && leftDistance < 50)
	{
		leftLED = YELLOW;
	}
	else
	{
		leftLED = GREEN;
	}

	if(frontDistance < 20)
	{
		frontLED = RED;
	}
	else if(frontDistance >=20 && frontDistance < 50)
	{
		frontLED = YELLOW;
	}
	else
	{
		frontLED = GREEN;
	}

	if(rightDistance < 20)
	{
		rightLED = RED;
	}
	else if(rightDistance >=20 && rightDistance < 50)
	{
		rightLED = YELLOW;
	}
	else
	{
		rightLED = GREEN;
	}

	//combine LEDs into one byte
	uint8_t LEDmessage = leftLED | (frontLED>>2) | (rightLED>>4);

	static GPIO *chip_select = new GPIO(P1_19);
	chip_select->setAsOutput();
	chip_select->setLow();
	//send LED data
	ssp1_exchange_byte(LEDmessage);
	chip_select->setHigh();
}

void enableHeadlights()
{
	static GPIO *headlights = new GPIO(P0_26);
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

