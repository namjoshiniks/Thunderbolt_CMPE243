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

void setLED(int &distance, uint8_t &LED)
{
	if(distance < 20)
	{
		LED = RED;
	}
	else if(distance >= 20 && distance<= 50)
	{
		LED = YELLOW;
	}
	else
	{
		LED = GREEN;
	}
}
void setLEDmessage(uint8_t &LED1, uint8_t &LED2, uint8_t &LED3, uint8_t &LEDmessage)
{
	LEDmessage = LED1 | (LED2>>2) | (LED3>>4);
}
