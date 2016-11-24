/*
 * lcd.cpp
 *
 *  Created on: Nov 20, 2016
 *      Author: arthurnguyen
 */

#include "lcd.hpp"

void writetoLCD(string data)
{
	//next0xFE 0x56
	int strlength = data.length();
	char currentChar;
	for(int i=0; i<strlength; i++)
	{
		//set char
		currentChar = data[i];

		//pass char data
		ssp0_exchange_byte(data[i]);

		//shift display
		ssp0_exchange_byte(0xFE);
		ssp0_exchange_byte(0x56);
	}
}

void initializeLCD()
{
	//initialize ssp0
	ssp0_init(1);

	//Turn on display
	ssp0_exchange_byte(0xFE);
	ssp0_exchange_byte(0x41);
	vTaskDelay(2);

	//set cursor row 1 (speed)
	ssp0_exchange_byte(0xFE);
	ssp0_exchange_byte(0x45);
	ssp0_exchange_byte(0x00);
	vTaskDelay(2);
	//Write "Speed: "
	writetoLCD("Speed: ");


	//set cursor row 2 (battery)
	ssp0_exchange_byte(0xFE);
	ssp0_exchange_byte(0x45);
	ssp0_exchange_byte(0x40);
	vTaskDelay(2);
	//write "Battery Percent: "
	writetoLCD("Battery Percent: ");

	//set cursor row 3 (next coordinates)
	ssp0_exchange_byte(0xFE);
	ssp0_exchange_byte(0x45);
	ssp0_exchange_byte(0x14);
	vTaskDelay(2);
	//write "Next Coordinates: "
	writetoLCD("Next Coordinates: ");


	//set cursor row 4 (current direction)
	ssp0_exchange_byte(0x54);
	vTaskDelay(2);
	writetoLCD("Current Direction: ");

}

void updateLCD(string speed, string battery, string coordinates, string direction)
{

	//go to speed data
	ssp0_exchange_byte(0xFE);
	ssp0_exchange_byte(0x45);
	ssp0_exchange_byte(0x13);

	//delete old speed data

	//go to speed data
	ssp0_exchange_byte(0xFE);
	ssp0_exchange_byte(0x45);
	ssp0_exchange_byte(0x53);

	//delete old battery data

	//go to speed data
	ssp0_exchange_byte(0xFE);
	ssp0_exchange_byte(0x45);
	ssp0_exchange_byte(0x27);

	//delete old coordinates data

	//go to speed data
	ssp0_exchange_byte(0xFE);
	ssp0_exchange_byte(0x45);
	ssp0_exchange_byte(0x67);

	//delete old direction data
}



