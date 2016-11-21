/*
 * lcd.cpp
 *
 *  Created on: Nov 20, 2016
 *      Author: arthurnguyen
 */

#include "lcd.hpp"

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
	ssp0_exchange_byte('S'); //S
	ssp0_exchange_byte('p'); //p
	ssp0_exchange_byte('e'); //e
	ssp0_exchange_byte('e'); //e
	ssp0_exchange_byte('d'); //d
	ssp0_exchange_byte(':'); //:


	//set cursor row 2 (battery)
	ssp0_exchange_byte(0xFE);
	ssp0_exchange_byte(0x45);
	ssp0_exchange_byte(0x40);
	vTaskDelay(2);
	//write "Battery Percent: "
	ssp0_exchange_byte('B'); //B
	ssp0_exchange_byte('a'); //a
	ssp0_exchange_byte('t'); //t
	ssp0_exchange_byte('t'); //t
	ssp0_exchange_byte('e'); //e
	ssp0_exchange_byte('r'); //r
	ssp0_exchange_byte('y'); //y
	ssp0_exchange_byte(' '); //
	ssp0_exchange_byte('P'); //P
	ssp0_exchange_byte('e'); //e
	ssp0_exchange_byte('r'); //r
	ssp0_exchange_byte('c'); //c
	ssp0_exchange_byte('e'); //e
	ssp0_exchange_byte('n'); //n
	ssp0_exchange_byte('t'); //t
	ssp0_exchange_byte(':'); //:

	//set cursor row 3 (next coordinates)
	ssp0_exchange_byte(0xFE);
	ssp0_exchange_byte(0x45);
	ssp0_exchange_byte(0x14);
	vTaskDelay(2);
	//write "Next Coordinates: "
	ssp0_exchange_byte('N'); //N
	ssp0_exchange_byte('e'); //e
	ssp0_exchange_byte('x');
	ssp0_exchange_byte('t');
	ssp0_exchange_byte(' ');
	ssp0_exchange_byte('C');
	ssp0_exchange_byte('o');
	ssp0_exchange_byte('o');
	ssp0_exchange_byte('r');
	ssp0_exchange_byte('d');
	ssp0_exchange_byte('i');
	ssp0_exchange_byte('n');
	ssp0_exchange_byte('a');
	ssp0_exchange_byte('t');
	ssp0_exchange_byte('e');
	ssp0_exchange_byte('s');
	ssp0_exchange_byte(':');


	//set cursor row 4 (current direction)
	ssp0_exchange_byte(0x54);
	vTaskDelay(2);

}



