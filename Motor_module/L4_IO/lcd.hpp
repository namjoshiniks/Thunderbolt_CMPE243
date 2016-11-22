/*
 * lcd.hpp
 *
 *  Created on: Nov 20, 2016
 *      Author: arthurnguyen
 */

#ifndef L4_IO_LCD_HPP_
#define L4_IO_LCD_HPP_

#include "ssp0.h"
#include "tasks.hpp"
#include <string>
using namespace std;

void writetoLCD(string data);

void initializeLCD();

#endif /* L4_IO_LCD_HPP_ */
