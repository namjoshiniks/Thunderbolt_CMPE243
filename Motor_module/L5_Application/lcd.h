#ifndef L5_APPLICATION_LCD_H_
#define L5_APPLICATION_LCD_H_
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "lpc_pwm.hpp"
#include "eint.h"
#include "stdio.h"
#include "io.hpp"
#include "uart2.hpp"
#include "utilities.h"
//#include "L5_Application/periodic_scheduler/periodic_callback.h"
#include <string>
using namespace std;

//Uart2 &u2 = Uart2::getInstance();

//char lcd_init;
//char lcd_clear;
//char cursor_right;
//char cursor_line_two;
//char cursor_line_one;
//char set_cursor;
//string x;
//int length;
//
//string forward_string;
//string left_string;
//string right_string;

void print_forward(void);
void print_left(void);
void print_right(void);
void setupLcd(void);
void refreshLcd(void);



#endif /* L5_APPLICATION_LCD_H_ */
