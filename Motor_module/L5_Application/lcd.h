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
#include <string.h>
using namespace std;

void scroll(string data);
void processLineOne(void);
void clearLineTwo(void);
void processLineTwo(int leftVal, int frontVal, int rightVal, int backVal);
void processLineThree(char *direction, int speed);
void setupLcd(void);
void refreshLcd(void);



#endif /* L5_APPLICATION_LCD_H_ */
