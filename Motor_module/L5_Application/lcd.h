#ifndef L5_APPLICATION_LCD_H_
#define L5_APPLICATION_LCD_H_
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "lpc_pwm.hpp"
#include "eint.h"
#include "stdio.h"
#include "io.hpp"
#include "uart2.hpp"
#include "uart3.hpp"
#include "utilities.h"
//#include "L5_Application/periodic_scheduler/periodic_callback.h"
#include <string>
#include <string.h>
using namespace std;

void scroll(string data);
void processLineOne(void);
void clearLineTwo(void);
void processLineTwo(int leftVal, int frontVal, int rightVal, int backVal);
void processLineThree(string direction, int speed);
void processLineFour(float, float, float);
void setupLcd(void);
void initGlcd(void);
void setupGlcd(char16_t a, char16_t b, char16_t c, char16_t d, char16_t e);
void refreshLcd(void);



#endif /* L5_APPLICATION_LCD_H_ */
