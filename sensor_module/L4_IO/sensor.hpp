/*
 * sensor.h
 *
 *  Created on: Oct 20, 2016
 *      Author: arthurnguyen
 */

#ifndef L5_APPLICATION_SENSOR_HPP_
#define L5_APPLICATION_SENSOR_HPP_

#include "tasks.hpp"
#include "gpio.hpp"
#include "ssp1.h"
#include "io.hpp"
#include <stdio.h>
#include "can.h"
#include "_can_dbc/generated_can.h"


extern int leftDistance;
extern int rightDistance;
extern int frontDistance;
extern int backDistance;

extern bool Sensorcomplete;

//Start and stop signal
extern bool startProcess;
//Time
extern int frontStart;
extern int frontStop;
extern int backStart;
extern int backStop;
extern int leftStart;
extern int leftStop;
extern int rightStart;
extern int rightStop;

void initializeCAN();

bool checkCANbus();

void updateCANsonar(SENSOR_SONARS_t *CAN_sensor);

bool initializeRX_1();
bool initializeRX_2();

void frontstartTimer(void);
void frontstopTimer(void);
void backstartTimer(void);
void backstopTimer(void);
void leftstartTimer(void);
void leftstopTimer(void);
void rightstartTimer(void);
void rightstopTimer(void);

void sendLEDmessage(int distance1, int distance2, int distance3);

void enableHeadlights();

#endif /* L5_APPLICATION_SENSOR_HPP_ */
