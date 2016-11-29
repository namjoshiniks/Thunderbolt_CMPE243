/*
 * gps.h
 *
 *  Created on: 29-Nov-2016
 *      Author: SAMIKSHA
 */

#ifndef L5_APPLICATION_PERIODIC_SCHEDULER_GPS_H_
#define L5_APPLICATION_PERIODIC_SCHEDULER_GPS_H_
#include <stdint.h>
#include "io.hpp"
#include "periodic_callback.h"
#include "uart2.hpp"
#include <stdio.h>
#include <cmath>
#include <stdlib.h>
#include <math.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "can.h"
#include "_can_dbc/generated_can.h"
#include <string>
#include <iostream>
#include "i2c2.hpp"
#include "utilities.h"
using namespace std;

/*
 * Initiate UART2
 */
void uart2_init();


/*
 * Converts degrees to radian
 */
double toRadian( double degree );

/*
 * Converts radian to degrees
 */
double toDegree( double radian);

/*
 * Calculate distance between to points
 * @lat1  Latitude of first point
 * @lat2  Latitude of second point
 * @long1 Longitude of first point
 * @long2 Longitude of second point
 */
double distanceCalculation(double lat1,double lat2,double long1,double long2);

/*
 * Calculate turning angle
 * @lat1  Latitude of first point
 * @lat2  Latitude of second point
 * @long1 Longitude of first point
 * @long2 Longitude of second point
 */
double bearing(double Lat1,double Lat2,double Long1,double Long2);
/*
 * CAN bus initialization
 */
void can_init_func();

/*
 * Send Heartbeat signal to Master
 */
void heartbeat();

/*
 * If CAN bus is off ,reset it
 */
void can_reset();

/*
 * Compass reading obtained by I2C
 */
void getCompassReadings();

/*
 * Get data from GPS through UART2
 */
void getDataFromGPS();

/*
 * Get valid values from GPS data
 */
void parseGPSdata();

/*
 * Send Current location to COM-BRIGDE
 */
void sendCurrentLocation();

/*
 * Receive and save checkpoints from COM_BRIDGE
 */
void recieveAndSaveCheckpoints();

/*
 * After Receiving and saving all checkpoints ,send acknowledgment to COM_BRIDGE
 */
void acknowledgmentTocb();

/*
 * Send distance and turning angle to Master
 */
void sendtoMaster();

/*
 * Compass reading obtained by I2C
 */
void getCompassReadings();

/*
 * Send Compass reading to COM_BRIDGE
 */
void sendCompassReadingToCom();

/*
 * Calibrate Compass if corresponding switch is pressed
 */
void compassCalibration();


#endif /* L5_APPLICATION_PERIODIC_SCHEDULER_GPS_H_ */
