/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * This contains the period callback functions for the periodic scheduler
 *
 * @warning
 * These callbacks should be used for hard real-time system, and the priority of these
 * tasks are above everything else in the system (above the PRIORITY_CRITICAL).
 * The period functions SHOULD NEVER block and SHOULD NEVER run over their time slot.
 * For example, the 1000Hz take slot runs periodically every 1ms, and whatever you
 * do must be completed within 1ms.  Running over the time slot will reset the system.
 */

#include <stdint.h>
#include "io.hpp"
#include "periodic_callback.h"
#include <stdio.h>
//#include "_can_dbc/generated_can.h"
#include "sensor.hpp"
#include "ssp1.h"
#include "eint.h"

#include "tlm/c_tlm_var.h"

/// This is the stack size used for each of the period tasks (1Hz, 10Hz, 100Hz, and 1000Hz)
const uint32_t PERIOD_TASKS_STACK_SIZE_BYTES = (512 * 4);

/**
 * This is the stack size of the dispatcher task that triggers the period tasks to run.
 * Minimum 1500 bytes are needed in order to write a debug file if the period tasks overrun.
 * This stack size is also used while calling the period_init() and period_reg_tlm(), and if you use
 * printf inside these functions, you need about 1500 bytes minimum
 */
const uint32_t PERIOD_DISPATCHER_TASK_STACK_SIZE_BYTES = (512 * 3);

/// Called once before the RTOS is started, this is a good place to initialize things once
bool period_init(void)
{
	//enable interrupt pins
	eint3_enable_port2(0, eint_rising_edge, frontstartTimer);
	eint3_enable_port2(0, eint_falling_edge, frontstopTimer);
	eint3_enable_port2(1, eint_rising_edge, backstartTimer);
	eint3_enable_port2(1, eint_falling_edge, backstopTimer);
	eint3_enable_port2(2, eint_rising_edge, leftstartTimer);
	eint3_enable_port2(2, eint_falling_edge, leftstopTimer);
	eint3_enable_port2(3, eint_rising_edge, rightstartTimer);
	eint3_enable_port2(3, eint_falling_edge, rightstopTimer);

	//initialize RX1 and RX2 for sensors
	//initializeRX_1();
	//initializeRX_2();

	//enable SSP1
    ssp1_init();

	//CAN initialization
    initializeCAN();

	return true; // Must return true upon success
}

/// Register any telemetry variables
bool period_reg_tlm(void)
{
	tlm_component *bucket = tlm_component_get_by_name("disk");
	TLM_REG_VAR(bucket, leftDistance, tlm_int);
	TLM_REG_VAR(bucket, rightDistance, tlm_int);
	TLM_REG_VAR(bucket, frontDistance, tlm_int);
	TLM_REG_VAR(bucket, backDistance, tlm_int);

    // Make sure "SYS_CFG_ENABLE_TLM" is enabled at sys_config.h to use Telemetry
    return true; // Must return true upon success
}


/**
 * Below are your periodic functions.
 * The argument 'count' is the number of times each periodic task is called.
 */

void period_1Hz(uint32_t count)
{
	//Check CAN bus
	if(checkCANbus());
	else
	{
		LE.off(4);
		static SENSOR_HEARTBEAT_t sensor_heartbeat;
		sensor_heartbeat.SENSOR_HEARTBEAT_UNSIGNED = 336;

		can_msg_t can_msg = {0};

		// Encode the CAN message's data bytes, get its header and set the CAN message's DLC and length
		dbc_msg_hdr_t msg_hdr = dbc_encode_SENSOR_HEARTBEAT(can_msg.data.bytes, &sensor_heartbeat);
		can_msg.msg_id = msg_hdr.mid;
		can_msg.frame_fields.data_len = msg_hdr.dlc;

		if(CAN_tx(can1, &can_msg, 0))
		 {
			//printf("Send heartbeat success\n");
		 }
		 else
		 {
			printf("Send heartbeat fail!\n");
		 }
	}

	enableHeadlights();
}

void period_10Hz(uint32_t count)
{
	static SENSOR_SONARS_t sonar_data;
//	sonar_data.SENSOR_SONARS_LEFT_UNSIGNED = leftDistance;
//	sonar_data.SENSOR_SONARS_RIGHT_UNSIGNED = rightDistance; //change back after checking wiring
//	sonar_data.SENSOR_SONARS_FRONT_UNSIGNED = frontDistance;
//	sonar_data.SENSOR_SONARS_BACK_UNSIGNED = backDistance;
	updateCANsonar(&sonar_data);
	//can_msg_t can_msg;
	can_msg_t can_msg = {0};

	// Encode the CAN message's data bytes, get its header and set the CAN message's DLC and length

	dbc_msg_hdr_t msg_hdr = dbc_encode_SENSOR_SONARS(can_msg.data.bytes, &sonar_data);
	can_msg.msg_id = msg_hdr.mid;
	can_msg.frame_fields.data_len = msg_hdr.dlc;

	// Queue the CAN message to be sent out
	if(CAN_tx(can1, &can_msg, 0))
	{
	   //printf("Send data success\n");
	}
	else
	{
		printf("Send data fail!\n");
	}

	sendLEDmessage(leftDistance, frontDistance, rightDistance);
}

void period_100Hz(uint32_t count)
{
	//Sensor triggering RX
	static int sensorCount = 0;
    static GPIO *Sensor_RX1 = new GPIO(P0_30);
    static GPIO *Sensor_RX2 = new GPIO(P0_29);
    Sensor_RX1->setAsOutput();
    Sensor_RX2->setAsOutput();
	if(sensorCount == 0)
	{
		Sensor_RX1->setHigh();
		Sensor_RX2->setLow();
	}
	else if(sensorCount == 4)
	{
		printf("DISTANCE 1 and 2 **************\n"); //testing
		printf("Front: %i\n", frontDistance);
		printf("Back: %i\n", backDistance);
		Sensor_RX1->setLow();
		Sensor_RX2->setHigh();
	}
	else if(sensorCount == 8)
	{
        printf("DISTANCE 3 and 4 ===============\n"); //testing
        printf("Left: %i\n", leftDistance); //testing don't need delay
        printf("Right: %i\n", rightDistance); //testing don't need delay
		sensorCount = -1;
	}
	sensorCount++;
}

// 1Khz (1ms) is only run if Periodic Dispatcher was configured to run it at main():
// scheduler_add_task(new periodicSchedulerTask(run_1Khz = true));
void period_1000Hz(uint32_t count)
{
}
