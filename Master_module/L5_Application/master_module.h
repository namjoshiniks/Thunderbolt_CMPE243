/*
 * master_module.h
 *
 *  Created on: Nov 23, 2016
 *      Author: Abhishek
 */

#ifndef L5_APPLICATION_MASTER_MODULE_H_
#define L5_APPLICATION_MASTER_MODULE_H_

#include <stdio.h>
#include <stdint.h>
#include "io.hpp"
#include "_can_dbc/generated_can.h"
#include "can.h"

/**
 * Wrapper function to setup CAN module for communication.
 * Internally calls following sequence of functions:
 * 1. CAN_init
 * 2. CAN_bypass_filter_accept_all_msgs
 * 3. CAN_reset_bus
 **/
bool CAN_setup(can_t can, uint32_t baudrate_kbps, uint16_t rxq_size, uint16_t txq_size,
              can_void_func_t bus_off_cb, can_void_func_t data_ovr_cb);

void handle_can_reset(can_t can);

void default_motor_state();

void handle_motors_from_sensor_data();

bool handle_start_stop_signal();

void handle_heartbeat_leds();

void handle_mia();

void handle_can_rx(can_t can);

void handle_can_tx(can_t can);

#endif /* L5_APPLICATION_MASTER_MODULE_H_ */
