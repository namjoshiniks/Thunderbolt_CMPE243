#include "Motor_Servo_Control.h"
#include "lcd.h"
#include "periodic_callback.h"
#include <stdint.h>
#include "io.hpp"
#include "can.h"
#include <stdio.h>
#include "utilities.h"
#include "eint.h"
#include "gpio.hpp"
#include <lpc_sys.h>
#include <iostream>
#include <cstring>

using namespace std;

const uint32_t PERIOD_TASKS_STACK_SIZE_BYTES = (512 * 4);

const uint32_t PERIOD_DISPATCHER_TASK_STACK_SIZE_BYTES = (512 * 3);
string lcdCarDirection = "FORWARD";
MASTER_DRIVING_CAR_t rcv_car;
SENSOR_SONARS_t sensor_data = {0};
GPS_MASTER_DATA_t gps_data = {0};

MOTOR_HEARTBEAT_t motor_heartbeat = {0};
MOTOR_CAR_SPEED_t motor_speed = {0};

const uint32_t      MASTER_DRIVING_CAR__MIA_MS = 3000;
const MASTER_DRIVING_CAR_t    MASTER_DRIVING_CAR__MIA_MSG = {STOP,CENTER,MEDIUM};
const uint32_t                          GPS_MASTER_DATA__MIA_MS = 3000;
const GPS_MASTER_DATA_t                 GPS_MASTER_DATA__MIA_MSG = {0};
const uint32_t                          SENSOR_SONARS__MIA_MS = 3000;
const SENSOR_SONARS_t                   SENSOR_SONARS__MIA_MSG = {8,8,8,8};


can_msg_t msg={ 0 };
dbc_msg_hdr_t msg_hdr;

int wheel_rotation_count=0;
static int magnet_count = 0;
float Odometer = 0;
static float RPM_Speed=0;
string LCD_Movement;

void RPMSpeed_Func()
{
 wheel_rotation_count++;
}

bool period_init(void)
{
	eint3_enable_port2(5,eint_rising_edge,RPMSpeed_Func);
    CAN_init(can1,100,10,10,NULL, NULL);
    CAN_bypass_filter_accept_all_msgs();
    CAN_reset_bus(can1);
    initMotorModuleSetup();
    rcv_car.MASTER_DRIVE_ENUM =STOP;
    rcv_car.MASTER_SPEED_ENUM =MEDIUM;
    rcv_car.MASTER_STEER_ENUM =CENTER;
    setupLcd();

    return true;
}

bool period_reg_tlm(void)
{
    return true;
}

void period_1Hz(uint32_t count)
{
	if(CAN_is_bus_off(can1))
    {
        CAN_reset_bus(can1);
    }
    msg_hdr = dbc_encode_MOTOR_HEARTBEAT(msg.data.bytes,&motor_heartbeat);
    msg.msg_id = msg_hdr.mid;
    msg.frame_fields.data_len = msg_hdr.dlc;
    msg.data.qword = msg_hdr.mid;
    CAN_tx(can1, &msg, 0);
    processLineOne();
    processLineTwo(sensor_data.SENSOR_SONARS_LEFT_UNSIGNED, sensor_data.SENSOR_SONARS_FRONT_UNSIGNED, sensor_data.SENSOR_SONARS_RIGHT_UNSIGNED, sensor_data.SENSOR_SONARS_BACK_UNSIGNED);
    processLineThree(lcdCarDirection, (int)RPM_Speed);
    processLineFour(gps_data.GEO_DATA_TURNANGLE_SIGNED, gps_data.GEO_DATA_DISTANCE_TO_NEXT_CHECKPOINT_SIGNED, gps_data.GEO_DATA_DISTANCE_TO_FINAL_DESTINATION_SIGNED);
}

void period_10Hz(uint32_t count)
{
	if(count<12)
	{
	   rcv_car.MASTER_DRIVE_ENUM = STOP;
	}
	if(!(count%5))
		RPM_Speed=wheel_rotation_count * 12;
	magnet_count+=wheel_rotation_count;
	printf("RPM: %f",RPM_Speed);

	if (rcv_car.MASTER_DRIVE_ENUM != STOP)
	{
		Odometer = (magnet_count/10) * 0.31;
		//LD.setNumber(char(Odometer));
	}
	LD.setNumber(((int)RPM_Speed)%100);

	wheel_rotation_count=0;
	lcdCarDirection = Motor_Servo_Set(rcv_car);

    motor_speed.MOTOR_DISTANCE_FROM_START_POINT_UNSIGNED = Odometer;
    motor_speed.MOTOR_SPEED_DATA_UNSIGNED = RPM_Speed;
    msg_hdr = dbc_encode_MOTOR_CAR_SPEED(msg.data.bytes, &motor_speed);
    msg.msg_id = msg_hdr.mid;
    msg.frame_fields.data_len = msg_hdr.dlc;
    CAN_tx(can1, &msg, 0);
}

void period_100Hz(uint32_t count)
{
	if(CAN_rx(can1,&msg,0))
	{
		dbc_msg_hdr_t msg_header;
		msg_header.mid=msg.msg_id;
		msg_header.dlc=msg.frame_fields.data_len;

		if(msg_header.mid == MASTER_DRIVING_CAR_HDR.mid)
		{
			dbc_decode_MASTER_DRIVING_CAR(&rcv_car,msg.data.bytes,&msg_header);
		}
		else if(msg_header.mid == SENSOR_SONARS_HDR.mid)
		{
			dbc_decode_SENSOR_SONARS(&sensor_data, msg.data.bytes,&msg_header);
		}
		else if(msg_header.mid == GPS_MASTER_DATA_HDR.mid)
		{
			dbc_decode_GPS_MASTER_DATA(&gps_data, msg.data.bytes,&msg_header);
		}
		if(msg_header.mid == COM_BRIDGE_RESET_HDR.mid)
		{
			sys_reboot();
		}
	}
    if(dbc_handle_mia_MASTER_DRIVING_CAR(&rcv_car,10))
		LE.setAll(15);
	else
		LE.setAll(0);
    dbc_handle_mia_SENSOR_SONARS(&sensor_data, 10);
    dbc_handle_mia_GPS_MASTER_DATA(&gps_data, 10);
}

// 1Khz (1ms) is only run if Periodic Dispatcher was configured to run it at main():
// scheduler_add_task(new periodicSchedulerTask(run_1Khz = true));
void period_1000Hz(uint32_t count)
{
}
