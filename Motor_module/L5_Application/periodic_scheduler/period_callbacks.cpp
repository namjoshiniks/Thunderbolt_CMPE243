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
MASTER_DRIVING_CAR_t rcv_car={STOP,CENTER,MEDIUM};;
SENSOR_SONARS_t sensor_data = {0};
GPS_MASTER_DATA_t gps_data = {0};
SENSOR_SONARS_t sensor_old_data = {0};

MOTOR_HEARTBEAT_t motor_heartbeat = {0};
MOTOR_CAR_SPEED_t motor_speed = {0};

const uint32_t      MASTER_DRIVING_CAR__MIA_MS = 3000;
const MASTER_DRIVING_CAR_t    MASTER_DRIVING_CAR__MIA_MSG = {DRIVE,CENTER,MEDIUM};
const uint32_t                          GPS_MASTER_DATA__MIA_MS = 3000;
const GPS_MASTER_DATA_t                 GPS_MASTER_DATA__MIA_MSG = {0};
const uint32_t                          SENSOR_SONARS__MIA_MS = 3000;
const SENSOR_SONARS_t                   SENSOR_SONARS__MIA_MSG = sensor_old_data;


can_msg_t msg={ 0 };
dbc_msg_hdr_t msg_hdr;

int wheel_rotation_count=0;
static int magnet_count = 0;
float Odometer = 0;
static float RPM_Speed=0;
string LCD_Movement;
bool final=false;
long int compassAngle;

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
    setupLcd();
    initGlcd();
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
	//setupGlcd(0x01, 0x07, 0x00, 0x00, (int)RPM_Speed);
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
//	printf("RPM: %f",RPM_Speed);

	if (rcv_car.MASTER_DRIVE_ENUM != STOP)
	{
		Odometer = (magnet_count/10) * 0.31;
		//LD.setNumber(char(Odometer));
	}
	LD.setNumber(((int)RPM_Speed)%100);

	wheel_rotation_count=0;
	if(final)
	{
        rcv_car.MASTER_DRIVE_ENUM=STOP;
        lcdCarDirection = Motor_Servo_Set(rcv_car);
        lcdCarDirection="DESTINATION";
        //printf("\nREACHED DESTINATION");
	}
	else
	{
		lcdCarDirection = Motor_Servo_Set(rcv_car);
	}

	if(count%5==0)
	{
//		gps_data.GEO_DATA_TURNANGLE_SIGNED = 90;
//		RPM_Speed = 24.8;
		compassAngle = 180 + (int)gps_data.GEO_DATA_TURNANGLE_SIGNED;
//		printf("compassAngle : %ld\n",compassAngle);

		if(compassAngle > 255)
		{
			//setupGlcd(0x01, 0x08, 0x01, 0x01,(char)compassAngle%100); //old
			//setupGlcd(0x01, 0x0A, 0x01, 0x00, 0x00);  //form 0
			setupGlcd(0x01, 0x08, 0x00, 0x01, ((char)compassAngle)%100); //new compass

		}
		else
		{
			//setupGlcd(0x01, 0x08, 0x01, 0x00, compassAngle); //old
			setupGlcd(0x01, 0x08, 0x00, 0x00, (char)compassAngle);
		}

		//setupGlcd(0x01, 0x10, 0x00, 0x00, (int)RPM_Speed); // old
		//setupGlcd(0x01, 0x07, 0x00, 0x00, 0x00); // form 1
		setupGlcd(0x01, 0x07, 0x00, 0x00, (int)RPM_Speed); //new angular
		setupGlcd(0x01, 0x0F, 0x00, 0x00, (int)RPM_Speed); //new speed digit

	}

    motor_speed.MOTOR_DISTANCE_FROM_START_POINT_UNSIGNED = Odometer;
    motor_speed.MOTOR_SPEED_DATA_UNSIGNED = RPM_Speed;
    msg_hdr = dbc_encode_MOTOR_CAR_SPEED(msg.data.bytes, &motor_speed);
    msg.msg_id = msg_hdr.mid;
    msg.frame_fields.data_len = msg_hdr.dlc;
    CAN_tx(can1, &msg, 0);
}

void period_100Hz(uint32_t count)
{
	while(CAN_rx(can1,&msg,0))
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
					sensor_old_data=sensor_data;
				}
				else if(msg_header.mid == GPS_MASTER_DATA_HDR.mid)
				{
					dbc_decode_GPS_MASTER_DATA(&gps_data, msg.data.bytes,&msg_header);
					if(gps_data.GEO_DATA_ISFINAL_SIGNED == 1)
						{
					     final=true;
						}
				}
				else if(msg_header.mid == COM_BRIDGE_RESET_HDR.mid)
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

