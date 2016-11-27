/*
 * master_module.cpp
 *
 *  Created on: Nov 23, 2016
 *      Author: Abhishek
 */
#include "master_module.h"

#define LEFT_MIN 		20
#define RIGHT_MIN 		20
#define FRONT_MIN 		20
#define LEFT_MIDDLE 	50
#define RIGHT_MIDDLE 	50
#define FRONT_MIDDLE 	50
#define LEFT_MAX 		200
#define RIGHT_MAX 		200
#define FRONT_MAX 		200

const uint32_t                             MOTOR_HEARTBEAT__MIA_MS = 3000;
const MOTOR_HEARTBEAT_t                    MOTOR_HEARTBEAT__MIA_MSG = {0};
const uint32_t                             SENSOR_HEARTBEAT__MIA_MS = 3000;
const SENSOR_HEARTBEAT_t                   SENSOR_HEARTBEAT__MIA_MSG = {0};
const uint32_t                             COM_BRIDGE_HEARTBEAT__MIA_MS = 3000;
const COM_BRIDGE_HEARTBEAT_t               COM_BRIDGE_HEARTBEAT__MIA_MSG = {0};
const uint32_t                             GPS_HEARTBEAT__MIA_MS = 3000;
const GPS_HEARTBEAT_t                      GPS_HEARTBEAT__MIA_MSG = {0};
const uint32_t                             SENSOR_SONARS__MIA_MS = 3000;
const SENSOR_SONARS_t                      SENSOR_SONARS__MIA_MSG = {8,8,8,8};
const uint32_t                             COM_BRIDGE_CLICKED_START__MIA_MS = 3000;
const COM_BRIDGE_CLICKED_START_t           COM_BRIDGE_CLICKED_START__MIA_MSG = {0};
const uint32_t                             COM_BRIDGE_STOPALL__MIA_MS = 3000;
const COM_BRIDGE_STOPALL_t                 COM_BRIDGE_STOPALL__MIA_MSG = {0};
const uint32_t                             GPS_MASTER_DATA__MIA_MS = 3000;
const GPS_MASTER_DATA_t                    GPS_MASTER_DATA__MIA_MSG = {0};


MOTOR_HEARTBEAT_t motor_heartbeat_status = {0};
SENSOR_HEARTBEAT_t sensor_heartbeat_status = {0};
COM_BRIDGE_HEARTBEAT_t com_bridge_heartbeat_status = {0};
GPS_HEARTBEAT_t gps_heartbeat_status = {0};
SENSOR_SONARS_t sensor_data = {0};
COM_BRIDGE_CLICKED_START_t com_bridge_start = {0};
COM_BRIDGE_STOPALL_t com_bridge_stop = {0};
MASTER_DRIVING_CAR_t motor_drive = {STOP, CENTER, MEDIUM};
GPS_MASTER_DATA_t gps_data = {0};

static can_msg_t rx_msg = {0};
static can_msg_t tx_msg = {0};

//static bool turn=0;

bool CAN_setup(can_t can, uint32_t baudrate_kbps, uint16_t rxq_size, uint16_t txq_size,
              can_void_func_t bus_off_cb, can_void_func_t data_ovr_cb)
{
	bool status = false;
	status = CAN_init(can, baudrate_kbps, rxq_size, txq_size, bus_off_cb, data_ovr_cb);
	if(status)
	{
		CAN_bypass_filter_accept_all_msgs();
		CAN_reset_bus(can);
	}
	return status;
}

void handle_can_reset(can_t can)
{
    // BUS RESET
    if(CAN_is_bus_off(can))
        CAN_reset_bus(can);
}

void handle_heartbeat_leds()
{
    if(motor_heartbeat_status.MOTOR_HEARTBEAT_UNSIGNED == MOTOR_HEARTBEAT_HDR.mid)
        LE.on(1);
    else
        LE.off(1);

    if(sensor_heartbeat_status.SENSOR_HEARTBEAT_UNSIGNED == SENSOR_HEARTBEAT_HDR.mid)
        LE.on(2);
    else
        LE.off(2);

    if(com_bridge_heartbeat_status.COM_BRIDGE_HEARTBEAT_UNSIGNED == COM_BRIDGE_HEARTBEAT_HDR.mid)
        LE.on(3);
    else
        LE.off(3);

    if(gps_heartbeat_status.GPS_HEARTBEAT_UNSIGNED == GPS_HEARTBEAT_HDR.mid)
        LE.on(4);
    else
        LE.off(4);
}

bool handle_start_stop_signal()
{
	bool start = false;
	if(com_bridge_start.COM_BRIDGE_CLICKED_START_UNSIGNED == COM_BRIDGE_CLICKED_START_HDR.mid)
	{
		com_bridge_start.COM_BRIDGE_CLICKED_START_UNSIGNED = 0;
		start = true;
	}

	if(com_bridge_stop.COM_BRIDGE_STOPALL_UNSIGNED == COM_BRIDGE_STOPALL_HDR.mid)
	{
		com_bridge_stop.COM_BRIDGE_STOPALL_UNSIGNED = 0;
		start = false;
	}

	return start;
}

void default_motor_state()
{
	//STOP
    motor_drive.MASTER_DRIVE_ENUM = STOP;
    motor_drive.MASTER_SPEED_ENUM =  MEDIUM;
    motor_drive.MASTER_STEER_ENUM = CENTER;
    LD.setNumber(0);
}

void handle_motors_from_sensor_data()
{
	if(sensor_data.SENSOR_SONARS_FRONT_UNSIGNED > 50)
	{
		if(sensor_data.SENSOR_SONARS_RIGHT_UNSIGNED>35)
		{
			if(sensor_data.SENSOR_SONARS_LEFT_UNSIGNED>35)
			{
		        //MOVE_FORWARD
		        motor_drive.MASTER_DRIVE_ENUM= DRIVE;
		        motor_drive.MASTER_SPEED_ENUM =  MEDIUM;
		        motor_drive.MASTER_STEER_ENUM = CENTER;
		        LD.setNumber(1);
			}
			else
			{
				//RIGHT
                motor_drive.MASTER_DRIVE_ENUM = DRIVE;
                motor_drive.MASTER_SPEED_ENUM =  MEDIUM;
                motor_drive.MASTER_STEER_ENUM = RIGHT;
                LD.setNumber(3);
			}
		}
		else if((sensor_data.SENSOR_SONARS_RIGHT_UNSIGNED>20) && (sensor_data.SENSOR_SONARS_RIGHT_UNSIGNED<35))
		{
			if(sensor_data.SENSOR_SONARS_LEFT_UNSIGNED>35)
			{
		        //LEFT
		        motor_drive.MASTER_DRIVE_ENUM= DRIVE;
		        motor_drive.MASTER_SPEED_ENUM =  MEDIUM;
		        motor_drive.MASTER_STEER_ENUM = LEFT;
		        LD.setNumber(2);
			}
			else
			{
	            //STOP
				default_motor_state();
			}
		}
	}
	else if((sensor_data.SENSOR_SONARS_FRONT_UNSIGNED > 20) &&  (sensor_data.SENSOR_SONARS_FRONT_UNSIGNED < 50))
	{
		if(sensor_data.SENSOR_SONARS_RIGHT_UNSIGNED>35)
		{
			if(sensor_data.SENSOR_SONARS_LEFT_UNSIGNED>35)
			{
				if(sensor_data.SENSOR_SONARS_LEFT_UNSIGNED > sensor_data.SENSOR_SONARS_RIGHT_UNSIGNED)
				{
					//FAR_LEFT
	                motor_drive.MASTER_DRIVE_ENUM = DRIVE;
	                motor_drive.MASTER_SPEED_ENUM =  MEDIUM;
	                motor_drive.MASTER_STEER_ENUM = FAR_LEFT;
	                LD.setNumber(4);
				}
				else
				{
					//FAR_RIGHT
	                motor_drive.MASTER_DRIVE_ENUM = DRIVE;
	                motor_drive.MASTER_SPEED_ENUM =  MEDIUM;
	                motor_drive.MASTER_STEER_ENUM = FAR_RIGHT;
	                LD.setNumber(5);
				}
			}
			else
			{
				//FAR_RIGHT
                motor_drive.MASTER_DRIVE_ENUM = DRIVE;
                motor_drive.MASTER_SPEED_ENUM =  MEDIUM;
                motor_drive.MASTER_STEER_ENUM = FAR_RIGHT;
                LD.setNumber(5);
			}
		}
		else if((sensor_data.SENSOR_SONARS_RIGHT_UNSIGNED>20) && (sensor_data.SENSOR_SONARS_RIGHT_UNSIGNED<35))
		{
			if(sensor_data.SENSOR_SONARS_LEFT_UNSIGNED>35)
			{
		        //FAR_LEFT
		        motor_drive.MASTER_DRIVE_ENUM= DRIVE;
		        motor_drive.MASTER_SPEED_ENUM =  MEDIUM;
		        motor_drive.MASTER_STEER_ENUM = FAR_LEFT;
		        LD.setNumber(4);
			}
			else
			{
	            //STOP
				default_motor_state();
			}
		}
	}
	else
	{
        //STOP
		default_motor_state();
	}
}

void handle_can_rx(can_t can)
{
	dbc_msg_hdr_t msg_header;
	if(CAN_rx(can, &rx_msg, 0))
	{
		msg_header.mid = rx_msg.msg_id;
		msg_header.dlc = rx_msg.frame_fields.data_len;

		if(msg_header.mid == MOTOR_HEARTBEAT_HDR.mid)
			dbc_decode_MOTOR_HEARTBEAT(&motor_heartbeat_status, rx_msg.data.bytes, &msg_header);
		else if(msg_header.mid == SENSOR_HEARTBEAT_HDR.mid)
			dbc_decode_SENSOR_HEARTBEAT(&sensor_heartbeat_status, rx_msg.data.bytes, &msg_header);
		else if(msg_header.mid == COM_BRIDGE_HEARTBEAT_HDR.mid)
			dbc_decode_COM_BRIDGE_HEARTBEAT(&com_bridge_heartbeat_status, rx_msg.data.bytes, &msg_header);
		else if(msg_header.mid == GPS_HEARTBEAT_HDR.mid)
			dbc_decode_GPS_HEARTBEAT(&gps_heartbeat_status, rx_msg.data.bytes, &msg_header);
		else if(msg_header.mid == SENSOR_SONARS_HDR.mid)
			dbc_decode_SENSOR_SONARS(&sensor_data, rx_msg.data.bytes, &msg_header);
		else if(msg_header.mid == COM_BRIDGE_CLICKED_START_HDR.mid)
			dbc_decode_COM_BRIDGE_CLICKED_START(&com_bridge_start, rx_msg.data.bytes, &msg_header);
		else if(msg_header.mid == COM_BRIDGE_STOPALL_HDR.mid)
			dbc_decode_COM_BRIDGE_STOPALL(&com_bridge_stop, rx_msg.data.bytes, &msg_header);
		else if(msg_header.mid == GPS_MASTER_DATA_HDR.mid)
		{
			dbc_decode_GPS_MASTER_DATA(&gps_data, rx_msg.data.bytes, &msg_header);
			printf("Final_dest = %d\n", gps_data.GEO_DATA_DISTANCE_TO_FINAL_DESTINATION_SIGNED);
			printf("Next_checkpointt = %d\n", gps_data.GEO_DATA_DISTANCE_TO_NEXT_CHECKPOINT_SIGNED);
			printf("Is final ?  = %d\n", gps_data.GEO_DATA_ISFINAL_SIGNED);
			printf("Turn angle  = %d\n", gps_data.GEO_DATA_TURNANGLE_SIGNED);
		}
	}
}

void handle_can_tx(can_t can)
{
	dbc_msg_hdr_t msg_header = dbc_encode_MASTER_DRIVING_CAR(tx_msg.data.bytes, &motor_drive);
	tx_msg.msg_id = msg_header.mid;
	tx_msg.frame_fields.data_len = msg_header.dlc;
	CAN_tx(can, &tx_msg, 0);
}

void handle_mia()
{
	dbc_handle_mia_MOTOR_HEARTBEAT(&motor_heartbeat_status, 100);
	dbc_handle_mia_SENSOR_HEARTBEAT(&sensor_heartbeat_status, 100);
	dbc_handle_mia_COM_BRIDGE_HEARTBEAT(&com_bridge_heartbeat_status, 100);
	dbc_handle_mia_GPS_HEARTBEAT(&gps_heartbeat_status, 100);
	// Incrementing time by 0 so that mia will always be in disable state (For time being. Need to give more thought on this)
	dbc_handle_mia_COM_BRIDGE_CLICKED_START(&com_bridge_start, 0);
	dbc_handle_mia_COM_BRIDGE_STOPALL(&com_bridge_stop, 0);
	dbc_handle_mia_SENSOR_SONARS(&sensor_data, 100);
	dbc_handle_mia_GPS_MASTER_DATA(&gps_data, 100);
}