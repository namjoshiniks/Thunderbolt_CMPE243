/*
 * sensor.cpp
 *
 *  Created on: Oct 20, 2016
 *      Author: arthurnguyen
 */

#include "sensor.hpp"

#define RED 0x80
#define YELLOW 0xC0
#define GREEN 0x40

COM_BRIDGE_RESET_t com_can_msg_t;

int frontStart = 0;
int frontStop = 0;
int backStart = 0;
int backStop = 0;
int leftStart = 0;
int leftStop = 0;
int rightStart = 0;
int rightStop = 0;

int frontDistance = 0;
int backDistance = 0;
int leftDistance = 0;
int rightDistance = 0;

void initializeCAN()
{
	CAN_init(can1, 100, 4, 4, NULL, NULL);
	CAN_reset_bus(can1);
	CAN_bypass_filter_accept_all_msgs();
}

bool checkCANbus()
{
	//check CAN bus
	if(CAN_is_bus_off(can1))
	{
		CAN_reset_bus(can1);
		LE.on(4);
		return true;
	}
	return false;
}

void updateCANsonar(SENSOR_SONARS_t *CAN_sensor)
{
	SENSOR_SONARS_t sensor_data;
	sensor_data.SENSOR_SONARS_LEFT_UNSIGNED = leftDistance;
	sensor_data.SENSOR_SONARS_RIGHT_UNSIGNED = rightDistance;
	sensor_data.SENSOR_SONARS_FRONT_UNSIGNED = frontDistance;
	sensor_data.SENSOR_SONARS_BACK_UNSIGNED = backDistance;
	*CAN_sensor = sensor_data;
}


void frontstartTimer(void)
{
    frontStart = (int)sys_get_uptime_us();
}

void frontstopTimer(void)
{
   frontStop = (int)sys_get_uptime_us() - frontStart;
   frontDistance = frontStop/147;
}

void backstartTimer(void)
{
    backStart = (int)sys_get_uptime_us();
}
void backstopTimer(void)
{
    backStop = (int)sys_get_uptime_us() - backStart;
    backDistance = backStop/147;
}

void leftstartTimer(void)
{
    leftStart = (int)sys_get_uptime_us();
}
void leftstopTimer(void)
{
    leftStop = (int)sys_get_uptime_us() - leftStart;
    leftDistance = leftStop/147;
}

void rightstartTimer(void)
{
    rightStart = (int)sys_get_uptime_us();
}
void rightstopTimer(void)
{
    rightStop = (int)sys_get_uptime_us() - rightStart;
    rightDistance = rightStop/147;
}


void enableHeadlights()
{
	static GPIO *headlights = new GPIO(P1_22);
		headlights->setAsOutput();
		if(LS.getPercentValue() <= 10)
		{
			headlights->setHigh();
		}
		else
		{
			headlights->setLow();
		}
}

void receive_COM_reset()
{
	//Receiver code
		static can_msg_t can_msg;
		while (CAN_rx(can1, &can_msg, 0))
		{
			static dbc_msg_hdr_t COM_bridge_reset_can_msg;
			// Form the message header from the metadata of the arriving message
			dbc_msg_hdr_t can_msg_hdr;
			can_msg_hdr.dlc = can_msg.frame_fields.data_len;
			can_msg_hdr.mid = can_msg.msg_id;

			// Attempt to decode the message (brute force, but should use switch/case with MID)
			dbc_decode_COM_BRIDGE_RESET(&com_can_msg_t, can_msg.data.bytes, &can_msg_hdr);
			if(can_msg_hdr.mid == COM_BRIDGE_RESET_HDR.mid)
			{
				sys_reboot();
			}
		}
}

void sensor_send_heartbeat()
{
	LE.off(4);
	static SENSOR_HEARTBEAT_t sensor_heartbeat;
	sensor_heartbeat.SENSOR_HEARTBEAT_UNSIGNED = 336;

	can_msg_t can_msg = {0};

	// Encode the CAN message's data bytes, get its header and set the CAN message's DLC and length
	dbc_msg_hdr_t msg_hdr = dbc_encode_SENSOR_HEARTBEAT(can_msg.data.bytes, &sensor_heartbeat);
	can_msg.msg_id = msg_hdr.mid;
	can_msg.frame_fields.data_len = msg_hdr.dlc;
}
