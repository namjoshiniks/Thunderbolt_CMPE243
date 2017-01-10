#include "Com_Bridge.h"
#include <stdint.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include "io.hpp"
#include "can.h"

#include <string>
#include <cstring>
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "gpio.hpp"
#include "uart2.hpp"
#include "uart3.hpp"
#include "utilities.h"
using namespace std;


bool disableflag = false;
bool isReset = false;

string Latlng = "";
string latlon[] = {"", ""};
string latlon1[] = {"37.337354#", "-121.882924$"};
string speedDistance[] = {"", ""};
string speedDistance1[] = {"27%", "50^"};
string turn1[] = {"R*","L*"};
string compassValue;
string compassValue1 = "13.6&";
long double currentLoc[2];
long double latitude[100];
long double longitude[100];
bool flagx;
bool flag1 = false;
bool flag2 = false;
bool flag3 = false;
bool currentLocationFlag = false;
bool continuousCurrentLocation = false;
bool isACK = false;
bool isInitialLocation = false;
bool isClickEnabled = false;
bool isStopEnabled = false;
bool myReset = false;
int previousCount;
COM_BRIDGE_CHECK_POINT_t m ={ 0 };
COM_BRIDGE_HEARTBEAT_t hb = { 0 };
COM_BRIDGE_STOPALL_t stopSig = { 0 };
GPS_CURRENT_LOCATION_t gpsData = { 0 };
GPS_ACKNOWLEDGEMENT_t gpsACK = { 0 };
COM_BRIDGE_CLICKED_START_t startSig = { 0 };
MOTOR_CAR_SPEED_t motorData = { 0 };
GPS_COMPASS_HEADING_t gpsCompass = { 0 };
MASTER_DRIVING_CAR_t masterDriving;
COM_BRIDGE_RESET_t resetCom = { 0 };
int count1 = -1;
int setCount = -1;
const uint32_t                             GPS_CURRENT_LOCATION__MIA_MS = 3000;
const GPS_CURRENT_LOCATION_t               GPS_CURRENT_LOCATION__MIA_MSG = { 0 };
const uint32_t                             GPS_ACKNOWLEDGEMENT__MIA_MS = 1500;
const GPS_ACKNOWLEDGEMENT_t                GPS_ACKNOWLEDGEMENT__MIA_MSG ={ 100 };
const uint32_t                             MOTOR_CAR_SPEED__MIA_MS = 3000;
const MOTOR_CAR_SPEED_t                MOTOR_CAR_SPEED__MIA_MSG={ 0 };
const uint32_t                             GPS_COMPASS_HEADING__MIA_MS = 3000;
const GPS_COMPASS_HEADING_t               GPS_COMPASS_HEADING__MIA_MSG = { 0 };
const uint32_t                             MASTER_DRIVING_CAR__MIA_MS = 6000;
const MASTER_DRIVING_CAR_t               MASTER_DRIVING_CAR__MIA_MSG = { STOP,CENTER,MEDIUM };


Uart3 *u3 = &(Uart3::getInstance());

can_msg_t can_msg = { 0 };

void init_periodic()
{
	CAN_init(can1,100,10,10,NULL,NULL);
	CAN_reset_bus(can1);
	CAN_bypass_filter_accept_all_msgs();
	u3->init(38400, 1000, 1000);
	u3->flush();
	setupBT();
}

void Heartbeat()
{
	if(CAN_is_bus_off(can1))
	{
		CAN_reset_bus(can1);
		LE.on(1);
	}
	else
	{
		LE.off(1);
	}
	hb.COM_BRIDGE_HEARTBEAT_UNSIGNED = COM_BRIDGE_HEARTBEAT_HDR.mid;
	can_msg = { 0 };
	dbc_msg_hdr_t msg_hdr = dbc_encode_COM_BRIDGE_HEARTBEAT(can_msg.data.bytes, &hb);
	can_msg.msg_id = msg_hdr.mid;
	can_msg.frame_fields.data_len = msg_hdr.dlc;
	bool fg = CAN_tx(can1, &can_msg, 0);
}

void DummyData(uint32_t count)
{
	if(count)
	{

		const char* s1 ;
		for(int j = 0; j < 2; j++)
		{
			s1 = latlon1[j].c_str();
			u3->putline(s1);
			s1 = speedDistance1[j].c_str();
			u3->putline(s1);
			cout << "HI" << endl;

		}
		s1 = compassValue1.c_str();
		u3->putline(s1);
	}
	if(count%2 == 0)
	{
		u3->putline("L*");
	}
	if(count%3 == 0)
	{
		u3->putline("R*");
	}

}

void MyReset(uint32_t count)
{
	if(myReset && count == previousCount + 4)
	{
		myReset = false;
		sys_reboot();
	}
}

void Reset(uint32_t count)
{
	if(isReset)
	{

		can_msg = { 0 };
		resetCom.COM_BRIDGE_RESET_UNSIGNED = COM_BRIDGE_RESET_HDR.mid;
		dbc_msg_hdr_t msg_hdr = dbc_encode_COM_BRIDGE_RESET(can_msg.data.bytes,&resetCom);
		can_msg.msg_id = msg_hdr.mid;
		cout << msg_hdr.mid << endl;
		can_msg.frame_fields.data_len = msg_hdr.dlc;
		CAN_tx(can1, &can_msg, 0);
		isReset = false;
		myReset = true;
		previousCount = count;
	}
}

void CAN_Receive()
{
	while(CAN_rx(can1, &can_msg, 0))
	{
		dbc_msg_hdr_t can_msg_hdr;
		can_msg_hdr.dlc = can_msg.frame_fields.data_len;
		can_msg_hdr.mid = can_msg.msg_id;
		if(can_msg_hdr.mid == GPS_CURRENT_LOCATION_HDR.mid)
		{
			dbc_decode_GPS_CURRENT_LOCATION(&gpsData, can_msg.data.bytes, &can_msg_hdr);
		}
		else if(can_msg_hdr.mid == GPS_ACKNOWLEDGEMENT_HDR.mid)
		{
			dbc_decode_GPS_ACKNOWLEDGEMENT(&gpsACK, can_msg.data.bytes, &can_msg_hdr);
			isACK = true;
		}
		else if(can_msg_hdr.mid == MOTOR_CAR_SPEED_HDR.mid)
		{
			dbc_decode_MOTOR_CAR_SPEED(&motorData,can_msg.data.bytes, &can_msg_hdr);
		}
		else if(can_msg_hdr.mid == GPS_COMPASS_HEADING_HDR.mid)
		{
			dbc_decode_GPS_COMPASS_HEADING(&gpsCompass,can_msg.data.bytes, &can_msg_hdr);
		}
		else if(can_msg_hdr.mid == MASTER_DRIVING_CAR_HDR.mid)
		{
			dbc_decode_MASTER_DRIVING_CAR(&masterDriving,can_msg.data.bytes, &can_msg_hdr );
		}

	}
}

void Handle_MiaMsg()
{
	dbc_handle_mia_GPS_CURRENT_LOCATION(&gpsData, 100);
	dbc_handle_mia_GPS_ACKNOWLEDGEMENT(&gpsACK, 100);
	dbc_handle_mia_MOTOR_CAR_SPEED(&motorData, 100);
	dbc_handle_mia_MASTER_DRIVING_CAR(&masterDriving, 100);
}

void MotorDecode(uint32_t count)
{
	//if(motorData.MOTOR_DISTANCE_FROM_START_POINT_UNSIGNED != 0 && motorData.MOTOR_SPEED_DATA_UNSIGNED != 0 && count%4 == 0)
	if(count)
	{
		const char* s2 ;
		ostringstream os1;
		os1 << fixed << (int)motorData.MOTOR_DISTANCE_FROM_START_POINT_UNSIGNED;
		speedDistance[0] = os1.str();
		ostringstream os2;
		os2 << fixed << (int) motorData.MOTOR_SPEED_DATA_UNSIGNED;
		speedDistance[1] = os2.str();
		speedDistance[0] = speedDistance[0] + "%";
		speedDistance[1] = speedDistance[1] + "^";
		//cout << "Distance: " << speedDistance[0]<< endl;
		//cout << "Speed: " << speedDistance[1]<< endl;
		s2 = speedDistance[0].c_str();
		u3->putline(s2);
		s2 = speedDistance[1].c_str();
		u3->putline(s2);
		//motorData.MOTOR_DISTANCE_FROM_START_POINT_UNSIGNED = 0;
		//motorData.MOTOR_SPEED_DATA_UNSIGNED = 0;

	}
}

void CompassDecode(uint32_t count)
{
	//if(gpsCompass.GEO_DATA_COMPASS_HEADING_UNSIGNED != 0 && count%5 == 0)
	//if(count)
	//{
		const char* s2 ;
		ostringstream os1;
		os1 <<  gpsCompass.GEO_DATA_COMPASS_HEADING_UNSIGNED;
		compassValue = os1.str();
		compassValue = compassValue + "&";
		//cout << "Compass Heading: " << compassValue << endl;
		LD.setNumber((int)gpsCompass.GEO_DATA_COMPASS_HEADING_UNSIGNED%100);
		s2 = compassValue.c_str();
		u3->putline(s2);
		//gpsCompass.GEO_DATA_COMPASS_HEADING_UNSIGNED = 0;
	//}
}

void GPSDecode()
{
	if(gpsData.GPS_LATTITUDE_SIGNED != 0 && gpsData.GPS_LONGITUDE_SIGNED != 0)
	{
		//printf(" Current Location = %f : %f\n", gpsData.GPS_LATTITUDE_SIGNED, gpsData.GPS_LONGITUDE_SIGNED );
		const char* s2 ;
		ostringstream os1;
		os1 << fixed << gpsData.GPS_LATTITUDE_SIGNED;
		latlon[0] = os1.str();
		ostringstream os2;
		os2 << fixed << gpsData.GPS_LONGITUDE_SIGNED;
		latlon[1] = os2.str();
		//Continue accepting current location and sending to android
		latlon[0] = latlon[0] + "#";
		latlon[1] = latlon[1] + "$";
		//cout << "Latitude " <<   gpsData.GPS_LATTITUDE_SIGNED << " Longitude " << gpsData.GPS_LONGITUDE_SIGNED << endl;
		s2 = latlon[0].c_str();
		u3->putline(s2);
		s2 = latlon[1].c_str();
		u3->putline(s2);
		gpsData.GPS_LATTITUDE_SIGNED = 0;
		gpsData.GPS_LONGITUDE_SIGNED = 0;
	}
}


void StartStopCheckpoint(uint32_t count)
{
	  if(isClickEnabled)
	  {
		if(count1 < 0 && gpsACK.GPS_ACKNOWLEDGEMENT_UNSIGNED == 100)
			count1 = setCount;
		if(gpsACK.GPS_ACKNOWLEDGEMENT_UNSIGNED != 0 && count1 >= 0 and !isACK)
		{
			 //printf("Set Count : %d\n", setCount);
			 //printf("Count : %d\n", count1);
			 m.m0.COM_BRIDGE_TOTAL_COUNT_UNSIGNED = setCount;
			 m.m0.COM_BRIDGE_CURRENT_COUNT_UNSIGNED = count1;
			 m.m0.COM_BRIDGE_LATTITUDE_SIGNED = latitude[count1];
			 //cout << "Latitude: " << latitude[count1] << endl;
			 can_msg = { 0 };
			 dbc_msg_hdr_t msg_hdr = dbc_encode_COM_BRIDGE_CHECK_POINT_m0(can_msg.data.bytes,&m.m0);
			 can_msg.msg_id = msg_hdr.mid;
			 can_msg.frame_fields.data_len = msg_hdr.dlc;
			 CAN_tx(can1, &can_msg, 0);

			 m.m1.COM_BRIDGE_TOTAL_COUNT_UNSIGNED = setCount;
			 m.m1.COM_BRIDGE_CURRENT_COUNT_UNSIGNED = count1;
			 m.m1.COM_BRIDGE_LONGITUDE_SIGNED = longitude[count1];
			 //cout << "Longitude: " << longitude[count1] << endl;
			 can_msg = { 0 };
			 msg_hdr = dbc_encode_COM_BRIDGE_CHECK_POINT_m1(can_msg.data.bytes,&m.m1);
			 can_msg.msg_id = msg_hdr.mid;
			 can_msg.frame_fields.data_len = msg_hdr.dlc;
			 CAN_tx(can1, &can_msg, 0);
			 count1--;
		}
		else
		{
			if(startSig.COM_BRIDGE_CLICKED_START_UNSIGNED && count%20 )
			{
			 can_msg = { 0 };
			 dbc_msg_hdr_t msg_hdr = dbc_encode_COM_BRIDGE_CLICKED_START(can_msg.data.bytes,&startSig);
			 can_msg.msg_id = msg_hdr.mid;
			 can_msg.frame_fields.data_len = msg_hdr.dlc;
			 CAN_tx(can1, &can_msg, 0);
			 //startSig.COM_BRIDGE_CLICKED_START_UNSIGNED = { 0 };
			}

		}
	   }
	 else
	 {
		   if(isStopEnabled && count%5)
		   {
			can_msg = { 0 };
			stopSig.COM_BRIDGE_STOPALL_UNSIGNED = COM_BRIDGE_STOPALL_HDR.mid;
			dbc_msg_hdr_t msg_hdr = dbc_encode_COM_BRIDGE_STOPALL(can_msg.data.bytes,&stopSig);
			can_msg.msg_id = msg_hdr.mid;
			can_msg.frame_fields.data_len = msg_hdr.dlc;
			CAN_tx(can1, &can_msg, 0);
			//isStopEnabled = false;
		   }
	 }

}

void setupBT()
{
	int i = 0;
	delay_ms(3000);
	while(1)
	{
		i++;

		if(i >= 3)
		{
			printf("BTSETUP");
			i = 0;
			break;
		}

	}
}

void BT(void *p)
{
	string temp = "";
	int i = 0;
	char c;
	cout << fixed;
	while(u3->getChar(&c, 100))
	{
		temp = " ";
		temp[0] = c;
		if(c == 'B')
		{
			isClickEnabled = true;
			startSig.COM_BRIDGE_CLICKED_START_UNSIGNED = COM_BRIDGE_CLICKED_START_HDR.mid;
			count1 = setCount;
			isStopEnabled = false;
			LE.on(4);
		}
		else if(c == 'E')
		{
			isClickEnabled = false;
			isStopEnabled = true;
			setCount = 0;
			LE.off(4);
		}
		else if(c == '#')
		{
			 istringstream(Latlng) >> latitude[i];
			 cout << i  << " Latitude: " << latitude[i] << endl;
			Latlng = "";
		}
		else if(c == '$')
		{
			istringstream(Latlng) >> longitude[i] ;
			cout << i << " Longitude: " << longitude[i] << endl;
			Latlng = "";
			i++;
			setCount++;
		}
		else if(c == 'R')
		{
			isReset = true;
		}
		else
		{
			Latlng = Latlng + temp;
		}
	}
}


