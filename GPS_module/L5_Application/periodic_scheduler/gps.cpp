///*
// * gps.cpp
// *
// *  Created on: 28-Nov-2016
// *      Author: SAMIKSHA
// */
#include "gps.h"
#include <iostream>
#include "lpc_sys.h"
#include "LPC17xx.h"
#define EARTH_RADIUS 6384000
#define PI 3.14159
#define MIN_PER_DEGREE 60.0
#define SEC_PER_MINUTE 60.0

/* Temporary strings */
string temp1, temp2, temp3, temp4, temp5, temp6,temp11 , temp12 , temp13 ,temp14 , temp15, temp16;
string deg ="";

/* Com-bridge reset */
COM_BRIDGE_RESET_t resetFromCom {0};

/* Compass Reading to COM-BRIDGE */
GPS_COMPASS_HEADING_t compassTocom {0};

/* Current Location */
GPS_CURRENT_LOCATION_t com_data = { 0 };

/* GPS_ACKNOWLEDGEMENT to COM_BRIDGE */
GPS_ACKNOWLEDGEMENT_t ackToComBridge ={ 0 };

/* Data to be sent to Master from GPS */
GPS_MASTER_DATA_t master_data = {0};

/* Distance to final destination from current Location */
double finalDestinationDistance ;

/* Turn angle to be sent to master */
double turn_angle;

/* Distance to next Checkpoint */
double distanceToNextCheckpoint;

/* Total count initialized to check if first checkpoint received */
int TotalCountReceived=0;

/* Flag to check Checkpoint is received */
bool isCheckpointReceived = false;

/* CAN messages */
dbc_msg_hdr_t can_msg_hdr;
dbc_msg_hdr_t msg_hdr;
can_msg_t msg;
can_msg_t can_msg;


/* Flag to check if acknowledgment is sent to COM_BRIDGE */
bool AcknowledgmentSent=false;

/* Total checkpoint sent by COM_BRIDGE */
int TotalCheckpointCount = 0 ;

/* MIA handling of COM_BRIDGE */
const uint32_t COM_BRIDGE_RESET__MIA_MS = 3000;
const COM_BRIDGE_RESET_t COM_BRIDGE_RESET__MIA_MSG { 0 };
const uint32_t COM_BRIDGE_CHECK_POINT_m0__MIA_MS = 3000;
const uint32_t COM_BRIDGE_CHECK_POINT_m1__MIA_MS = 3000;
const COM_BRIDGE_CHECK_POINT_m0_t COM_BRIDGE_CHECK_POINT_m0__MIA_MSG = { 0 };
const COM_BRIDGE_CHECK_POINT_m1_t COM_BRIDGE_CHECK_POINT_m1__MIA_MSG = { 0 };

/* COM_BRIDGE checkpoint */
COM_BRIDGE_CHECK_POINT_t checkpoint_can_msg = { 0 };

/* GPS pointer */
char *gps;
/*IsFinal*/
uint8_t isFinal = 0;
/* Checkpoint Structure */
struct checkPoint
{
	double longitude;
	double lattitude;
	bool isLattitudeAdded = false;
	bool isLongitudeAdded = false;
	int TotalCount;
};

/* Number of latitude */
int latitude_number = 0;

/* Longitude Number */
int longitude_number = 0;

/* Total Latitude Received */
int totalLatitudeReceived = 0;

/* Total Latitude Received */
int totalLongitudeReceived= 0;

/* Variable to manipulate GPS data */
string str ="";
string currentLocationString;
char c = '-';
string lat1 = "";
string c1, c2;
double deg1, min1;
string long1 = "";
double latitude_dcm = 0;
double longitude_dcm = 0;
string token;

/* Checkpoint array */
checkPoint checkPoints[256] ;

/* Checkpoint number to send master */
int checkPointNumber = 0;

/* Compass variables */
double compass_reading;
int compass_last;
double final_Reading ;
uint8_t data;
uint8_t compass[2] = {0};
char reg_address ; //command register
char dev_address= 0xC0; //device address
uint16_t compass1;
uint16_t compass2;
double turn_angle1;
double turn_angle2;

/* Create i2c2 instance */
I2C2& i2c2 = I2C2::getInstance();

/* Create uart2 instance */
Uart2& u2 = Uart2::getInstance();

/* Initiate UART2 */
void uart2_init()
{
	u2.init(38400,100,100);

}

/* Converts degrees to radian */
double toRadian( double degree )
{
	const double halfC = PI / 180;
	return (degree * halfC);
}

/* Converts radian to degrees */
double toDegree( double radian)
{
	const double halfR = 180/PI;
	return (radian * halfR);
}




/* * Calculate distance between to points
 * @lat1  Latitude of first point
 * @lat2  Latitude of second point
 * @long1 Longitude of first point
 * @long2 Longitude of second point
 */
double distanceCalculation(double lat1,double lat2,double long1,double long2)
{
	double a,C;
	double Lat1 = toRadian(lat1);
	double Lat2 = toRadian(lat2);
	double Long1 = toRadian(long1);
	double Long2 = toRadian(long2);
	double dLat = Lat1 - Lat2 ;
	double dLong = Long1 - Long2;
	a = pow((sin(dLat)/2),2) + (cos (Lat1) * cos (Lat2) * pow((sin(dLong)/2),2));
	C = 2 * atan2(sqrt(a),sqrt(1-a));
	double distance = EARTH_RADIUS *C;
	return(distance);
}


/*
 * Calculate turning angle
 * @lat1  Latitude of first point
 * @lat2  Latitude of second point
 * @long1 Longitude of first point
 * @long2 Longitude of second point
 */
double bearing(double Lat1,double Lat2,double Long1,double Long2)
{
	double dLong = Long2 - Long1;
	double arg1 = sin (dLong) * cos (Lat2);
	double arg2 = ((cos (Lat1) *  sin(Lat2)) - ( sin (Lat1) * cos(Lat2) * cos(dLong)));
	turn_angle1 = atan2(arg1 , arg2);
	turn_angle2 = toDegree(turn_angle1);

	if(turn_angle2 < 0)
	{
		turn_angle2 = 360 + turn_angle2;
	}
	turn_angle = turn_angle2 - final_Reading ;
	if(turn_angle > 180 )
	{
		turn_angle = turn_angle-360;
	}
	return (turn_angle);

}
/*
 * CAN bus initialization
 */
void can_init_func()
{

	CAN_init(can1,100,5,5,NULL,NULL);
	CAN_bypass_filter_accept_all_msgs();
	CAN_reset_bus(can1);
}

/*
 * Send Heartbeat signal to Master
 */
void heartbeat()
{
	GPS_HEARTBEAT_t heartbeat = { 0 };
	heartbeat.GPS_HEARTBEAT_UNSIGNED = GPS_HEARTBEAT_HDR.mid;
	can_msg = { 0 };
	msg_hdr = dbc_encode_GPS_HEARTBEAT(can_msg.data.bytes, &heartbeat);
	can_msg.msg_id = msg_hdr.mid;
	can_msg.frame_fields.data_len = msg_hdr.dlc;
	CAN_tx(can1, &can_msg, 0);
}
/*
 * If CAN bus is off ,reset it
 */
void can_reset()
{

	if(CAN_is_bus_off(can1))
	{
		LE.toggle(1);
		CAN_reset_bus(can1);
	}
}

/*
 * Get data from GPS through UART2
 */
void getDataFromGPS()
{
	gps = new char[75];
	u2.gets(gps,75,0);

}

/*
 * Get valid values from GPS data
 */
void parseGPSdata()
{
	stringstream ss(gps);
	int i = 1;
	string t1, t2;

	while (getline(ss,token, ','))
	{
		if (i == 3)
			lat1 = token;
		else if (i == 5)
			long1 = token;
		else if (i == 4 && token == "S")
			lat1.insert(0, 1,c);
		else if (i == 6  && token == "W")
			long1.insert(0, 1, c);
		i++;
	}

	delete gps;
}

/*
 * Convert values of Latitude and Longitude to decimal degrees
 */
void toDecimalDegrees()
{string min ="";
//Converting Latitude
if (lat1.length() > 9)
{
	c1  = lat1[0] ;
	temp1 = lat1[1];
	temp2 = lat1[2];
	deg = temp1 + temp2;
	temp1 = lat1[3];
	temp2 = lat1[4];
	temp3 = lat1[6];
	temp4 = lat1[7];
	temp5= lat1[8];
	temp6 = lat1[9];
	min = temp1 + temp2 + temp3 + temp4 + temp5 + temp6;
}
else
{

	temp1 = lat1[0];
	temp2 = lat1[1];
	deg = temp1 + temp2;
	temp1 = lat1[2];
	temp2 = lat1[3];
	temp3 = lat1[5];
	temp4 = lat1[6];
	temp5 = lat1[7];
	temp6 = lat1[8];
	min = temp1 + temp2 + temp3 + temp4 + temp5 + temp6;
}

istringstream(deg) >> deg1;
istringstream(min) >> min1;
min1 = (double)min1/10000;
latitude_dcm = deg1 + (min1 / MIN_PER_DEGREE);
if (c1=="-")
{
	latitude_dcm = -1 * latitude_dcm;
}
//Converting Longitude
if (long1.length() > 10)
{
	c2  = long1[0];
	temp11 = long1[1];
	temp12 = long1[2];
	temp13 = long1[3];
	deg = temp11 + temp12 + temp13;

	temp11 = long1[4];
	temp12 = long1[5];
	temp13 = long1[7];
	temp14 = long1[8];
	temp15= long1[9];
	temp16 = long1[10];
	min = temp11 + temp12 + temp13 + temp14 + temp15 + temp16;
}
else if(long1.length() == 10)
{
	temp11 = long1[0];
	temp12 = long1[1];
	temp13 = long1[2];
	deg = temp11 + temp12 + temp13;
	temp11 = long1[3];
	temp12 = long1[4];
	temp13 = long1[6];
	temp14 = long1[7];
	temp15= long1[8];
	temp16 = long1[9];
	min = temp11 + temp12 + temp13 + temp14 + temp15 + temp16;
}

istringstream(deg) >> deg1;
istringstream(min) >> min1;
min1 = (double)min1/10000;
longitude_dcm = deg1 + (min1 / MIN_PER_DEGREE);
if (c2 == "-")
{
	longitude_dcm = -1 * longitude_dcm;
}

currentLocationString ="";

}

/*
 * Send Current location to COM-BRIGDE
 */
void sendCurrentLocation()
{
	com_data.GPS_LATTITUDE_SIGNED = latitude_dcm;
	com_data.GPS_LONGITUDE_SIGNED = longitude_dcm;
	can_msg = { 0 };
	msg_hdr = dbc_encode_GPS_CURRENT_LOCATION(can_msg.data.bytes, &com_data);
	can_msg.msg_id = msg_hdr.mid;
	can_msg.frame_fields.data_len = msg_hdr.dlc;
	if(CAN_tx(can1, &can_msg, 0))
	{
		printf ("Current location \n lat= %lf \n long= %lf \n", latitude_dcm, longitude_dcm  );
	}

}

/*
 * Receive and save checkpoints from COM_BRIDGE
 */
void recieveAndSaveCheckpoints()
{
	/*
	 * Recieve checkpoints
	 */
	while(CAN_rx(can1, &msg, 0))
	{
		can_msg_hdr.dlc = msg.frame_fields.data_len;
		can_msg_hdr.mid = msg.msg_id;
		dbc_decode_COM_BRIDGE_CHECK_POINT(&checkpoint_can_msg ,msg.data.bytes, &can_msg_hdr);
		if(can_msg_hdr.mid == COM_BRIDGE_CHECK_POINT_HDR.mid)
		{
			TotalCheckpointCount = checkpoint_can_msg.m0.COM_BRIDGE_TOTAL_COUNT_UNSIGNED;
		}
		else if (can_msg_hdr.mid == COM_BRIDGE_RESET_HDR.mid)
		{
			sys_reboot();
		}


	}


	/*
	 * Save Latitude and Longitude
	 */

	if (checkpoint_can_msg.m0.COM_BRIDGE_LATTITUDE_SIGNED != 0)
	{
		latitude_number = checkpoint_can_msg.m0.COM_BRIDGE_CURRENT_COUNT_UNSIGNED;
		if(!(checkPoints[latitude_number].isLattitudeAdded))
		{
			checkPoints[latitude_number].isLattitudeAdded = true;
			checkPoints[latitude_number].TotalCount = checkpoint_can_msg.m0.COM_BRIDGE_TOTAL_COUNT_UNSIGNED;
			checkPoints[latitude_number].lattitude = checkpoint_can_msg.m0.COM_BRIDGE_LATTITUDE_SIGNED;
			totalLatitudeReceived++;
		}
		checkpoint_can_msg.m0.COM_BRIDGE_LATTITUDE_SIGNED = 0;
	}

	else if( checkpoint_can_msg.m1.COM_BRIDGE_LONGITUDE_SIGNED != 0 )
	{
		longitude_number = checkpoint_can_msg.m1.COM_BRIDGE_CURRENT_COUNT_UNSIGNED;
		if(!(checkPoints[longitude_number].isLongitudeAdded))
		{
			checkPoints[longitude_number].isLongitudeAdded = true;
			checkPoints[longitude_number].TotalCount = checkpoint_can_msg.m1.COM_BRIDGE_TOTAL_COUNT_UNSIGNED;
			checkPoints[longitude_number].longitude = checkpoint_can_msg.m1.COM_BRIDGE_LONGITUDE_SIGNED;
			totalLongitudeReceived++;
		}
		checkpoint_can_msg.m1.COM_BRIDGE_LONGITUDE_SIGNED = 0;
	}
}

/*
 * After Receiving and saving all checkpoints ,send acknowledgment to COM_BRIDGE
 */
void acknowledgmentTocb()
{

	if(totalLongitudeReceived == TotalCheckpointCount+1 && totalLatitudeReceived == TotalCheckpointCount+1 && TotalCheckpointCount)
	{

		ackToComBridge.GPS_ACKNOWLEDGEMENT_UNSIGNED = GPS_ACKNOWLEDGEMENT_HDR.mid;
		can_msg = { 0 };
		msg_hdr = dbc_encode_GPS_ACKNOWLEDGEMENT(can_msg.data.bytes, &ackToComBridge);
		can_msg.msg_id = msg_hdr.mid;
		can_msg.frame_fields.data_len = msg_hdr.dlc;
		CAN_tx(can1, &can_msg, 0);
		isCheckpointReceived = false;
		AcknowledgmentSent= true;
		totalLongitudeReceived = 0;
		totalLatitudeReceived = 0;

	}

}

/*
 * Send distance and turning angle to Master
 */
void sendtoMaster()
{
	if(AcknowledgmentSent )
	{
		double distanceToFinal = distanceCalculation(latitude_dcm,checkPoints[TotalCheckpointCount].lattitude,longitude_dcm,checkPoints[TotalCheckpointCount].longitude);
		master_data.GEO_DATA_DISTANCE_TO_FINAL_DESTINATION_SIGNED = distanceToFinal;
		master_data.GEO_DATA_DISTANCE_TO_NEXT_CHECKPOINT_SIGNED = distanceCalculation(latitude_dcm,checkPoints[checkPointNumber].lattitude,longitude_dcm,checkPoints[checkPointNumber].longitude);
		double distanceOfCheckpointToFinal = distanceCalculation(checkPoints[checkPointNumber].lattitude,checkPoints[TotalCheckpointCount].lattitude,checkPoints[checkPointNumber].longitude,checkPoints[TotalCheckpointCount].longitude);
		master_data.GEO_DATA_TURNANGLE_SIGNED = bearing(latitude_dcm,checkPoints[checkPointNumber].lattitude,longitude_dcm,checkPoints[checkPointNumber].longitude);
		distanceToNextCheckpoint = distanceCalculation(latitude_dcm,checkPoints[checkPointNumber].lattitude,longitude_dcm,checkPoints[checkPointNumber].longitude);

		if(distanceToFinal <= 3 )
		{
			isFinal = 1;
			master_data.GEO_DATA_ISFINAL_SIGNED = isFinal;

		}

		if(((master_data.GEO_DATA_DISTANCE_TO_NEXT_CHECKPOINT_SIGNED<=3) && !isFinal) ||((distanceOfCheckpointToFinal > distanceToFinal) && !isFinal) )
		{
			checkPointNumber++;
			LD.setNumber(checkPointNumber);
			master_data.GEO_DATA_DISTANCE_TO_NEXT_CHECKPOINT_SIGNED = distanceCalculation(latitude_dcm,checkPoints[checkPointNumber].lattitude,longitude_dcm,checkPoints[checkPointNumber].longitude);
			master_data.GEO_DATA_TURNANGLE_SIGNED = bearing(latitude_dcm,checkPoints[checkPointNumber].lattitude,longitude_dcm,checkPoints[checkPointNumber].longitude);
		}
		msg_hdr = dbc_encode_GPS_MASTER_DATA(can_msg.data.bytes, &master_data);
		can_msg.msg_id = msg_hdr.mid;
		can_msg.frame_fields.data_len = msg_hdr.dlc;
		CAN_tx(can1, &can_msg, 0);
	}
	if(isFinal)
	{
		LD.setRightDigit(9);
		LD.setLeftDigit(9);
	}
}


/*
 * Compass reading obtained by I2C
 */
void getCompassReadings()
{
	reg_address = 0x02;
	i2c2.readRegisters(dev_address, reg_address, compass, 16);
	compass1 = ((compass[0]<<8) | (compass[1] & 0xFF));
	compass_last = compass1%10;
	compass2 = (compass1/10) ;
	final_Reading = (int)compass2 + 0.1 * compass_last;
}

/*
 * Send Compass reading to COM_BRIDGE
 */
void sendCompassReadingToCom()
{
	compassTocom.GEO_DATA_COMPASS_HEADING_UNSIGNED = (double)((int)(final_Reading*10))/10;
	msg_hdr = dbc_encode_GPS_COMPASS_HEADING(can_msg.data.bytes, &compassTocom);
	can_msg.msg_id = msg_hdr.mid;
	can_msg.frame_fields.data_len = msg_hdr.dlc;
	CAN_tx(can1, &can_msg, 0);

}

