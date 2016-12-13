/*
 * Motor_Servo_Control.cpp
 *
 *  Created on: Nov 15, 2016
 *      Author: SaurabhDeshmukh
 */
#include "Motor_Servo_Control.h"
#include "lpc_pwm.hpp"
#include <stdio.h>
#include "gpio.hpp"

using namespace std;

//#include <string>
//#include <string.h>
//
//using namespace std;

GPIO BL_RED_pin(P0_29);
GPIO BL_GREEN_pin(P0_30);
GPIO BL_IND_RIGHT_pin(P1_19);
GPIO BL_IND_LEFT_pin(P1_20);

static int count_initial=0;
string lcdServoDirection = "FORWARD";

void setServoControl(MASTER_DRIVING_CAR_t rcv_car);
void initMotorModuleSetup()
{
  BL_GREEN_pin.setAsOutput();
  BL_RED_pin.setAsOutput();
  BL_IND_LEFT_pin.setAsOutput();
  BL_IND_RIGHT_pin.setAsOutput();
}


string Motor_Servo_Set(MASTER_DRIVING_CAR_t rcv_car)
{
	static PWM motor(PWM::pwm2, 50);
	    motor.set(7.5);

	  if(rcv_car.MASTER_DRIVE_ENUM==DRIVE && rcv_car.MASTER_STEER_ENUM!=CENTER) //turning Condition
	   {
		   BL_GREEN_pin.setLow();
		   BL_RED_pin.setLow();
	   }

	   switch(rcv_car.MASTER_DRIVE_ENUM)
	   {
	       case REVERSE:
	           motor.set(7.5);
	           motor.set(7.2);
	           BL_GREEN_pin.setHigh();
			   BL_IND_LEFT_pin.setLow();
			   BL_IND_RIGHT_pin.setLow();
			   BL_RED_pin.setLow();
			   if(count_initial<6)
			   {
				   motor.set(8.05);
				   count_initial++;
			   }
			   else
			   {
				   switch(rcv_car.MASTER_SPEED_ENUM)
				   {
					   case LOW:
						 motor.set(7.9);
					   break;
					   case MEDIUM:
						 motor.set(8);
					   break;
					   case HIGH:
						 motor.set(8.09);
						 break;
					   default:
						 break;
				   }
			   }
			   setServoControl(rcv_car);
			   break;
	       case STOP:
	           motor.set(7.5);
	           BL_GREEN_pin.setLow();
	           BL_IND_LEFT_pin.setLow();
	           BL_IND_RIGHT_pin.setLow();
	           BL_RED_pin.setHigh();
	           lcdServoDirection = "STOP";
	           break;
	       case DRIVE:
			   BL_GREEN_pin.setHigh();
			   BL_IND_LEFT_pin.setLow();
			   BL_IND_RIGHT_pin.setLow();
			   BL_RED_pin.setLow();
			   if(count_initial<6)
			   {
				   motor.set(8.1);
				   count_initial++;
			   }
			   else
			   {
				   switch(rcv_car.MASTER_SPEED_ENUM)
				   {
					   case LOW:
						 motor.set(7.9);
					   break;
					   case MEDIUM:
						 motor.set(8);
					   break;
					   case HIGH:
						 motor.set(8.09);
						 break;
					   default:
						 break;
				   }
			   }
			   setServoControl(rcv_car);
	        break;

	        default:
	        break;
	    }
	   return lcdServoDirection;
}

void setServoControl(MASTER_DRIVING_CAR_t rcv_car)
{
	static PWM servo(PWM::pwm1, 50);
	 switch(rcv_car.MASTER_STEER_ENUM)
		   {
		   case FAR_RIGHT:
	           BL_IND_LEFT_pin.setLow();
	           BL_IND_RIGHT_pin.setHigh();
		       servo.set(9.2);
		       lcdServoDirection = "RIGHT";
		       break;
		   case RIGHT:
			   BL_IND_LEFT_pin.setLow();
			   BL_IND_RIGHT_pin.setHigh();
		       servo.set(8);
		       lcdServoDirection = "RIGHT";
		       break;
		   case CENTER:
		       servo.set(7.2);
		       lcdServoDirection = "FORWARD";
		       break;
		   case LEFT:
			   BL_IND_LEFT_pin.setHigh();
	           BL_IND_RIGHT_pin.setLow();
		       servo.set(6.1);
		       lcdServoDirection = "LEFT";
		       break;
		   case FAR_LEFT:
			   BL_IND_LEFT_pin.setHigh();
			   BL_IND_RIGHT_pin.setLow();
		       servo.set(5.1);
		       lcdServoDirection = "LEFT";
		       break;
		   }

}





