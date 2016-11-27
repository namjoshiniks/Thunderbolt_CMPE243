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

GPIO BL_RED_pin(P0_29);
GPIO BL_GREEN_pin(P0_30);
GPIO BL_IND_RIGHT_pin(P1_19);
GPIO BL_IND_LEFT_pin(P1_20);


static float Set_DC=7.9;
static int count_initial=0;

void initMotorModuleSetup()
{
  BL_GREEN_pin.setAsOutput();
  BL_RED_pin.setAsOutput();
  BL_IND_LEFT_pin.setAsOutput();
  BL_IND_RIGHT_pin.setAsOutput();
}


void Motor_Servo_Set(MASTER_DRIVING_CAR_t rcv_car,float RPM_speed)
{
	static PWM motor(PWM::pwm2, 50);
	    static PWM servo(PWM::pwm1, 50);
	    servo.set(7.2);
	    motor.set(7.5);

	    if(rcv_car.MASTER_DRIVE_ENUM==DRIVE && rcv_car.MASTER_STEER_ENUM!=CENTER) //turning Condition
	   {
		   BL_GREEN_pin.setLow();
		   BL_RED_pin.setLow();
	   }

	   switch(rcv_car.MASTER_STEER_ENUM)
	   {
	   case FAR_RIGHT:
           BL_IND_LEFT_pin.setLow();
           BL_IND_RIGHT_pin.setHigh();
	       servo.set(10);
	       break;
	   case RIGHT:
		   BL_IND_LEFT_pin.setLow();
		   BL_IND_RIGHT_pin.setHigh();
	       servo.set(8.2);
	       break;
	   case CENTER:
	       servo.set(7.2);
	       break;
	   case LEFT:
		   BL_IND_LEFT_pin.setHigh();
           BL_IND_RIGHT_pin.setLow();
	       servo.set(6.7);
	       break;
	   case FAR_LEFT:
		   BL_IND_LEFT_pin.setHigh();
		   BL_IND_RIGHT_pin.setLow();
	       servo.set(5.1);
	       break;
	   }

	   switch(rcv_car.MASTER_DRIVE_ENUM)
	   {
	       case REVERSE:
	           motor.set(7.9);
	           break;
	       case STOP:
	           motor.set(7.5);
	           BL_GREEN_pin.setLow();
	           BL_IND_LEFT_pin.setLow();
	           BL_IND_RIGHT_pin.setLow();
	           BL_RED_pin.setHigh();
	           break;
	       case DRIVE:
			   BL_GREEN_pin.setHigh();
			   BL_IND_LEFT_pin.setLow();
			   BL_IND_RIGHT_pin.setLow();
			   BL_RED_pin.setLow();
			   if(count_initial<8)
			   {
				   motor.set(8.1);
				   count_initial++;
			   }
			   else
			   {
				   switch(rcv_car.MASTER_SPEED_ENUM)
				   {
					   case LOW:
						 motor.set(7.9);   //120 RPM
					   break;
					   case MEDIUM:    //200-300 RPM

					 if(rcv_car.MASTER_STEER_ENUM != FAR_LEFT && rcv_car.MASTER_STEER_ENUM != FAR_RIGHT )
					 {
						   if(RPM_speed<250)
						   {
							   if(RPM_speed==0.0)
							   {
							   Set_DC+=0.005;
							   motor.set(Set_DC);
							   }
							   else
							   {
								   motor.set(Set_DC);
							   }

						   }
						   else
						   {
//							   if(RPM_speed>140&&RPM_speed<240)
//							   {
//								   motor.set(Set_DC);
//
//							   }
//							   else
//							   {
							      motor.set(7.5);
								   Set_DC=7.8;
							   							   //}

						   }
						   printf("\nSET_DC:%f    RPM:%f",Set_DC,RPM_speed);
					   break;
					   case HIGH:   // 240 RPM
						 motor.set(8.3);
					   break;

					   default:
					   break;
					 }
					 else
					 {
						 motor.set(7.9); //turning speed will be low
					 }
					}
			   }
	        break;

	        default:
	        break;
	    }
}






